"""Pre-detach-check baseline (proposal, predetach_check_v1)."""

from __future__ import annotations

import inspect
import json
from math import pi
from pathlib import Path
from types import SimpleNamespace

import pytest

from morphology_planner import HybridState, load_catalog, make_method_planner
from morphology_planner.grid import OccupancyGrid
from morphology_planner.observability import FiducialStation, observed_by_any
from morphology_planner.planner import HybridPlan, HybridSegment
from morphology_planner.predetach_check import (
    ExcludingTransitionPolicy, SiteExclusionFilter, SiteRejection,
    dump_site_exclusions, parse_site_exclusions, port_point,
)
from morphology_planner.transition_policy import CoupledTransitionPolicy
from morphology_planner.transition_validation import Box3
from modular_robot_bringup.predetach import (
    CheckLedger, EstimateWindow, probe_plan, unseen, validate_dwell,
)
from modular_robot_benchmarks import predetach_check as P
from modular_robot_benchmarks.confirmatory_scenarios import make_confirmatory_scenario
from modular_robot_benchmarks.design import (
    METHODS, PredetachTrialSpec, StudyDesign, TrialSpec, trial_spec_from_dict,
)
from modular_robot_benchmarks.mission_batch import pending_trials
from modular_robot_benchmarks.records import TrialStore


ROOT = Path(__file__).parents[1]
CATALOG = ROOT / "src/modular_robot_description/config/morphologies.yaml"
STUDY = ROOT / "studies/predetach_check_v1"


@pytest.fixture(scope="module")
def catalog():
    return load_catalog(str(CATALOG)).supported_experiment_subset()


# --- existing behaviour is untouched ---------------------------------------

def test_existing_methods_validate_with_the_shipped_policy_itself(catalog):
    grid = OccupancyGrid(30, 30, 0.1)
    for method in METHODS:
        planner = make_method_planner(method, catalog, grid, heading_bins=8)
        search = getattr(planner.planner, "hybrid", planner.planner)
        assert search.transition_validator is planner.transition_policy
        assert isinstance(planner.transition_policy, CoupledTransitionPolicy)
        with pytest.raises(ValueError):
            make_method_planner(method, catalog, grid, heading_bins=8,
                                site_rejections=(SiteRejection(
                                    "compact_to_narrow", 1.0, 1.0, 0.0, ("pod_0",)),))


def test_methods_tuple_and_frozen_design_hashes_are_unchanged():
    assert METHODS == ("route_first_adaptation", "geometry_coupled",
                       "feasibility_coupled", "sensing_feasibility_coupled")
    known = {
        "confirmatory_v3": "c939f12699e1eca0e7f74bfe9f5dc0ef69343c92fe82c1ce95e5c6af1dad5734",
    }
    for path, envelope in _frozen_designs():
        design = StudyDesign.read_frozen(path)       # verifies the stored hash
        assert design.design_hash == envelope["design_hash"]
        if path.parent.name in known:
            assert design.design_hash == known[path.parent.name]
        if path.parent.name != "predetach_check_v1":
            assert all(type(spec) is TrialSpec for spec in design.trials)


def test_unchanged_feasibility_plan_on_a_confirmatory_layout(catalog):
    """feasibility_checked with no rejection plans exactly as feasibility_coupled."""
    source = StudyDesign.read_frozen(ROOT / "studies/confirmatory_v3/design.json")
    seed, _ = P.layout_world_seed("combined_constraints-05", source)
    scenario = make_confirmatory_scenario("combined_constraints", 5, seed, generator=3)
    start = HybridState(*scenario.start, 0, "compact_diff")
    plans = [P._plan(method, scenario, catalog, start, ())
             for method in ("feasibility_coupled", "feasibility_checked")]
    assert plans[0].states == plans[1].states
    assert [s.transition_id for s in plans[0].segments] == [
        s.transition_id for s in plans[1].segments]


def test_pending_sort_is_unchanged_for_plain_designs_and_run_order_for_predetach(tmp_path):
    source = StudyDesign.read_frozen(ROOT / "studies/confirmatory_v3/design.json")
    store = TrialStore(tmp_path)
    order = [s.trial_id for s in pending_trials(source, store)]
    legacy = [s.trial_id for s in sorted(
        source.trials, key=lambda s: (s.family, s.layout_id, s.replicate, s.method_order))]
    assert order == legacy
    design = StudyDesign.read_frozen(STUDY / "design.json")
    order = pending_trials(design, store)
    assert [s.run_order for s in order] == list(range(len(design.trials)))
    smoke = json.loads((STUDY / "smoke_trial_ids.json").read_text(encoding="utf-8"))
    only = pending_trials(design, store, smoke)
    assert [s.trial_id for s in only] == [s.trial_id for s in order[:len(smoke)]]
    with pytest.raises(ValueError):
        pending_trials(design, store, ["not-a-trial"])


# --- robot side uses only robot observations ---------------------------------

def test_check_reads_only_the_estimator_stream():
    """Ground-truth guard for the check: the robot-side code never names the
    occluders, the scenario, the station rule or simulator/evaluator truth."""
    forbidden = ("/evaluator", "module_pose", "topology_joint/pose", "core_world_",
                 "drive_diagnostics", "transition_obstacles", "observed_by_any",
                 "station_observes", "sight_screen", "fiducial_stations",
                 "load_environment_boxes", "pod_pose_samples")
    for path in (ROOT / "src/modular_robot_bringup/modular_robot_bringup/predetach.py",
                 ROOT / "src/modular_robot_bringup/modular_robot_bringup/hybrid_navigator.py"):
        text = path.read_text(encoding="utf-8")
        assert [m for m in forbidden if m in text] == [], path
    navigator = (ROOT / "src/modular_robot_bringup/modular_robot_bringup/"
                 "hybrid_navigator.py").read_text(encoding="utf-8")
    # The window is fed from the estimator's RelativePoseEstimate callback.
    sensing = navigator.split("def _on_sensing", 1)[1].split("def ", 1)[0]
    assert "self.estimate_window.add(" in sensing
    assert "message.connector_visible" in sensing


def test_planner_exclusion_takes_no_occluder_geometry():
    params = inspect.signature(SiteExclusionFilter.__init__).parameters
    assert set(params) == {"self", "catalog", "grid", "heading_bins", "stations",
                           "rejections"}
    source = (ROOT / "src/morphology_planner/morphology_planner/predetach_check.py"
              ).read_text(encoding="utf-8")
    assert "observed_by_any" not in source and "segment_intersects_box" not in source


def test_estimate_window_needs_a_fresh_visible_sample_and_fails_closed():
    window = EstimateWindow()
    window.add("pod_0", 10.0, True, ("fiducial",))       # before the window
    window.add("pod_1", 10.6, True, ("fiducial",))
    window.add("pod_1", 10.9, False, ("wheel_odometry",))
    window.add("pod_2", 10.7, False, ())
    window.add("pod_2", 10.8, True, ("fiducial",))
    seen = window.assess(("pod_0", "pod_1", "pod_2", "pod_3"), 10.5)
    assert {pod: v["seen"] for pod, v in seen.items()} == {
        "pod_0": False, "pod_1": False, "pod_2": True, "pod_3": False}
    assert unseen(seen) == ("pod_0", "pod_1", "pod_3")
    with pytest.raises(ValueError):
        validate_dwell(0.5)
    assert validate_dwell(1.0) == 1.0


def test_ledger_records_rejection_and_caps_attempts():
    ledger = CheckLedger("feasibility_checked", "ray_wedge", 0.3, 10.0, 1.0, 2)
    good = {"pod_0": {"seen": True}}
    bad = {"pod_0": {"seen": False}}
    assert ledger.record_check(1.0, "compact_to_narrow", (1, 2, 0), None, good, [])["passed"]
    assert not ledger.cap_reached
    event = ledger.record_check(2.0, "compact_to_narrow", (1, 2, 0), (1.01, 2, 0), bad, [])
    assert not event["passed"] and event["unseen_pods"] == ["pod_0"]
    assert ledger.rejections[0]["rule"] == "ray_wedge"
    # The rejection is exactly what the planner parses.
    parsed = parse_site_exclusions(json.dumps(ledger.rejections))
    assert parsed[0].unseen_pods == ("pod_0",) and parsed[0].rule == "ray_wedge"
    ledger.record_check(3.0, "compact_to_narrow", (2, 2, 0), None, bad, [])
    assert ledger.cap_reached and ledger.rejection_count == 2
    assert [e["event"] for e in ledger.events] == [
        "check", "check", "rejection", "check", "rejection"]
    with pytest.raises(ValueError):
        CheckLedger("feasibility_coupled", "radius", 0.3, 10.0, 1.0, 2)


# --- exclusion and replan ------------------------------------------------------

def test_radius_and_wedge_exclusion_geometry(catalog):
    grid = OccupancyGrid(60, 40, 0.1)
    station = FiducialStation("s", (1.0, 0.0, 0.6))
    transition = next(t for t in catalog.transitions if t.id == "compact_to_narrow")
    rejection = SiteRejection("compact_to_narrow", 2.05, 2.05, 0.0, ("pod_0",),
                              "radius", 0.3)
    radius = SiteExclusionFilter(catalog, grid, 8, (station,), (rejection,))
    assert radius.excluded_by(transition, HybridState(20, 20, 0, "compact_diff"))
    assert radius.excluded_by(transition, HybridState(22, 22, 3, "compact_diff"))
    assert not radius.excluded_by(transition, HybridState(24, 20, 0, "compact_diff"))
    wedge = SiteExclusionFilter(catalog, grid, 8, (station,),
                                (SiteRejection("compact_to_narrow", 2.05, 2.05, 0.0,
                                               ("pod_0",), "ray_wedge"),))
    # Same site: the unseen pod is on the axis at the axis range.
    assert wedge.excluded_by(transition, HybridState(20, 20, 0, "compact_diff"))
    # Further from the station along the sight line through pod_0: excluded.
    pod = port_point(catalog, "compact_diff", "pod_0", 2.05, 2.05, 0.0)
    axis = [p - s for p, s in zip(pod, station.position)]
    far = [p + 0.25 * a for p, a in zip(pod, axis)]
    cx, cy = far[0] - 0.23, far[1] - 0.20     # core so that pod_0 sits at `far`
    cell = grid.world_to_cell(cx, cy)
    x, y = grid.cell_center(*cell)
    moved = port_point(catalog, "compact_diff", "pod_0", x, y, 0.0)
    import math
    angle = math.degrees(math.acos(sum(
        (m - s) * a for m, s, a in zip(moved, station.position, axis)) / (
        math.dist(moved, station.position) * math.hypot(*axis))))
    expect = angle <= 10.0
    assert bool(wedge.excluded_by(transition, HybridState(*cell, 0, "compact_diff"))) == expect
    # Nearer the station than the unseen pod, off axis: not excluded.
    assert not wedge.excluded_by(transition, HybridState(12, 12, 0, "compact_diff"))
    assert dump_site_exclusions((rejection,)) and parse_site_exclusions("") == ()


def test_failed_check_excludes_the_site_and_the_replan_moves(catalog):
    source = StudyDesign.read_frozen(ROOT / "studies/confirmatory_v3/design.json")
    seed, _ = P.layout_world_seed("docking_observability-01", source)
    scenario = make_confirmatory_scenario("docking_observability", 1, seed, generator=3)
    start = HybridState(*scenario.start, 0, "compact_diff")
    first = P._plan("feasibility_checked", scenario, catalog, start, ())
    site = next(s for s in first.segments if s.kind == "reconfigure").source
    x, y = scenario.grid.cell_center(site.x, site.y)
    rejection = SiteRejection("compact_to_narrow", x, y, site.heading * 2 * pi / 8,
                              ("pod_0",), "radius", 0.3)
    second = P._plan("feasibility_checked", scenario, catalog, start, (rejection,))
    moved = next(s for s in second.segments if s.kind == "reconfigure").source
    mx, my = scenario.grid.cell_center(moved.x, moved.y)
    assert (mx - x) ** 2 + (my - y) ** 2 > 0.3 ** 2
    planner = make_method_planner(
        "feasibility_checked", catalog, scenario.grid, heading_bins=8,
        environment=scenario.transition_obstacles, site_rejections=(rejection,),
        stations=scenario.fiducial_stations)
    assert isinstance(planner.planner.transition_validator, ExcludingTransitionPolicy)
    planner.plan(start, scenario.goal, 2.5)
    reasons = {r for d in planner.transition_policy.decisions for r in d.reasons}
    assert "predetach_excluded:radius" in reasons


def _frozen_designs():
    """Every StudyDesign envelope under studies/ (other JSON named design.json skipped)."""
    for path in sorted((ROOT / "studies").rglob("design.json")):
        envelope = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(envelope, dict) and "design" in envelope and "design_hash" in envelope:
            yield path, envelope


def _hand_scenario():
    """A station at the origin and a tall box shadowing everything with x > 3."""
    grid = OccupancyGrid(80, 40, 0.1)
    station = FiducialStation("hand", (0.0, 2.0, 0.5))
    box = Box3("hand_screen", (3.0, 2.0, 0.3), (0.05, 4.0, 0.6))
    return SimpleNamespace(grid=grid, fiducial_stations=(station,),
                           transition_obstacles=(box,), start=(5, 20), goal=(75, 20))


def _scripted_plan(cells, catalog):
    segments = []
    for (x, y) in cells:
        s = HybridState(x, y, 0, "compact_diff")
        segments.append(HybridSegment("traverse", HybridState(5, 20, 0, "compact_diff"),
                                      s, 1.0, ""))
        segments.append(HybridSegment("reconfigure", s,
                                      HybridState(x, y, 0, "narrow_tandem"), 1.0,
                                      "compact_to_narrow"))
    return HybridPlan(tuple(), tuple(segments), 1.0, 0, 1.0, 0)


def test_planner_level_simulation_agrees_with_a_hand_worked_layout(catalog, monkeypatch):
    """Hand-worked: the screen at x=3.0 hides any point with x > 3.0 (height 0.06
    is below the 0.6 m screen top on every line from the 0.5 m station).

    Site 1, core x=2.95: pod_0/pod_1 compact ports at x=3.18 are hidden, so the
    check rejects. Site 2, core x=2.55: compact ports at x <= 2.78 are seen, but
    pod_0 docks at x=3.26: the check passes and the docking is refused at pod_0.
    The probed variant sees that and rejects site 2 too; site 3, core x=1.55,
    docks every pod at x <= 2.26 and completes.
    """
    scenario = _hand_scenario()
    script = {"feasibility_checked": [(29, 20), (25, 20)],
              "feasibility_probed": [(29, 20), (25, 20), (15, 20)]}
    calls = []

    def fake_plan(method, scenario_, catalog_, state, rejections):
        calls.append(len(rejections))
        if state.morphology == "narrow_tandem":
            return HybridPlan((), (), 0.0, 0, 1.0, 0)
        cells = script[method]
        return _scripted_plan([cells[len(rejections)]], catalog)

    monkeypatch.setattr(P, "_plan", fake_plan)
    monkeypatch.setattr(P, "probe_poses_free", lambda *a: {"all_free": True})
    checked = P.simulate_mission(scenario, catalog, "feasibility_checked", "radius")
    assert checked["outcome"] == "docking_refused" and checked["rejections"] == 1
    assert checked["checks"][0]["unseen_pods"] == ["pod_0", "pod_1"]
    assert checked["final_site"]["refused_pod"] == "pod_0"
    calls.clear()
    probed = P.simulate_mission(scenario, catalog, "feasibility_probed", "radius")
    assert probed["outcome"] == "completed" and probed["rejections"] == 2
    assert probed["checks"][1]["unseen_pods"] == []
    assert set(probed["checks"][1]["unseen_docked_pods"]) == {"pod_0", "pod_1"}
    assert calls[:3] == [0, 1, 2]


def test_probe_plan_covers_every_docked_point_of_compact_to_narrow(catalog):
    t = next(t for t in catalog.transitions if t.id == "compact_to_narrow")
    probes, unreachable = probe_plan(catalog.morphologies["compact_diff"].pod_poses,
                                     catalog.morphologies["narrow_tandem"].pod_poses,
                                     t.moved_pods)
    assert unreachable == ()
    assert {p.offset_m for p in probes} == {-0.48, -0.12, 0.12, 0.48}
    covered = {docked for p in probes for _, docked in p.pairs}
    assert covered == set(t.moved_pods)


# --- design and predictions ----------------------------------------------------

def test_design_shape_seeds_and_run_order():
    design = StudyDesign.read_frozen(STUDY / "design.json")
    assert design.design_kind == "predetach_check_v1"
    n_layouts = len(P.REFUSED_LAYOUTS) + len(P.CONTROL_LAYOUTS) + len(P.FRESH_LAYOUTS)
    assert len(design.trials) == n_layouts * len(P.CELLS)
    assert sorted(s.run_order for s in design.trials) == list(range(len(design.trials)))
    assert len({s.trial_id for s in design.trials}) == len(design.trials)
    source = StudyDesign.read_frozen(ROOT / "studies/confirmatory_v3/design.json")
    by_id = {s.trial_id: s for s in source.trials}
    others = [StudyDesign.read_frozen(p) for p, _ in _frozen_designs()
              if p.parent.name != "predetach_check_v1"]
    used = {getattr(s, f) for d in others for s in d.trials
            for f in ("world_seed", "sensing_seed", "friction_seed", "fault_seed")}
    for spec in design.trials:
        assert isinstance(spec, PredetachTrialSpec)
        if spec.arm == "same_layout":
            original = by_id[spec.source_trial_id]
            assert original.replicate == 0 and original.layout_id == spec.layout_id
            assert (spec.world_seed, spec.sensing_seed, spec.friction_seed,
                    spec.fault_seed) == (original.world_seed, original.sensing_seed,
                                         original.friction_seed, original.fault_seed)
        else:
            assert not {spec.world_seed, spec.sensing_seed, spec.friction_seed,
                        spec.fault_seed} & used
        assert trial_spec_from_dict(json.loads(json.dumps(
            spec.__dict__))) == spec
    smoke = json.loads((STUDY / "smoke_trial_ids.json").read_text(encoding="utf-8"))
    assert smoke == list(P.SMOKE_TRIAL_IDS) and 0 < len(smoke) <= 10
    ranks = {s.trial_id: s.run_order for s in design.trials}
    assert [ranks[t] for t in smoke] == list(range(len(smoke)))
    # Tiers are contiguous in run_order: smoke, checked, fresh, probed, references.
    tiers = [P.tier_of(s) for s in sorted(design.trials, key=lambda s: s.run_order)]
    assert tiers == sorted(tiers)
    # The frozen file is exactly what the generator builds.
    assert P.build_design(source).design_hash == design.design_hash


def test_refused_layout_list_matches_the_failure_table():
    table = json.loads((ROOT / "studies/confirmatory_v3/failure_table.json").read_text())
    counts = table["counts"]
    for family in ("docking_observability", "combined_constraints"):
        layouts = [l for l in P.REFUSED_LAYOUTS if l.startswith(family)]
        assert all(int(l.rsplit("-", 1)[1]) % 4 for l in layouts)
        assert counts[family]["feasibility_coupled"] == {"Docking": 3 * len(layouts),
                                                         "trials": 3 * len(layouts)}
    assert counts["neutral"]["feasibility_coupled"]["Completed"] == 45


def test_predictions_agree_with_a_direct_geometric_check(catalog):
    if not (STUDY / "predictions.json").exists():
        pytest.skip("predictions.json not generated yet (see RUNBOOK)")
    predictions = json.loads((STUDY / "predictions.json").read_text(encoding="utf-8"))
    design = StudyDesign.read_frozen(STUDY / "design.json")
    assert predictions["design_hash"] == design.design_hash
    specs = {s.trial_id: s for s in design.trials}
    assert set(predictions["cells"]) == set(specs)
    scenarios = {}
    checked = 0
    for trial, cell in predictions["cells"].items():
        spec = specs[trial]
        key = (spec.layout_id, spec.world_seed)
        if key not in scenarios:
            scenarios[key] = make_confirmatory_scenario(
                spec.family, int(spec.layout_id.rsplit("-", 1)[1]), spec.world_seed,
                generator=3)
        scenario = scenarios[key]
        for check in cell["checks"]:
            transition = next(t for t in catalog.transitions
                              if t.id == check["transition_id"])
            hidden = [pod for pod in transition.moved_pods
                      if observed_by_any(scenario.fiducial_stations,
                                         port_point(catalog, transition.source, pod,
                                                    check["x"], check["y"], check["yaw"]),
                                         scenario.transition_obstacles) is None]
            if "unseen_pods" in check:
                assert check["unseen_pods"] == hidden
                checked += 1
        site = cell["final_site"]
        if cell["outcome"] == "docking_refused":
            transition = next(t for t in catalog.transitions
                              if t.id == site["transition_id"])
            hidden = [pod for pod in transition.moved_pods
                      if observed_by_any(scenario.fiducial_stations,
                                         port_point(catalog, transition.target, pod,
                                                    site["x"], site["y"], site["yaw"]),
                                         scenario.transition_obstacles) is None]
            assert hidden and hidden[0] == site["refused_pod"]
    assert checked > 0
    # The reference arm reproduces confirmatory_v3: every refused layout's
    # feasibility_coupled cell is predicted refused (or unplannable), the
    # controls complete.
    for trial, cell in predictions["cells"].items():
        if cell["method"] != "feasibility_coupled" or cell["arm"] != "same_layout":
            continue
        if cell["layout_id"] in P.CONTROL_LAYOUTS:
            assert cell["outcome"] == "completed", trial
        else:
            assert cell["outcome"] in {"docking_refused", "no_plan"}, trial

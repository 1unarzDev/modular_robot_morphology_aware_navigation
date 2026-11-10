"""Offset-world pilot: world differs from the planner's prior; prior-map margin.

Pure Python. The zero-offset tests pin the bytes `origin/main` (00ac590) produced,
so a regression in the default path fails here rather than in a campaign.
"""

import hashlib
import json
from dataclasses import asdict
from pathlib import Path

import pytest

from morphology_planner import (
    FiducialStation, HybridState, OccupancyGrid, load_catalog,
    station_sensing_provider,
)
from morphology_planner.observability import observed_by_any
from morphology_planner.transition_policy import (
    SIGHT_SCREEN_NAME, _world_pose, prior_margin_maps,
)
from morphology_planner.transition_validation import Box3, Pose3
from modular_robot_benchmarks import offset_world_pilot as pilot
from modular_robot_benchmarks.confirmatory_scenarios import (
    apply_world_offset, make_confirmatory_scenario,
)
from modular_robot_benchmarks.design import (
    OFFSET_WORLD_PILOT_DESIGN_KIND, OffsetTrialSpec, StudyDesign, TrialSpec,
    layout_generator_for, trial_spec_from_dict,
)
from modular_robot_benchmarks.sdf_export import (
    export_confirmatory_sdf, prior_manifest_path,
)

ROOT = Path(__file__).parents[1]
CATALOG = ROOT / "src/modular_robot_description/config/morphologies.yaml"
STUDY = ROOT / "studies/offset_world_pilot_v1"
V3_DESIGN = ROOT / "studies/confirmatory_v3/design.json"
V3_HASH = "c939f12699e1eca0e7f74bfe9f5dc0ef69343c92fe82c1ce95e5c6af1dad5734"
# Layout used for the planning tests; wave-1 of the prior-map study planned
# nominal (27, 13) and inflate_0.04 (29, 12) here.
LAYOUT = ("docking_observability", 2)


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _v3_seed(layout_id):
    design = StudyDesign.read_frozen(V3_DESIGN)
    return next(s.world_seed for s in design.trials if s.layout_id == layout_id)


# SHA-256 of the SDF and manifest origin/main wrote (friction seed and
# generator as given), generated from a pristine export of commit 00ac590.
GOLDEN = {
    ("docking_observability", 2, 1237576661121503458, 11, 3): (
        "1da91d822d1d9d3f4799b90d80b330a2f2ba6d80827de6351da93b8ee9def15d",
        "a8fbbb09605864f90f8d0292f78fb1161b417a5581a5f36c18976b5ffe40682e"),
    ("combined_constraints", 1, 8, None, 1): (
        "164d831f3d05966dc75b363aae4c35959a1828031f3521525cf26cec2aa20837",
        "c2e512a1f5a5ea4a4a1cb90591e09fafd1b07f64a76bbc1358ccbc3a62eaa78a"),
}


@pytest.mark.parametrize("key", list(GOLDEN))
def test_zero_offset_export_is_byte_identical_to_main(tmp_path, key):
    family, index, seed, friction, generator = key
    for offset in (None, (0.0, 0.0, 0.0, 0.0)):
        world, manifest = export_confirmatory_sdf(
            family, index, seed, tmp_path / "w.sdf", CATALOG, friction,
            generator=generator, world_offset=offset)
        assert (_sha(world), _sha(manifest)) == GOLDEN[key]
        assert not prior_manifest_path(world).exists()
        (tmp_path / "w.sdf").unlink()
        (tmp_path / "w.manifest.json").unlink()


def test_existing_frozen_designs_keep_their_hash():
    designs = sorted((ROOT / "studies").glob("**/design.json"))
    assert designs
    for path in designs:
        if path.parent == STUDY:
            continue
        envelope = json.loads(path.read_text())
        if "design" not in envelope or "design_hash" not in envelope:
            continue  # not a StudyDesign file
        # read_frozen recomputes the hash from the dataclasses and raises on
        # any difference from the committed one.
        assert StudyDesign.read_frozen(path).design_hash == envelope["design_hash"]
    assert StudyDesign.read_frozen(V3_DESIGN).design_hash == V3_HASH


def test_plain_trial_specs_do_not_gain_fields():
    spec = StudyDesign.read_frozen(V3_DESIGN).trials[0]
    assert type(spec) is TrialSpec
    assert "world_offset" not in asdict(spec)
    assert trial_spec_from_dict(asdict(spec)) == spec


def test_offset_moves_only_the_screen_and_station_in_the_world():
    seed = _v3_seed("docking_observability-02")
    prior = make_confirmatory_scenario(*LAYOUT, seed, generator=3)
    assert apply_world_offset(prior, None) is prior
    assert apply_world_offset(prior, (0, 0, 0, 0)) is prior
    world = apply_world_offset(prior, (0.02, -0.03, 0.01, 0.005))
    old = {box.name: box for box in prior.transition_obstacles}
    new = {box.name: box for box in world.transition_obstacles}
    assert set(old) == set(new)
    screen_old, screen_new = old[SIGHT_SCREEN_NAME], new[SIGHT_SCREEN_NAME]
    assert screen_new.size == screen_old.size
    assert screen_new.center == pytest.approx(
        (screen_old.center[0] + 0.02, screen_old.center[1] - 0.03, screen_old.center[2]))
    assert all(new[name] == old[name] for name in old if name != SIGHT_SCREEN_NAME)
    station_old, = prior.fiducial_stations
    station_new, = world.fiducial_stations
    assert station_new.position == pytest.approx(
        (station_old.position[0] + 0.01, station_old.position[1] + 0.005,
         station_old.position[2]))
    for field in ("grid", "start", "goal", "observability_regions", "parameters"):
        assert getattr(world, field) == getattr(prior, field)


def test_offset_export_moves_world_and_leaves_the_planner_prior(tmp_path):
    seed = _v3_seed("docking_observability-02")
    offset = (0.04, 0.0, 0.0, 0.0)
    plain_world, plain_manifest = export_confirmatory_sdf(
        *LAYOUT, seed, tmp_path / "plain.sdf", CATALOG, 5, generator=3)
    world, manifest = export_confirmatory_sdf(
        *LAYOUT, seed, tmp_path / "off.sdf", CATALOG, 5, generator=3,
        world_offset=offset)
    plain = json.loads(plain_manifest.read_text())
    moved = json.loads(manifest.read_text())
    prior = json.loads(prior_manifest_path(world).read_text())
    screens = lambda m: {b["name"]: b for b in m["transition_obstacles"]}[SIGHT_SCREEN_NAME]
    assert screens(moved)["center"][0] == pytest.approx(screens(plain)["center"][0] + 0.04)
    assert moved["world_offset"] == {
        "screen_dx": 0.04, "screen_dy": 0.0, "station_dx": 0.0, "station_dy": 0.0}
    # The prior is the unperturbed manifest, tagged with its role.
    assert prior.pop("role") == "planner_prior"
    assert prior == plain
    # The SDF carries the displaced screen and nothing else differs.
    def model_centre(path):
        from xml.etree.ElementTree import parse
        node = parse(path).getroot().find(f"world/model[@name='{SIGHT_SCREEN_NAME}']")
        return [float(v) for v in node.findtext("pose").split()]
    assert model_centre(world)[0] == pytest.approx(model_centre(plain_world)[0] + 0.04)
    assert model_centre(world)[1:] == pytest.approx(model_centre(plain_world)[1:])
    assert _sha(world) != _sha(plain_world)


def test_offset_spec_roundtrips_and_is_recorded(tmp_path):
    spec = OffsetTrialSpec(
        "t", "docking_observability", "docking_observability-02", 0,
        "sensing_feasibility_coupled", 0, 1, 2, 3, 4,
        world_offset=(0.02, 0.0, 0.0, 0.0), prior_margin_m=0.04, world_condition="t02")
    restored = trial_spec_from_dict(json.loads(json.dumps(asdict(spec))))
    assert restored == spec and isinstance(restored, OffsetTrialSpec)
    assert layout_generator_for(OFFSET_WORLD_PILOT_DESIGN_KIND) == 3


# --- prior-map margin ------------------------------------------------------


def _synthetic():
    """A station behind a screen, and a core placed on the edge of its shadow."""
    catalog = load_catalog(CATALOG).supported_experiment_subset()
    transition = next(t for t in catalog.transitions if t.id == "compact_to_narrow")
    grid = OccupancyGrid(80, 40, 0.1)
    station = FiducialStation("s", (4.0, 0.15, 1.8))
    screen = Box3(SIGHT_SCREEN_NAME, (4.0, 1.0, 1.3), (1.0, 0.1, 1.4))
    return catalog, transition, grid, station, screen


def _first_visible_x(catalog, transition, grid, station, screen, margin):
    """Smallest site x (cells) at which every moved pod is visible."""
    provider = station_sensing_provider(
        catalog, (station,), (screen,), grid, 8, margin)
    for x in range(10, 79):
        sensing = provider(transition, HybridState(x, 20, 0, "compact_diff"))
        if all(state.connector_visible for state in sensing.values()):
            return x, provider
    raise AssertionError("no visible site")


def test_margin_zero_is_the_shipped_provider():
    catalog, transition, grid, station, screen = _synthetic()
    default = station_sensing_provider(catalog, (station,), (screen,), grid, 8)
    zero = station_sensing_provider(catalog, (station,), (screen,), grid, 8, 0.0)
    assert prior_margin_maps((station,), (screen,), 0.0) == [((station,), (screen,))]
    for x in range(10, 79, 3):
        state = HybridState(x, 20, 0, "compact_diff")
        assert default(transition, state) == zero(transition, state)


def test_margin_pushes_the_visible_site_away_from_the_shadow():
    catalog, transition, grid, station, screen = _synthetic()
    shipped_x, _ = _first_visible_x(catalog, transition, grid, station, screen, 0.0)
    robust_x, _ = _first_visible_x(catalog, transition, grid, station, screen, 0.04)
    assert robust_x >= shipped_x
    # Robust visibility implies shipped visibility everywhere, never the reverse.
    shipped = station_sensing_provider(catalog, (station,), (screen,), grid, 8)
    robust = station_sensing_provider(catalog, (station,), (screen,), grid, 8, 0.04)
    differs = 0
    for x in range(10, 79):
        state = HybridState(x, 20, 0, "compact_diff")
        left = all(s.connector_visible for s in shipped(transition, state).values())
        right = all(s.connector_visible for s in robust(transition, state).values())
        assert left or not right
        differs += left and not right
    assert differs > 0


def test_margin_inflates_only_the_screen_and_leaves_collision_inputs_alone():
    _, _, _, station, screen = _synthetic()
    shelf = Box3("raised_transition_shelf", (1.0, 1.0, 0.14), (0.6, 0.1, 0.08))
    maps = prior_margin_maps((station,), (screen, shelf), 0.04)
    assert len(maps) == 8  # nominal + inflated at 7 station poses
    for stations, boxes in maps[1:]:
        by_name = {box.name: box for box in boxes}
        assert by_name[shelf.name] == shelf
        assert by_name[SIGHT_SCREEN_NAME].size == pytest.approx(
            tuple(s + 0.08 for s in screen.size))
    assert maps[0] == ((station,), (screen, shelf))
    with pytest.raises(ValueError):
        prior_margin_maps((station,), (screen,), -0.01)


def test_shipped_site_unchanged_and_margin_moves_it_on_a_real_layout():
    catalog = load_catalog(CATALOG).supported_experiment_subset()
    prior = make_confirmatory_scenario(
        *LAYOUT, _v3_seed("docking_observability-02"), generator=3)
    shipped = pilot.plan_site(prior, catalog, 0.0)
    robust = pilot.plan_site(prior, catalog, 0.04)
    # (27, 13) is what the committed manipulation check records for this layout.
    report = json.loads((ROOT / "studies/confirmatory_v3/manipulation_check.json").read_text())
    entry = next(e for e in report["layouts"]
                 if (e["family"], e["layout_index"]) == LAYOUT)
    assert [shipped.x, shipped.y] == entry["decisions"][pilot.METHOD]["transition_site"]
    assert (robust.x, robust.y) == (29, 12)
    # The wave-1 prior-map study planned the same two sites.
    assert (shipped.x, shipped.y) != (robust.x, robust.y)


# --- committed pilot design and predictions ---------------------------------


@pytest.fixture(scope="module")
def committed():
    design = StudyDesign.read_frozen(STUDY / "design.json")
    predictions = json.loads((STUDY / "predictions.json").read_text())
    smoke = json.loads((STUDY / "smoke_trial_ids.json").read_text())
    return design, predictions, smoke


def test_pilot_design_shape_and_pairing(committed):
    design, predictions, smoke = committed
    assert design.design_kind == OFFSET_WORLD_PILOT_DESIGN_KIND
    assert predictions["design_hash"] == design.design_hash
    ids = [spec.trial_id for spec in design.trials]
    assert len(set(ids)) == len(ids) == design.expected_trials
    assert design.expected_trials == len(predictions["cells"])
    assert design.expected_trials == 10 * len({s.layout_id for s in design.trials})
    source = StudyDesign.read_frozen(V3_DESIGN)
    report = json.loads((ROOT / "studies/confirmatory_v3/manipulation_check.json").read_text())
    assert predictions["source_design_hash"] == source.design_hash
    expected_layouts = {f"{f}-{i:02d}" for f, i in pilot.eligible_layouts(report)}
    assert {s.layout_id for s in design.trials} == expected_layouts
    assert all(int(layout.rsplit("-", 1)[1]) % 4 for layout in expected_layouts)
    origin = {s.layout_id: s for s in source.trials
              if s.method == pilot.METHOD and s.replicate == 0}
    by_layout = {}
    for spec in design.trials:
        assert isinstance(spec, OffsetTrialSpec) and spec.replicate == 0
        for field in ("world_seed", "sensing_seed", "friction_seed", "fault_seed"):
            assert getattr(spec, field) == getattr(origin[spec.layout_id], field)
        by_layout.setdefault(spec.layout_id, []).append(spec)
    for specs in by_layout.values():
        assert {(s.prior_margin_m, s.world_condition) for s in specs} == {
            (m, c) for m in pilot.MARGINS_M for c, _, _ in pilot.WORLD_CONDITIONS}
        assert sorted(s.method_order for s in specs) == list(range(10))
        for s in specs:
            if s.world_condition == "ctl":
                assert s.world_offset == (0.0, 0.0, 0.0, 0.0)
    assert len(smoke) == 12 and set(smoke) <= set(ids)
    assert len({i.rsplit("-m", 1)[0] for i in smoke}) == 3


def test_offset_sizes_and_direction_signs(committed):
    design, predictions, _ = committed
    layouts = {item["layout_id"]: item for item in predictions["layouts"]}
    for spec in design.trials:
        norm = sum(v * v for v in spec.world_offset[:2]) ** 0.5
        magnitude = {c: m for c, m, _ in pilot.WORLD_CONDITIONS}[spec.world_condition]
        assert norm == pytest.approx(magnitude, abs=1e-9)
        assert spec.world_offset[2:] == (0.0, 0.0)
        direction = layouts[spec.layout_id]["direction_toward"]
        ux, uy = dict(pilot.AXIS_DIRECTIONS)[direction]
        if spec.world_condition in ("t02", "t04"):
            assert spec.world_offset[:2] == pytest.approx((ux * magnitude, uy * magnitude))
        if spec.world_condition == "a02":
            assert spec.world_offset[:2] == pytest.approx((-ux * magnitude, -uy * magnitude))


def test_committed_sites_match_the_manipulation_check(committed):
    _, predictions, _ = committed
    report = json.loads((ROOT / "studies/confirmatory_v3/manipulation_check.json").read_text())
    shipped = {(e["family"], e["layout_index"]): e["decisions"][pilot.METHOD]["transition_site"]
               for e in report["layouts"] if e["decisions"][pilot.METHOD]["planned"]}
    for layout in predictions["layouts"]:
        site = layout["site_margin_0"]
        assert [site["cell_x"], site["cell_y"]] == shipped[
            (layout["family"], layout["layout_index"])]


def test_prediction_file_matches_a_direct_geometric_check(committed):
    """Recompute every cell's verdict without the pilot module's helpers."""
    design, predictions, _ = committed
    catalog = load_catalog(CATALOG).supported_experiment_subset()
    transition = next(t for t in catalog.transitions if t.id == "compact_to_narrow")
    specs = {s.trial_id: s for s in design.trials}
    scenarios = {}
    for cell in predictions["cells"]:
        spec = specs[cell["trial_id"]]
        assert cell["world_offset"] == dict(zip(
            ("screen_dx", "screen_dy", "station_dx", "station_dy"), spec.world_offset))
        if cell["site"] is None:
            assert cell["predicted_transition"] == "no_plan"
            assert cell["predicted_terminal"] == "planning_failure"
            continue
        if spec.layout_id not in scenarios:
            scenarios[spec.layout_id] = make_confirmatory_scenario(
                spec.family, int(spec.layout_id.rsplit("-", 1)[1]), spec.world_seed,
                generator=3)
        prior = scenarios[spec.layout_id]
        (station,) = prior.fiducial_stations
        dx, dy = spec.world_offset[:2]
        boxes = tuple(
            Box3(b.name, (b.center[0] + dx, b.center[1] + dy, b.center[2]), b.size)
            if b.name == SIGHT_SCREEN_NAME else b
            for b in prior.transition_obstacles)
        wx, wy = prior.grid.cell_center(cell["site"]["cell_x"], cell["site"]["cell_y"])
        yaw = cell["site"]["heading"] * 2 * 3.141592653589793 / 8
        hidden = []
        for pod in transition.moved_pods:
            docked = _world_pose(catalog.morphologies[transition.target].pod_poses[pod],
                                 Pose3(wx, wy, 0.0, yaw), catalog.module_sizes[pod][2])
            if observed_by_any((station,), (docked.x, docked.y, docked.z), boxes) is None:
                hidden.append(pod)
        assert cell["hidden_pods"] == sorted(hidden)
        assert cell["predicted_transition"] == ("refused" if hidden else "accepted")
        assert cell["predicted_completion"] == (not hidden)
        # Zero offset at margin 0 is the shipped planner's own prediction.
        if spec.world_condition == "ctl":
            assert not hidden
        assert cell["confidence"] in ("firm", "marginal")


def test_margin_sites_sit_no_closer_to_the_shadow_along_the_toward_direction(committed):
    """Where a 4 cm-margin plan exists its site tolerates at least as large a
    screen shift toward the site as the shipped site does (None = unbounded)."""
    _, predictions, _ = committed
    catalog = load_catalog(CATALOG).supported_experiment_subset()
    checked = 0
    for layout in predictions["layouts"]:
        robust = layout["site_margin_0p04"]
        if robust is None:
            continue
        prior = make_confirmatory_scenario(
            layout["family"], layout["layout_index"], layout["world_seed"], generator=3)
        site = HybridState(robust["cell_x"], robust["cell_y"], robust["heading"],
                           "compact_diff")
        escape = pilot.escape_distances(prior, catalog, site)
        toward = layout["direction_toward"]
        shipped = layout["escape_distance_m"][toward]
        assert escape[toward] is None or escape[toward] >= shipped
        checked += 1
    assert checked >= 20


def test_committed_summary_is_what_the_cells_imply(committed):
    _, predictions, _ = committed
    assert predictions["summary"] == json.loads(json.dumps(
        pilot.summarize(predictions["cells"])))
    reference = predictions["summary"]["refusal_reference"]
    assert reference["t04_margin_0"]["p_zero_refusals"] < 0.001
    assert reference["predicted_safe_cells"]["q95"] <= 1


def test_poisson_binomial_law():
    law = pilot.refusal_distribution([0.5, 0.5])
    assert law == pytest.approx([0.25, 0.5, 0.25])


def _fake(spec, status, failures=0, site=None):
    from types import SimpleNamespace
    return SimpleNamespace(
        spec=spec, terminal_status=status, completed=status == "completed",
        reconfiguration_failures=failures,
        planned_transition_sites=[site] if site else [])


def test_score_counts_refusals_against_the_reference(committed):
    design, predictions, _ = committed
    records = []
    for spec in design.trials:
        cell = next(c for c in predictions["cells"] if c["trial_id"] == spec.trial_id)
        if cell["predicted_transition"] == "no_plan":
            records.append(_fake(spec, "planning_failure"))
            continue
        site = {"transition_id": pilot.QUALIFYING_TRANSITION,
                "x": (cell["site"]["cell_x"] + 0.5) * 0.1,
                "y": (cell["site"]["cell_y"] + 0.5) * 0.1}
        refused = cell["predicted_transition"] == "refused"
        records.append(_fake(spec, "unsafe_topology" if refused else "completed",
                             int(refused), site))
    result = pilot.score(predictions, records)
    assert result["site_mismatch"] == [] and result["unrelated_failures"] == []
    assert result["t04_margin_0"]["refused"] == 4
    assert result["t04_margin_0p04"]["refused"] == 0
    assert result["verdicts"]["controls_all_complete"] is True
    assert result["verdicts"]["margin_removes_loss_at_4cm"] is True
    # Four refusals sit inside the model's 3..9 interval for 4 cm toward.
    assert result["verdicts"]["toward_4cm_shipped_shows_loss"] is True
    assert pilot.score(predictions, [])["verdicts"]["controls_all_complete"] is None


def test_run_order_puts_the_interim_cells_first_and_smoke_has_the_best_refusal(committed):
    design, predictions, smoke = committed
    ordered = sorted(design.trials, key=lambda s: s.run_order)
    assert [s.run_order for s in ordered] == list(range(design.expected_trials))
    first = ordered[:pilot.INTERIM_TRIALS]
    assert {(s.prior_margin_m, s.world_condition) for s in first} == {
        (0.0, "t06"), (0.0, "ctl")}
    # Paired cells of a layout run back to back within a tier.
    assert all(a.layout_id == b.layout_id for a, b in zip(first[::2], first[1::2]))
    cells = {c["trial_id"]: c for c in predictions["cells"]}
    best = min((c for c in cells.values() if c["prior_margin_m"] == 0.0
                and c["world_condition"] == "t06"
                and cells[pilot.cell_id(c["layout_id"], 0.04, "t06")]["site"] is not None),
               key=lambda c: (c["p_transition_accepted"], c["layout_id"]))
    for margin in (0.0, 0.04):
        for condition in ("ctl", "t06"):
            assert pilot.cell_id(best["layout_id"], margin, condition) in smoke
    assert len(smoke) == 12

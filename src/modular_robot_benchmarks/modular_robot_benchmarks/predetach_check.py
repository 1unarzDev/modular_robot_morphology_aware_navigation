"""predetach_check_v1: design, planner-level simulation, predictions, scoring.

The study executes the pre-detach-check baseline (`feasibility_checked`, and its
docked-point probe variant `feasibility_probed`) in the simulator, on the
confirmatory_v3 layouts where `feasibility_coupled` was refused, plus controls
and a fresh-layout arm. See studies/predetach_check_v1/declaration.md.

`simulate_mission` is the planner-level model of what the navigator does, with
no ROS: plan with the method's planner (and the accumulated site rejections),
walk the plan's transition sites in order, and at each one evaluate the check
with the TRUE station rule (`observed_by_any` against the scenario's station
and 3D obstacles) at the NOMINAL placement (the planned cell centre and
heading, planner convention z = module height / 2). This is the model the
predictions are generated from; it is evaluator-side and never runs on the
robot. Its known blind spot (wave-4 sourcing addendum): at 11 of 90 refused
sites the robot's actual placement, a few centimetres off the cell centre, put a
pod in shadow that the nominal rule sees, so each predicted check also reports
whether its verdict holds under +/- `FRAGILITY_SHIFT_M` placement shifts.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import hashlib
import json
from math import cos, hypot, pi, sin
from pathlib import Path
from random import Random
import sys
from typing import Iterable

from morphology_planner import (
    HybridState, load_catalog, make_method_planner, observed_by_any,
    station_sensing_provider,
)
from morphology_planner.planner import NoPathError
from morphology_planner.predetach_check import SiteRejection, port_point

from .confirmatory_scenarios import make_confirmatory_scenario
from .design import (
    CHECKED_METHODS, METHODS, PREDETACH_CHECK_DESIGN_KIND, PredetachTrialSpec,
    StudyDesign, _seed, layout_generator_for,
)


DEFAULT_CATALOG = Path("src/modular_robot_description/config/morphologies.yaml")
STUDY_DIR = Path("studies/predetach_check_v1")
HEADING_BINS = 8          # morphology_planner.ros_node defaults
EPSILON = 2.5
MAX_ATTEMPTS = 8          # hybrid_navigator predetach_max_attempts default
DEADLINE_S = 300.0
FRAGILITY_SHIFT_M = 0.05
MAX_PROBE_OFFSET_M = 0.5

# The 30 confirmatory_v3 design layouts on which feasibility_coupled was
# refused (terminal_status unsafe_topology in all three replicates). Read from
# the committed records archive studies/confirmatory_v3/records/ (720 records):
# exactly the non-neutral docking_observability and combined_constraints
# layouts. `failure_table.json` agrees: feasibility_coupled has 45 "Docking"
# failures in each sensing family and 45 completions in the neutral stratum.
REFUSED_LAYOUTS = tuple(
    [f"docking_observability-{i:02d}" for i in
     (1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15, 17, 18, 19)]
    + [f"combined_constraints-{i:02d}" for i in
       (1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15, 17, 18, 19)])
# Controls, declared by rule: the lowest-index neutral layouts of the two
# sensing families, and the three lowest-index manipulated
# reconfiguration_workspace layouts (feasibility_coupled completed all of them).
CONTROL_LAYOUTS = (
    "docking_observability-00", "combined_constraints-00", "docking_observability-04",
    "reconfiguration_workspace-01", "reconfiguration_workspace-02",
    "reconfiguration_workspace-03",
)
# Fresh-layout (confirmatory) arm: new world and plant seeds, indices outside
# confirmatory_v3's 0..19 and non-neutral (index % 4 != 0), no selection.
FRESH_MASTER_SEED = 20261109
FRESH_LAYOUTS = tuple(
    f"{family}-{index:02d}" for family in ("docking_observability", "combined_constraints")
    for index in (21, 22, 23, 25, 26))
# (method, exclusion rule) cells, in the order a layout lists them.
CELLS = (
    ("feasibility_coupled", ""),
    ("feasibility_checked", "ray_wedge"),
    ("feasibility_checked", "radius"),
    ("feasibility_probed", "ray_wedge"),
    ("feasibility_probed", "radius"),
    ("sensing_feasibility_coupled", ""),
)
# Wall-time model for the hour estimate (min per trial), from confirmatory_v3
# records: a completed mission about 4.3 min of wall time plus setup/teardown,
# a docking refusal about 1.2 min, a planning failure about 0.2 min.
WALL_MIN = {"completed": 4.5, "docking_refused": 1.5, "no_plan": 0.5,
            "attempt_cap": 3.0, "deadline": 5.5}


def trial_id(layout_id: str, arm: str, method: str, rule: str) -> str:
    tag = "r0" if arm == "same_layout" else "fresh"
    return f"pdc-{layout_id}-{tag}-{method}" + (f"-{rule}" if rule else "")


# ---------------------------------------------------------------------------
# Planner-level simulation


def _visible(scenario, point) -> bool:
    return observed_by_any(scenario.fiducial_stations, point,
                           scenario.transition_obstacles) is not None


def _shifted(scenario, catalog, morphology, pod, x, y, yaw) -> bool:
    """Whether visibility of a port flips under any +/- shift of the placement."""
    nominal = _visible(scenario, port_point(catalog, morphology, pod, x, y, yaw))
    for dx, dy in ((FRAGILITY_SHIFT_M, 0.0), (-FRAGILITY_SHIFT_M, 0.0),
                   (0.0, FRAGILITY_SHIFT_M), (0.0, -FRAGILITY_SHIFT_M)):
        if _visible(scenario, port_point(catalog, morphology, pod, x + dx, y + dy,
                                         yaw)) != nominal:
            return True
    return False


def probe_poses_free(scenario, catalog, transition, x, y, yaw) -> dict:
    """Is every docked-point probe pose free for the source body?

    The same tests the planner applies to a driving state (occupancy footprint
    and raised drive obstacles), at the probe pose rather than a cell centre.
    """
    from modular_robot_bringup.predetach import offset_pose, probe_plan
    from morphology_planner.planner import (
        DRIVE_OBSTACLE_CLEARANCE_M, MorphologyAStar, inflated_footprint,
    )
    source = catalog.morphologies[transition.source]
    target = catalog.morphologies[transition.target]
    probes, unreachable = probe_plan(source.pod_poses, target.pod_poses,
                                     transition.moved_pods, MAX_PROBE_OFFSET_M)
    astar = MorphologyAStar(catalog, scenario.grid, HEADING_BINS,
                            drive_obstacles=scenario.transition_obstacles)
    result = []
    for probe in probes:
        px, py, pyaw = offset_pose(x, y, yaw, probe.offset_m)
        free = (scenario.grid.footprint_is_free(px, py, pyaw, source.footprint)
                and not astar._footprint_hits_drive_obstacle(
                    px, py, pyaw, source,
                    inflated_footprint(source.footprint, DRIVE_OBSTACLE_CLEARANCE_M)))
        result.append({"offset_m": probe.offset_m, "free": bool(free),
                       "pairs": [list(pair) for pair in probe.pairs]})
    return {"probes": result, "unprobeable_pods": list(unreachable),
            "all_free": all(p["free"] for p in result) and not unreachable}


_PLANNERS: dict = {}
_PLAN_MEMO: dict = {}


def _plan(method, scenario, catalog, state, rejections):
    """Plan as `morphology_planner.ros_node` would, reusing one planner per layout.

    The planner and its transition policy are built by `make_method_planner`
    once per (layout, policy) and reused, so their pure collision caches carry
    over; for the checked methods only the rejection wrapper is rebuilt per
    call, which is exactly what a fresh `make_method_planner` would install.
    """
    from morphology_planner.predetach_check import (
        ExcludingTransitionPolicy, SiteExclusionFilter,
    )
    kind = "feasibility_coupled" if method in CHECKED_METHODS else method
    key = (id(scenario), id(catalog), kind)
    cached = _PLANNERS.get(key)
    if cached is None or cached[0] is not scenario:
        planner = make_method_planner(
            kind, catalog, scenario.grid, heading_bins=HEADING_BINS,
            environment=scenario.transition_obstacles,
            sensing_provider=station_sensing_provider(
                catalog, scenario.fiducial_stations, scenario.transition_obstacles,
                scenario.grid, HEADING_BINS))
        cached = (scenario, planner)
        _PLANNERS[key] = cached
    planner = cached[1]
    memo_key = (key, state, tuple(rejections) if method in CHECKED_METHODS else ())
    if memo_key in _PLAN_MEMO:
        outcome = _PLAN_MEMO[memo_key]
        if isinstance(outcome, Exception):
            raise outcome
        return replace(outcome, method_id=method)
    policy = planner.transition_policy
    if method in CHECKED_METHODS and rejections:
        planner.planner.transition_validator = ExcludingTransitionPolicy(
            policy, SiteExclusionFilter(catalog, scenario.grid, HEADING_BINS,
                                        scenario.fiducial_stations, tuple(rejections)))
    else:
        planner.planner.transition_validator = policy
    try:
        result = planner.plan(state, scenario.goal, EPSILON)
    except (NoPathError, ValueError) as error:
        _PLAN_MEMO[memo_key] = error
        raise
    _PLAN_MEMO[memo_key] = result
    return result


def _leg_length(plan, scenario, upto_segment, start_segment=0) -> float:
    length = 0.0
    for segment in plan.segments[start_segment:upto_segment]:
        if segment.kind != "traverse":
            continue
        a = scenario.grid.cell_center(segment.source.x, segment.source.y)
        b = scenario.grid.cell_center(segment.target.x, segment.target.y)
        length += hypot(b[0] - a[0], b[1] - a[1])
    return length


def simulate_mission(scenario, catalog, method: str, rule: str = "",
                     max_attempts: int = MAX_ATTEMPTS, radius_m: float = 0.3,
                     wedge_deg: float = 10.0, probe: bool | None = None) -> dict:
    """Planner-level prediction of one mission (see the module docstring)."""
    checked = method in CHECKED_METHODS
    probe = (method == "feasibility_probed") if probe is None else probe
    state = HybridState(*scenario.start, 0, "compact_diff")
    rejections: list[SiteRejection] = []
    checks: list[dict] = []
    transitions = {t.id: t for t in catalog.transitions}
    travelled = 0.0
    plans = 0
    while True:
        try:
            plan = _plan(method, scenario, catalog, state, rejections)
        except (NoPathError, ValueError) as error:
            return _outcome("no_plan", checks, rejections, None, plans, travelled,
                            f"{type(error).__name__}: {error}")
        plans += 1
        rejected = False
        walked = 0
        for index, segment in enumerate(plan.segments):
            if segment.kind != "reconfigure":
                continue
            travelled += _leg_length(plan, scenario, index, walked)
            walked = index + 1
            verdict = _site(scenario, catalog, transitions[segment.transition_id],
                            segment, checked, probe)
            if verdict.get("check") is not None:
                checks.append(verdict["check"])
            if verdict["rejected"]:
                transition = transitions[segment.transition_id]
                rejections.append(SiteRejection(
                    transition.id, verdict["site"]["x"], verdict["site"]["y"],
                    verdict["site"]["yaw"], tuple(verdict["check"]["unseen_pods"]),
                    rule, radius_m, wedge_deg,
                    tuple(verdict["check"]["unseen_docked_pods"])))
                if len(rejections) >= max_attempts:
                    return _outcome("attempt_cap", checks, rejections, verdict["site"],
                                    plans, travelled)
                state = segment.source
                rejected = True
                break
            if verdict["refused_pod"] is not None:
                return _outcome("docking_refused", checks, rejections,
                                {**verdict["site"], "refused_pod": verdict["refused_pod"],
                                 "fragile": verdict["fragile"]}, plans, travelled)
            # Committed. The navigator replans from the new morphology here; the
            # model keeps the current plan's suffix (same planner, same
            # constraints), which avoids one planner call per transformation.
            if verdict.get("check") is None:
                checks.append({**verdict["site"], "committed": True,
                               "fragile": verdict["fragile"]})
            else:
                checks[-1]["committed"] = True
        if not rejected:
            travelled += _leg_length(plan, scenario, len(plan.segments), walked)
            return _outcome("completed", checks, rejections, None, plans, travelled)


def _site(scenario, catalog, transition, segment, checked: bool, probe: bool) -> dict:
    """The true-rule verdict at one planned site (nominal placement)."""
    x, y = scenario.grid.cell_center(segment.source.x, segment.source.y)
    yaw = segment.source.heading * 2.0 * pi / HEADING_BINS
    site = {"transition_id": transition.id, "x": x, "y": y, "yaw": yaw,
            "cell": [segment.source.x, segment.source.y, segment.source.heading]}
    source_seen = {pod: _visible(scenario, port_point(
        catalog, transition.source, pod, x, y, yaw)) for pod in transition.moved_pods}
    docked_seen = {pod: _visible(scenario, port_point(
        catalog, transition.target, pod, x, y, yaw)) for pod in transition.moved_pods}
    fragile = sorted(
        {pod for pod in transition.moved_pods
         if _shifted(scenario, catalog, transition.source, pod, x, y, yaw)}
        | {f"{pod}@docked" for pod in transition.moved_pods
           if _shifted(scenario, catalog, transition.target, pod, x, y, yaw)})
    check = None
    rejected = False
    if checked:
        unseen_source = [p for p in transition.moved_pods if not source_seen[p]]
        unseen_docked = ([p for p in transition.moved_pods if not docked_seen[p]]
                         if probe and not unseen_source else [])
        check = {**site, "source_seen": source_seen, "unseen_pods": unseen_source,
                 "unseen_docked_pods": unseen_docked,
                 "passed": not unseen_source and not unseen_docked,
                 "fragile": fragile}
        if probe:
            check["probe_geometry"] = probe_poses_free(
                scenario, catalog, transition, x, y, yaw)
        rejected = not check["passed"]
    refused = next((p for p in transition.moved_pods if not docked_seen[p]), None)
    return {"site": site, "check": check, "rejected": rejected,
            "refused_pod": refused, "fragile": fragile}


def _outcome(outcome, checks, rejections, site, plans, travelled, error=None):
    committed = [c for c in checks if c.get("committed")]
    value = {
        "outcome": outcome,
        "rejections": len(rejections),
        "checks": checks,
        "planning_calls": plans,
        "planned_drive_m": round(travelled, 3),
        # Mission time model (wave-4 fit, frozen): drive 6.41 s/m, 4.23 s per
        # leg, 101.9 s for the transformation and remainder after the good
        # site, plus 1.0 s dwell + 7.5 s replan per rejection. Startup excluded.
        "predicted_time_s": round(
            6.41 * travelled + 4.23 * (len(rejections) + 1) + 8.5 * len(rejections)
            + (101.9 if outcome == "completed" else 0.0), 1),
        "final_site": (site if site is not None
                       else (committed[0] if committed else None)),
    }
    if outcome == "completed" and value["predicted_time_s"] > DEADLINE_S:
        value["deadline_risk"] = True
    if error:
        value["error"] = error
    return value


# ---------------------------------------------------------------------------
# Design


def _source_design(path: Path) -> StudyDesign:
    design = StudyDesign.read_frozen(path)
    if design.design_kind != "confirmatory_v3":
        raise ValueError("the source design must be confirmatory_v3")
    return design


def layout_world_seed(layout_id: str, source: StudyDesign | None) -> tuple[int, dict]:
    family, index = layout_id.rsplit("-", 1)
    if layout_id in FRESH_LAYOUTS:
        index = int(index)
        world = _seed(FRESH_MASTER_SEED, PREDETACH_CHECK_DESIGN_KIND, "fresh", family,
                      index, "world")
        plant = {name: _seed(FRESH_MASTER_SEED, PREDETACH_CHECK_DESIGN_KIND, "fresh",
                             family, index, name.split("_")[0])
                 for name in ("sensing_seed", "friction_seed", "fault_seed")}
        return world, {**plant, "source_trial_id": ""}
    spec = next(s for s in source.trials
                if s.layout_id == layout_id and s.replicate == 0
                and s.method == "feasibility_coupled")
    return spec.world_seed, {"sensing_seed": spec.sensing_seed,
                             "friction_seed": spec.friction_seed,
                             "fault_seed": spec.fault_seed,
                             "source_trial_id": spec.trial_id}


def build_design(source: StudyDesign) -> StudyDesign:
    """The frozen predetach_check_v1 design (independent of the predictions)."""
    rows = []
    for arm, layouts in (("same_layout", REFUSED_LAYOUTS + CONTROL_LAYOUTS),
                         ("fresh", FRESH_LAYOUTS)):
        for layout_id in layouts:
            world_seed, seeds = layout_world_seed(layout_id, source)
            family = layout_id.rsplit("-", 1)[0]
            for order, (method, rule) in enumerate(CELLS):
                source_trial_id = seeds["source_trial_id"]
                if source_trial_id and method in METHODS:
                    source_trial_id = source_trial_id.replace(
                        "feasibility_coupled", method)
                rows.append(PredetachTrialSpec(
                    trial_id=trial_id(layout_id, arm, method, rule), family=family,
                    layout_id=layout_id, replicate=0, method=method,
                    method_order=order, world_seed=world_seed,
                    sensing_seed=seeds["sensing_seed"],
                    friction_seed=seeds["friction_seed"],
                    fault_seed=seeds["fault_seed"], exclusion_rule=rule, arm=arm,
                    source_trial_id=source_trial_id, run_order=0))
    ranks = run_order(rows)
    trials = tuple(replace(row, run_order=ranks[row.trial_id]) for row in rows)
    trials = tuple(sorted(trials, key=lambda t: t.run_order))
    return StudyDesign(
        1, PREDETACH_CHECK_DESIGN_KIND, len(REFUSED_LAYOUTS + CONTROL_LAYOUTS + FRESH_LAYOUTS),
        1, FRESH_MASTER_SEED, tuple(dict.fromkeys(m for m, _ in CELLS)),
        ("docking_observability", "combined_constraints", "reconfiguration_workspace"),
        trials)


# Layouts whose confirmatory_v3 records show the refused pod already hidden at
# its compact port at transition start (review r9 / wave-4 sourcing addendum:
# 26 of 90 refused records; combined-09 in replicates 0 and 2). The check is
# expected to fire there, so they run first within each tier. An ordering
# choice only; it fixes no outcome.
COMPACT_HIDDEN_LAYOUTS = (
    "docking_observability-01", "docking_observability-02", "docking_observability-03",
    "docking_observability-09", "docking_observability-10", "docking_observability-11",
    "docking_observability-17", "docking_observability-19", "combined_constraints-09",
)
# Smoke subset (run first, run_order 0..9), fixed by rule rather than by the
# predictions: the check where it should fire (both rules), where it should
# pass and the docking still be refused, the probe on both kinds, two
# controls, and one fresh layout.
SMOKE_TRIAL_IDS = (
    "pdc-docking_observability-01-r0-feasibility_checked-ray_wedge",
    "pdc-docking_observability-01-r0-feasibility_checked-radius",
    "pdc-docking_observability-02-r0-feasibility_checked-ray_wedge",
    "pdc-combined_constraints-09-r0-feasibility_checked-ray_wedge",
    "pdc-docking_observability-05-r0-feasibility_checked-ray_wedge",
    "pdc-docking_observability-05-r0-feasibility_probed-ray_wedge",
    "pdc-combined_constraints-01-r0-feasibility_probed-ray_wedge",
    "pdc-docking_observability-00-r0-feasibility_checked-ray_wedge",
    "pdc-reconfiguration_workspace-01-r0-feasibility_checked-ray_wedge",
    "pdc-docking_observability-21-fresh-feasibility_checked-ray_wedge",
)
TIERS = {0: "smoke", 1: "feasibility_checked, same layouts and controls (interim look after)",
         2: "fresh arm, every cell (confirmatory)", 3: "feasibility_probed, same layouts and controls",
         4: "references (feasibility_coupled, sensing_feasibility_coupled), same layouts and controls"}


def tier_of(spec) -> int:
    """Execution tier, see TIERS."""
    if spec.trial_id in SMOKE_TRIAL_IDS:
        return 0
    if spec.arm == "fresh":
        return 2
    if spec.method == "feasibility_checked":
        return 1
    if spec.method == "feasibility_probed":
        return 3
    return 4


def run_order(rows) -> dict[str, int]:
    smoke = {trial: rank for rank, trial in enumerate(SMOKE_TRIAL_IDS)}

    def key(spec) -> tuple:
        return (tier_of(spec), smoke.get(spec.trial_id, 0),
                0 if spec.layout_id in COMPACT_HIDDEN_LAYOUTS else 1,
                spec.layout_id, spec.method_order)
    ordered = sorted(rows, key=key)
    return {spec.trial_id: rank for rank, spec in enumerate(ordered)}


def predict(design_rows, catalog, generator: int = 3) -> dict:
    """Planner-level predictions for every cell (no trial has run)."""
    cells = {}
    cache = {}
    # Layout by layout, so one layout's planners and plans are reused across
    # its cells (the reference and checked methods share their first plan).
    for spec in sorted(design_rows, key=lambda s: (s.arm, s.layout_id, s.method_order)):
        index = int(spec.layout_id.rsplit("-", 1)[1])
        key = (spec.layout_id, spec.world_seed)
        if key not in cache:
            # One layout at a time: drop the previous layout's planners.
            cache.clear()
            _PLANNERS.clear()
            _PLAN_MEMO.clear()
            cache[key] = make_confirmatory_scenario(spec.family, index, spec.world_seed,
                                                    generator=generator)
        scenario = cache[key]
        sim = simulate_mission(scenario, catalog, spec.method, spec.exclusion_rule)
        print(f"predicted {spec.trial_id}: {sim['outcome']} "
              f"rejections={sim['rejections']}", file=sys.stderr, flush=True)
        cells[spec.trial_id] = {"layout_id": spec.layout_id, "arm": spec.arm,
                                "method": spec.method, "rule": spec.exclusion_rule,
                                **sim}
    _mark_changes(cells)
    return cells


def _mark_changes(cells: dict) -> None:
    for cell in cells.values():
        reference = next(
            (c for c in cells.values() if c["layout_id"] == cell["layout_id"]
             and c["arm"] == cell["arm"] and c["method"] == "feasibility_coupled"), None)
        cell["check_changes_outcome"] = bool(
            reference is not None and cell["method"] in CHECKED_METHODS
            and cell["outcome"] != reference["outcome"])


def summarize(cells: dict) -> dict:
    summary: dict = {}
    for cell in cells.values():
        group = "control" if cell["layout_id"] in CONTROL_LAYOUTS else cell["arm"]
        key = f"{cell['method']}" + (f"/{cell['rule']}" if cell["rule"] else "")
        row = summary.setdefault(group, {}).setdefault(key, {})
        row[cell["outcome"]] = row.get(cell["outcome"], 0) + 1
        row["rejections"] = row.get("rejections", 0) + cell["rejections"]
        row["fragile_checks"] = row.get("fragile_checks", 0) + sum(
            1 for check in cell["checks"] if check.get("fragile"))
    return summary


def hours(cells: dict, trial_ids: Iterable[str] | None = None) -> float:
    ids = list(cells) if trial_ids is None else list(trial_ids)
    minutes = 0.0
    for trial in ids:
        cell = cells[trial]
        outcome = cell["outcome"]
        if cell.get("deadline_risk"):
            outcome = "deadline"
        minutes += WALL_MIN[outcome] + 0.25 * cell["rejections"]
    return round(minutes / 60.0, 2)


def build_all(source_path: Path, catalog_path: Path,
              cells: dict | None = None) -> tuple[StudyDesign, dict]:
    source = _source_design(source_path)
    catalog = load_catalog(str(catalog_path)).supported_experiment_subset()
    provisional = build_design(source)
    if cells is None:
        cells = predict(provisional.trials, catalog)
    else:
        missing = {s.trial_id for s in provisional.trials} - set(cells)
        if missing:
            raise ValueError(f"precomputed cells missing {len(missing)} trials")
        cells = {s.trial_id: cells[s.trial_id] for s in provisional.trials}
        _mark_changes(cells)
    smoke = list(SMOKE_TRIAL_IDS)
    design = provisional
    by_tier: dict[int, list[str]] = {}
    for spec in design.trials:
        by_tier.setdefault(tier_of(spec), []).append(spec.trial_id)
    predictions = {
        "schema_version": 1,
        "purpose": "predetach_check_v1 planner-level predictions, generated before any trial",
        "design_hash": design.design_hash,
        "model": ("simulate_mission: method planner with accumulated site rejections; "
                  "true station rule at the nominal site placement (planner "
                  "convention) at each planned site in order; replan from the "
                  "site after a rejection; the plan's suffix after a "
                  "transformation"),
        "tier_meaning": {str(k): v for k, v in TIERS.items()},
        "parameters": {"heading_bins": HEADING_BINS, "epsilon": EPSILON,
                       "max_attempts": MAX_ATTEMPTS, "radius_m": 0.3,
                       "wedge_deg": 10.0, "fragility_shift_m": FRAGILITY_SHIFT_M,
                       "deadline_s": DEADLINE_S},
        "smoke_trial_ids": smoke,
        "tiers": {str(k): v for k, v in sorted(by_tier.items())},
        "estimated_hours": {"all": hours(cells),
                            **{f"tier_{k}": hours(cells, v)
                               for k, v in sorted(by_tier.items())}},
        "summary": summarize(cells),
        "cells": cells,
    }
    return design, predictions


# ---------------------------------------------------------------------------
# Scoring executed records against the predictions


def score(raw: Path, predictions: dict) -> dict:
    rows = []
    for path in sorted(raw.glob("*.json")):
        record = json.loads(path.read_text(encoding="utf-8"))
        trial = record["spec"]["trial_id"]
        cell = predictions["cells"].get(trial)
        if cell is None:
            continue
        events = record.get("controller_diagnostics", {}).get("predetach_checks", [])
        checks = [e for e in events if e.get("event") == "check"]
        rows.append({
            "trial_id": trial, "method": cell["method"], "rule": cell["rule"],
            "terminal_status": record["terminal_status"],
            "completed": record["completed"],
            "simulated_duration_s": record["simulated_duration_s"],
            "reconfiguration_failures": record["reconfiguration_failures"],
            "predicted_outcome": cell["outcome"],
            "predicted_rejections": cell["rejections"],
            "observed_checks": len(checks),
            "observed_rejections": sum(1 for e in checks if not e.get("passed")),
            "check_executed": (cell["method"] not in CHECKED_METHODS) or bool(checks),
            "outcome_matches": ((cell["outcome"] == "completed") == bool(record["completed"])),
        })
    return {"n": len(rows), "rows": rows,
            "completed": sum(r["completed"] for r in rows),
            "outcome_matches": sum(r["outcome_matches"] for r in rows),
            "checks_missing": [r["trial_id"] for r in rows if not r["check_executed"]]}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--source", type=Path,
                        default=Path("studies/confirmatory_v3/design.json"))
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--out", type=Path, default=STUDY_DIR)
    parser.add_argument("--score", type=Path, help="raw record directory to score")
    parser.add_argument("--shard", help="i/n: predict only every n-th layout, "
                                        "write cells to --cells-dir/shard_i.json")
    parser.add_argument("--cells-dir", type=Path,
                        help="read precomputed shard_*.json cells (or write a shard)")
    parser.add_argument("--design-only", action="store_true",
                        help="write design.json and smoke_trial_ids.json only")
    args = parser.parse_args()
    if args.shard:
        index, count = (int(v) for v in args.shard.split("/"))
        source = _source_design(args.source)
        catalog = load_catalog(str(args.catalog)).supported_experiment_subset()
        rows = build_design(source).trials
        layouts = sorted({r.layout_id for r in rows})[index::count]
        cells = predict([r for r in rows if r.layout_id in layouts], catalog)
        args.cells_dir.mkdir(parents=True, exist_ok=True)
        (args.cells_dir / f"shard_{index}.json").write_text(
            json.dumps(cells, sort_keys=True), encoding="utf-8")
        return
    if args.score:
        predictions = json.loads((args.out / "predictions.json").read_text(encoding="utf-8"))
        print(json.dumps(score(args.score, predictions), indent=2, sort_keys=True))
        return
    if args.design_only:
        design = build_design(_source_design(args.source))
        missing = set(SMOKE_TRIAL_IDS) - {t.trial_id for t in design.trials}
        if missing:
            raise ValueError(f"smoke ids not in the design: {sorted(missing)}")
        args.out.mkdir(parents=True, exist_ok=True)
        design.write_frozen(args.out / "design.json")
        (args.out / "smoke_trial_ids.json").write_text(
            json.dumps(list(SMOKE_TRIAL_IDS), indent=1) + "\n", encoding="utf-8")
        print(json.dumps({"design_hash": design.design_hash,
                          "trials": len(design.trials)}, sort_keys=True))
        return
    cells = None
    if args.cells_dir:
        cells = {}
        for path in sorted(args.cells_dir.glob("shard_*.json")):
            cells.update(json.loads(path.read_text(encoding="utf-8")))
    design, predictions = build_all(args.source, args.catalog, cells)
    args.out.mkdir(parents=True, exist_ok=True)
    design.write_frozen(args.out / "design.json")
    (args.out / "predictions.json").write_text(
        json.dumps(predictions, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    (args.out / "smoke_trial_ids.json").write_text(
        json.dumps(predictions["smoke_trial_ids"], indent=1) + "\n", encoding="utf-8")
    print(json.dumps({"design_hash": design.design_hash, "trials": len(design.trials),
                      "estimated_hours": predictions["estimated_hours"],
                      "summary": predictions["summary"]}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

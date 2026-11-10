"""Offset-world pilot (`offset_world_pilot_v1`): design, direction rule, predictions.

Question. The planner predicts station visibility from a prior map that is
identical to the simulated world, and chooses transition sites a few
centimetres from the edge of the elevated screen's shadow. If the real screen
is offset from the prior by a few centimetres, does the transition get refused,
and does a planner prior-map margin remove that?

This module is pure Python. It builds the pilot design from the frozen
`confirmatory_v3` design and its committed manipulation check, fixes the
direction in which each layout's screen is displaced, and writes the analysis's
prediction for every planned cell from geometry alone, before any trial exists.

Geometry behind the direction rule. The station S sees a pod P through the
screen's plane at a fraction `SCREEN_FRACTION` (0.5) of the way from S to P in
the horizontal, so a screen translated by t in the plane casts its shadow
translated by t / SCREEN_FRACTION = 2 t in the SAME direction. The shadow edge a
site is chosen next to therefore moves toward the site exactly when the screen
moves in the direction in which the site's pods first become hidden. The rule
(`toward_direction`) measures that directly with the shipped visibility
predicate instead of assuming it:

    for each of the four axis directions d in (+x, -x, +y, -y), the escape
    distance s_d is the smallest screen translation along d, in 1 mm steps up
    to 150 mm, that hides at least one moved pod at the planned site with the
    core placed exactly at the site (no placement error), in the world where
    only the screen has moved. "Toward" is the d with the smallest s_d (ties
    in the order listed). "Away" is -d.

The direction is taken from the shipped (margin 0) site and is shared by both
margins of a layout, so the two planner settings face the same world.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from dataclasses import replace
from random import Random

from morphology_planner import (
    HybridState, load_catalog, make_method_planner, station_sensing_provider,
)
from morphology_planner.observability import observed_by_any
from morphology_planner.planner import NoPathError
from morphology_planner.transition_policy import docked_pod_points

from .confirmatory_scenarios import (
    WORLD_OFFSET_FIELDS, apply_world_offset, make_confirmatory_scenario,
)
from .design import (
    OFFSET_WORLD_PILOT_DESIGN_KIND, SENSING_FAMILIES, OffsetTrialSpec, StudyDesign,
    _seed, layout_generator_for,
)

METHOD = "sensing_feasibility_coupled"
QUALIFYING_TRANSITION = "compact_to_narrow"
HEADING_BINS = 8
EPSILON = 2.5
MASTER_SEED = 20261109

MARGINS_M = (0.0, 0.04)
# (condition id, screen displacement in metres along the layout's direction, sign)
# sign +1 = toward the site's shadow edge, -1 = the opposite direction.
WORLD_CONDITIONS = (
    ("ctl", 0.0, 0),
    ("t02", 0.02, +1),
    ("t04", 0.04, +1),
    ("t06", 0.06, +1),
    ("a02", 0.02, -1),
)
SMOKE_CONDITIONS = ("ctl", "t06")

# Execution tiers: (margin, condition) cells run in this order. Tier 1 is the
# interim look (declaration.md): 6 cm at the shipped planner and its controls.
TIERS = (
    ((0.0, "t06"), (0.0, "ctl")),
    ((0.0, "t04"), (0.04, "t06")),
    ((0.04, "t04"), (0.04, "ctl")),
    ((0.0, "t02"), (0.0, "a02"), (0.04, "t02"), (0.04, "a02")),
)
INTERIM_TRIALS = 52

AXIS_DIRECTIONS = (("+x", (1.0, 0.0)), ("-x", (-1.0, 0.0)),
                   ("+y", (0.0, 1.0)), ("-y", (0.0, -1.0)))
ESCAPE_STEP_M = 0.001
ESCAPE_LIMIT_M = 0.150

# Placement error model of wave 2 (measured core placement): per-axis sigma
# 1.1 cm, plus a fixed 1.2 cm bias in a uniformly random direction, yaw sigma
# 2 degrees. Used only to attach a probability to each deterministic verdict.
PLACEMENT_SIGMA_M = 0.011
PLACEMENT_BIAS_M = 0.012
PLACEMENT_YAW_SIGMA_RAD = math.radians(2.0)
PLACEMENT_SAMPLES = 400
FIRM_PROBABILITY = 0.95


def cell_id(layout_id: str, margin_m: float, condition: str) -> str:
    return f"{layout_id}-r0-{METHOD}-m{round(margin_m * 100):02d}-{condition}"


def eligible_layouts(report: dict) -> list[tuple[str, int]]:
    """Non-neutral sensing-family layouts on which the shipped treatment plans.

    Read from the committed manipulation check, never recomputed here.
    """
    layouts = []
    for entry in report["layouts"]:
        if entry["family"] not in SENSING_FAMILIES or entry["layout_index"] % 4 == 0:
            continue
        if entry["decisions"][METHOD]["planned"]:
            layouts.append((entry["family"], int(entry["layout_index"])))
    return sorted(layouts, key=lambda item: (SENSING_ORDER.index(item[0]), item[1]))


SENSING_ORDER = ("docking_observability", "combined_constraints")


def plan_site(scenario, catalog, margin_m: float):
    """The treatment's compact_to_narrow site under a prior-map margin, or None."""
    planner = make_method_planner(
        METHOD, catalog, scenario.grid, heading_bins=HEADING_BINS,
        environment=scenario.transition_obstacles,
        sensing_provider=station_sensing_provider(
            catalog, scenario.fiducial_stations, scenario.transition_obstacles,
            scenario.grid, HEADING_BINS, margin_m))
    try:
        plan = planner.plan(
            HybridState(*scenario.start, 0, "compact_diff"), scenario.goal, EPSILON)
    except (NoPathError, ValueError):
        return None
    segment = next((value for value in plan.segments
                    if value.transition_id == QUALIFYING_TRANSITION), None)
    if segment is None:
        return None
    return segment.source


def hidden_pods(world, catalog, grid, site, placement=(0.0, 0.0), yaw_error=0.0):
    """Moved pods the WORLD's station cannot see at the planned site."""
    transition = next(value for value in catalog.transitions
                      if value.id == QUALIFYING_TRANSITION)
    points = docked_pod_points(
        catalog, transition, grid, site, HEADING_BINS, placement, yaw_error)
    return sorted(
        pod for pod, point in points.items()
        if observed_by_any(world.fiducial_stations, point,
                           world.transition_obstacles) is None)


def escape_distances(prior, catalog, site) -> dict[str, float | None]:
    """Screen translation along each axis that first hides a pod at `site`."""
    result: dict[str, float | None] = {}
    steps = round(ESCAPE_LIMIT_M / ESCAPE_STEP_M)
    for name, (ux, uy) in AXIS_DIRECTIONS:
        result[name] = None
        for step in range(steps + 1):
            t = step * ESCAPE_STEP_M
            moved = apply_world_offset(prior, (ux * t, uy * t, 0.0, 0.0))
            if hidden_pods(moved, catalog, prior.grid, site):
                result[name] = round(t, 6)
                break
    return result


def toward_direction(escape: dict[str, float | None]) -> str | None:
    """The axis direction with the smallest escape distance (ties: listed order)."""
    best = None
    for name, _ in AXIS_DIRECTIONS:
        distance = escape[name]
        if distance is not None and (best is None or distance < escape[best]):
            best = name
    return best


def world_offset_for(direction: str, magnitude_m: float, sign: int) -> list[float]:
    """(screen_dx, screen_dy, station_dx, station_dy) for one world condition."""
    ux, uy = dict(AXIS_DIRECTIONS)[direction]
    return [round(sign * magnitude_m * ux, 6), round(sign * magnitude_m * uy, 6),
            0.0, 0.0]


def placement_samples(layout_id: str) -> list[tuple[tuple[float, float], float]]:
    """Fixed placement draws, shared across every cell of a layout."""
    rng = Random(f"offset-pilot-placement-{layout_id}")
    samples = []
    for _ in range(PLACEMENT_SAMPLES):
        angle = rng.uniform(0.0, 2.0 * math.pi)
        placement = (rng.gauss(0.0, PLACEMENT_SIGMA_M) + PLACEMENT_BIAS_M * math.cos(angle),
                     rng.gauss(0.0, PLACEMENT_SIGMA_M) + PLACEMENT_BIAS_M * math.sin(angle))
        samples.append((placement, rng.gauss(0.0, PLACEMENT_YAW_SIGMA_RAD)))
    return samples


def predict_cell(prior, catalog, site, offset, layout_id: str) -> dict:
    """Deterministic verdict and placement-robust probability for one cell."""
    if site is None:
        return {"predicted_transition": "no_plan", "hidden_pods": [],
                "p_transition_accepted": None, "confidence": "firm",
                "predicted_completion": False,
                "predicted_terminal": "planning_failure"}
    world = apply_world_offset(prior, offset)
    hidden = hidden_pods(world, catalog, prior.grid, site)
    accepted_count = sum(
        not hidden_pods(world, catalog, prior.grid, site, placement, yaw)
        for placement, yaw in placement_samples(layout_id))
    probability = accepted_count / PLACEMENT_SAMPLES
    verdict = "refused" if hidden else "accepted"
    firm = (probability >= FIRM_PROBABILITY if not hidden
            else probability <= 1.0 - FIRM_PROBABILITY)
    return {
        "predicted_transition": verdict, "hidden_pods": hidden,
        "p_transition_accepted": round(probability, 4),
        "confidence": "firm" if firm else "marginal",
        "predicted_completion": not hidden,
        "predicted_terminal": "completed" if not hidden else "not_completed",
    }


def layout_block(task: tuple) -> tuple[dict, list[OffsetTrialSpec], list[dict]]:
    """Everything the pilot needs for one layout: its record, trials and cells."""
    family, index, source, generator, catalog_path, reuse = task
    catalog = load_catalog(str(catalog_path)).supported_experiment_subset()
    layout_id = f"{family}-{index:02d}"
    prior = make_confirmatory_scenario(
        family, index, source.world_seed, generator=generator)
    if reuse is None:
        sites = {margin: plan_site(prior, catalog, margin) for margin in MARGINS_M}
        escape = escape_distances(prior, catalog, sites[0.0])
    else:
        # Planning is deterministic: take the sites an earlier run of this very
        # generator recorded (same layout and world seed) instead of replanning.
        if reuse["world_seed"] != source.world_seed:
            raise ValueError(f"{layout_id}: reused plans belong to another world")
        sites = {0.0: _site_from_dict(reuse["site_margin_0"]),
                 0.04: _site_from_dict(reuse["site_margin_0p04"])}
        escape = reuse["escape_distance_m"]
    direction = toward_direction(escape)
    if direction is None:
        raise ValueError(f"{layout_id}: no screen translation hides a pod")
    layout = {
        "layout_id": layout_id, "family": family, "layout_index": index,
        "world_seed": source.world_seed, "direction_toward": direction,
        "escape_distance_m": escape,
        "site_margin_0": _site_dict(sites[0.0]),
        "site_margin_0p04": _site_dict(sites[0.04]),
    }
    block = [(margin, condition, magnitude, sign,
              world_offset_for(direction, magnitude, sign))
             for margin in MARGINS_M for condition, magnitude, sign in WORLD_CONDITIONS]
    order = list(range(len(block)))
    Random(_seed(MASTER_SEED, layout_id, "order")).shuffle(order)
    trials, cells = [], []
    for position, block_index in enumerate(order):
        margin, condition, magnitude, sign, offset = block[block_index]
        trial_id = cell_id(layout_id, margin, condition)
        trials.append(OffsetTrialSpec(
            trial_id=trial_id, family=family, layout_id=layout_id, replicate=0,
            method=METHOD, method_order=position, world_seed=source.world_seed,
            sensing_seed=source.sensing_seed, friction_seed=source.friction_seed,
            fault_seed=source.fault_seed, world_offset=tuple(offset),
            prior_margin_m=margin, world_condition=condition))
        cells.append({
            "trial_id": trial_id, "layout_id": layout_id, "family": family,
            "prior_margin_m": margin, "world_condition": condition,
            "world_offset": dict(zip(WORLD_OFFSET_FIELDS, offset)),
            "direction": direction if sign else None, "sign": sign,
            "site": _site_dict(sites[margin]),
            **predict_cell(prior, catalog, sites[margin], offset, layout_id),
        })
    return layout, trials, cells


def assign_run_order(trials: list[OffsetTrialSpec]) -> list[OffsetTrialSpec]:
    """Give every trial its global rank: tier by tier, layouts shuffled per tier
    (seeded), a layout's cells in the tier adjacent so paired cells run together."""
    layout_ids = sorted({trial.layout_id for trial in trials})
    by_key = {(t.layout_id, t.prior_margin_m, t.world_condition): t for t in trials}
    rank, ranked = 0, {}
    for tier_index, tier in enumerate(TIERS):
        order = list(layout_ids)
        Random(_seed(MASTER_SEED, "tier", tier_index)).shuffle(order)
        for layout_id in order:
            for margin, condition in tier:
                ranked[by_key[(layout_id, margin, condition)].trial_id] = rank
                rank += 1
    if rank != len(trials):
        raise ValueError("tiers do not cover the design exactly")
    return sorted((replace(t, run_order=ranked[t.trial_id]) for t in trials),
                  key=lambda t: t.run_order)


def build(source_design: StudyDesign, report: dict, catalog_path: Path,
          processes: int = 1, reuse_plans: dict | None = None):
    """The pilot design and its prediction table, from geometry alone."""
    generator = layout_generator_for(source_design.design_kind)
    sources = {(spec.family, spec.layout_id): spec for spec in source_design.trials
               if spec.method == METHOD and spec.replicate == 0}
    chosen = eligible_layouts(report)
    tasks = [(family, index, sources[(family, f"{family}-{index:02d}")],
              generator, catalog_path,
              None if reuse_plans is None else reuse_plans[f"{family}-{index:02d}"])
             for family, index in chosen]
    if processes > 1:
        from multiprocessing import Pool
        with Pool(processes) as pool:
            blocks = pool.map(layout_block, tasks, chunksize=1)
    else:
        blocks = [layout_block(task) for task in tasks]
    layouts = [block[0] for block in blocks]
    trials = assign_run_order([trial for block in blocks for trial in block[1]])
    cells = sorted((cell for block in blocks for cell in block[2]),
                   key=lambda cell: cell["trial_id"])
    design = StudyDesign(
        1, OFFSET_WORLD_PILOT_DESIGN_KIND,
        max(sum(1 for family, _ in chosen if family == name)
            for name in SENSING_ORDER),
        1, MASTER_SEED, (METHOD,), SENSING_ORDER, tuple(trials))
    predictions = {
        "schema_version": 1,
        "purpose": "offset_world_pilot_v1 predictions, written before any trial",
        "design_hash": design.design_hash,
        "source_design_hash": source_design.design_hash,
        "direction_rule": __doc__.split("Geometry behind the direction rule.")[1]
                                   .split("The direction is taken")[0].strip(),
        "placement_model": {
            "sigma_m": PLACEMENT_SIGMA_M, "bias_m": PLACEMENT_BIAS_M,
            "yaw_sigma_rad": PLACEMENT_YAW_SIGMA_RAD, "samples": PLACEMENT_SAMPLES,
            "firm_probability": FIRM_PROBABILITY},
        "layouts": layouts,
        "cells": cells,
        "summary": summarize(cells),
    }
    return design, predictions, smoke_trial_ids(layouts, cells)


def _site_from_dict(value) -> HybridState | None:
    return None if value is None else HybridState(
        value["cell_x"], value["cell_y"], value["heading"], "compact_diff")


def _site_dict(site) -> dict | None:
    return None if site is None else {
        "cell_x": site.x, "cell_y": site.y, "heading": site.heading}


def refusal_distribution(probabilities_refused: list[float]) -> list[float]:
    """Poisson-binomial law of the number of refusals among independent cells."""
    law = [1.0]
    for refusal in probabilities_refused:
        step = [0.0] * (len(law) + 1)
        for count, mass in enumerate(law):
            step[count] += mass * (1.0 - refusal)
            step[count + 1] += mass * refusal
        law = step
    return law


def _quantile(law: list[float], level: float) -> int:
    total = 0.0
    for count, mass in enumerate(law):
        total += mass
        if total >= level:
            return count
    return len(law) - 1


def _refusal_summary(cells: list[dict]) -> dict:
    refused = [1.0 - cell["p_transition_accepted"] for cell in cells
               if cell["p_transition_accepted"] is not None]
    law = refusal_distribution(refused)
    return {"cells": len(refused), "expected_refusals": round(sum(refused), 4),
            "p_zero_refusals": law[0], "q05": _quantile(law, 0.05),
            "q95": _quantile(law, 0.95), "q99": _quantile(law, 0.99)}


def summarize(cells: list[dict]) -> dict:
    """Counts by cell type, plus the refusal counts the placement model expects.

    `expected_refusals` is under the wave-2 placement model and independent cells:
    the reference an observed count is read against (see declaration.md).
    """
    table: dict[str, dict[str, int]] = {}
    for cell in cells:
        key = f"m{round(cell['prior_margin_m'] * 100):02d}-{cell['world_condition']}"
        row = table.setdefault(key, {"accepted": 0, "refused": 0, "no_plan": 0,
                                     "marginal": 0})
        row[cell["predicted_transition"]] += 1
        row["marginal"] += int(cell["confidence"] == "marginal")
    toward_shipped = [c for c in cells if c["prior_margin_m"] == 0.0
                      and c["world_condition"] == "t04"]
    toward_margin = [c for c in cells if c["prior_margin_m"] > 0.0
                     and c["world_condition"] == "t04"]
    six_shipped = [c for c in cells if c["prior_margin_m"] == 0.0
                   and c["world_condition"] == "t06"]
    six_margin = [c for c in cells if c["prior_margin_m"] > 0.0
                  and c["world_condition"] == "t06"]
    loss_cells = toward_shipped + six_shipped + six_margin
    predicted_safe = [c for c in cells if c["predicted_transition"] == "accepted"
                      and c not in loss_cells]
    return {
        "cells": len(cells), "by_cell_type": dict(sorted(table.items())),
        "refusal_reference": {
            "t04_margin_0": _refusal_summary(toward_shipped),
            "t04_margin_0p04": _refusal_summary(toward_margin),
            "t06_margin_0": _refusal_summary(six_shipped),
            "t06_margin_0p04": _refusal_summary(six_margin),
            "predicted_safe_cells": _refusal_summary(predicted_safe),
        },
    }


def smoke_trial_ids(layouts: list[dict], cells: list[dict]) -> list[str]:
    """3 layouts x {margin 0, 0.04} x {ctl, t06} = 12 trials.

    Fixed rule over layouts that plan under both margins: (1) the layout whose
    margin-0 6 cm cell has the highest predicted refusal probability (ties by
    id), with its margin-0.04 and no-offset partners; (2) the median and (3) the
    highest layout by escape distance along its direction, excluding (1).
    """
    planned = {cell["layout_id"] for cell in cells
               if cell["prior_margin_m"] > 0 and cell["predicted_transition"] != "no_plan"}
    refusal = {cell["layout_id"]: 1.0 - cell["p_transition_accepted"] for cell in cells
               if cell["prior_margin_m"] == 0.0 and cell["world_condition"] == "t06"
               and cell["layout_id"] in planned}
    first = min(refusal, key=lambda layout_id: (-refusal[layout_id], layout_id))
    ranked = sorted(
        (item for item in layouts if item["layout_id"] in planned
         and item["layout_id"] != first),
        key=lambda item: (item["escape_distance_m"][item["direction_toward"]],
                          item["layout_id"]))
    chosen = [first, ranked[len(ranked) // 2]["layout_id"], ranked[-1]["layout_id"]]
    return sorted(cell_id(layout_id, margin, condition)
                  for layout_id in chosen for margin in MARGINS_M
                  for condition in SMOKE_CONDITIONS)


REFUSAL_STATUSES = frozenset({"docking_failure", "unsafe_topology"})


def observed_refusal(record) -> bool:
    """A transition the mission attempted and the docking gate refused."""
    return (record.terminal_status in REFUSAL_STATUSES
            or int(record.reconfiguration_failures) > 0)


def score(predictions: dict, records: list) -> dict:
    """Read executed records against the stated predictions (declaration.md).

    Only counts and the declared decision rules; no significance test. Records
    outside the design are ignored; cells without a record are `pending`.
    """
    cells = {cell["trial_id"]: cell for cell in predictions["cells"]}
    reference = predictions["summary"]["refusal_reference"]
    by_id = {record.spec.trial_id: record for record in records
             if record.spec.trial_id in cells}
    rows, site_mismatch, unrelated = [], [], []
    for trial_id, cell in sorted(cells.items()):
        record = by_id.get(trial_id)
        if record is None:
            rows.append({**_key(cell), "state": "pending"})
            continue
        refused = observed_refusal(record)
        unrelated_failure = (not record.completed and not refused
                             and record.terminal_status != "planning_failure")
        if unrelated_failure:
            unrelated.append(trial_id)
        site = next((value for value in record.planned_transition_sites
                     if value.get("transition_id") == QUALIFYING_TRANSITION), None)
        if site is not None and cell["site"] is not None:
            observed = (math.floor(float(site["x"]) / 0.1), math.floor(float(site["y"]) / 0.1))
            if observed != (cell["site"]["cell_x"], cell["site"]["cell_y"]):
                site_mismatch.append(trial_id)
        rows.append({**_key(cell), "state": "done", "completed": record.completed,
                     "refused": refused, "status": record.terminal_status,
                     "predicted": cell["predicted_transition"]})

    def count(predicate):
        done = [row for row in rows if row["state"] == "done" and predicate(row)]
        return {"n": len(done), "refused": sum(row["refused"] for row in done),
                "completed": sum(row["completed"] for row in done)}

    shipped_toward = count(lambda r: r["margin"] == 0.0 and r["condition"] == "t04")
    margin_toward = count(lambda r: r["margin"] > 0.0 and r["condition"] == "t04"
                          and r["predicted"] != "no_plan")
    six_shipped = count(lambda r: r["margin"] == 0.0 and r["condition"] == "t06")
    six_margin = count(lambda r: r["margin"] > 0.0 and r["condition"] == "t06"
                       and r["predicted"] != "no_plan")
    safe = count(lambda r: r["predicted"] == "accepted"
                 and not (r["condition"] in ("t04", "t06")
                          and (r["margin"] == 0.0 or r["condition"] == "t06")))
    controls = count(lambda r: r["margin"] == 0.0 and r["condition"] == "ctl")
    complete_t06 = six_shipped["n"] == reference["t06_margin_0"]["cells"]
    complete_t04 = shipped_toward["n"] == reference["t04_margin_0"]["cells"]
    complete_safe = safe["n"] == reference["predicted_safe_cells"]["cells"]
    return {
        "t04_margin_0": shipped_toward, "t04_margin_0p04": margin_toward,
        "t06_margin_0": six_shipped, "t06_margin_0p04": six_margin,
        "predicted_safe_cells": safe, "control_cells": controls,
        "site_mismatch": site_mismatch, "unrelated_failures": unrelated,
        "verdicts": {
            # Falsifiers, each decidable only once its cells are all in.
            "controls_all_complete": (controls["completed"] == controls["n"]
                                      if controls["n"] else None),
            "safe_cells_refusals_within_q95": (
                safe["refused"] <= reference["predicted_safe_cells"]["q95"]
                if complete_safe else None),
            # Interim look (declaration.md): below q05 at 6 cm stops the pilot.
            "toward_6cm_shipped_shows_loss": (
                six_shipped["refused"] >= reference["t06_margin_0"]["q05"]
                if complete_t06 else None),
            "margin_removes_loss_at_6cm": (
                six_margin["refused"] <= reference["t06_margin_0p04"]["q95"]
                if six_margin["n"] == reference["t06_margin_0p04"]["cells"] else None),
            "toward_4cm_shipped_shows_loss": (
                shipped_toward["refused"] >= reference["t04_margin_0"]["q05"]
                if complete_t04 else None),
            "margin_removes_loss_at_4cm": (
                margin_toward["refused"] <= reference["t04_margin_0p04"]["q95"]
                if margin_toward["n"] == reference["t04_margin_0p04"]["cells"] else None),
        },
        "rows": rows,
    }


def _key(cell: dict) -> dict:
    return {"trial_id": cell["trial_id"], "margin": cell["prior_margin_m"],
            "condition": cell["world_condition"]}


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--source-design", type=Path,
                        default=Path("studies/confirmatory_v3/design.json"))
    parser.add_argument("--manipulation-check", type=Path,
                        default=Path("studies/confirmatory_v3/manipulation_check.json"))
    parser.add_argument("--catalog", type=Path,
                        default=Path("src/modular_robot_description/config/morphologies.yaml"))
    parser.add_argument("--out", type=Path,
                        default=Path("studies/offset_world_pilot_v1"))
    parser.add_argument("--processes", type=int, default=3)
    parser.add_argument("--reuse-plans", type=Path, metavar="PREDICTIONS_JSON",
                        help="take each layout's planned sites and escape distances "
                             "from an earlier predictions.json instead of replanning "
                             "(planning is deterministic; used for the 6 cm amendment)")
    parser.add_argument("--score", type=Path, metavar="RAW_DIR",
                        help="instead of generating, read the records in RAW_DIR "
                             "against --out/predictions.json")
    return parser


def main() -> None:
    args = _parser().parse_args()
    if args.score:
        from .records import TrialStore
        predictions = json.loads((args.out / "predictions.json").read_text(encoding="utf-8"))
        result = score(predictions, TrialStore(args.score).load_all())
        print(json.dumps(result, indent=2, sort_keys=True))
        return
    source = StudyDesign.read_frozen(args.source_design)
    report = json.loads(args.manipulation_check.read_text(encoding="utf-8"))
    if report["design_hash"] != source.design_hash:
        raise ValueError("manipulation check does not belong to the source design")
    reuse = None
    if args.reuse_plans:
        old = json.loads(args.reuse_plans.read_text(encoding="utf-8"))
        reuse = {item["layout_id"]: item for item in old["layouts"]}
    design, predictions, smoke = build(source, report, args.catalog, args.processes, reuse)
    args.out.mkdir(parents=True, exist_ok=True)
    design.write_frozen(args.out / "design.json")
    (args.out / "predictions.json").write_text(
        json.dumps(predictions, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (args.out / "smoke_trial_ids.json").write_text(
        json.dumps(smoke, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"design_hash": design.design_hash,
                      "trials": design.expected_trials, "smoke": len(smoke),
                      "summary": predictions["summary"]}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

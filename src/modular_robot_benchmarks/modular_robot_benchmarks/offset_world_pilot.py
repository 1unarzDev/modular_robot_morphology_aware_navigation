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
    ("a02", 0.02, -1),
)
SMOKE_CONDITIONS = ("ctl", "t04")

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
    family, index, source, generator, catalog_path = task
    catalog = load_catalog(str(catalog_path)).supported_experiment_subset()
    layout_id = f"{family}-{index:02d}"
    prior = make_confirmatory_scenario(
        family, index, source.world_seed, generator=generator)
    sites = {margin: plan_site(prior, catalog, margin) for margin in MARGINS_M}
    escape = escape_distances(prior, catalog, sites[0.0])
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


def build(source_design: StudyDesign, report: dict, catalog_path: Path,
          processes: int = 1):
    """The pilot design and its prediction table, from geometry alone."""
    generator = layout_generator_for(source_design.design_kind)
    sources = {(spec.family, spec.layout_id): spec for spec in source_design.trials
               if spec.method == METHOD and spec.replicate == 0}
    chosen = eligible_layouts(report)
    tasks = [(family, index, sources[(family, f"{family}-{index:02d}")],
              generator, catalog_path) for family, index in chosen]
    if processes > 1:
        from multiprocessing import Pool
        with Pool(processes) as pool:
            blocks = pool.map(layout_block, tasks, chunksize=1)
    else:
        blocks = [layout_block(task) for task in tasks]
    layouts = [block[0] for block in blocks]
    trials = [trial for block in blocks for trial in block[1]]
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


def _site_dict(site) -> dict | None:
    return None if site is None else {
        "cell_x": site.x, "cell_y": site.y, "heading": site.heading}


def summarize(cells: list[dict]) -> dict:
    table: dict[str, dict[str, int]] = {}
    for cell in cells:
        key = f"m{round(cell['prior_margin_m'] * 100):02d}-{cell['world_condition']}"
        row = table.setdefault(key, {"accepted": 0, "refused": 0, "no_plan": 0,
                                     "marginal": 0})
        row[cell["predicted_transition"]] += 1
        row["marginal"] += int(cell["confidence"] == "marginal")
    return {"cells": len(cells), "by_cell_type": dict(sorted(table.items()))}


def smoke_trial_ids(layouts: list[dict], cells: list[dict]) -> list[str]:
    """3 layouts x 2 margins x {ctl, t04}: lowest, median and highest escape distance.

    Fixed rule, applied to layouts that plan under both margins, ranked by the
    escape distance along their own direction (ties by layout id).
    """
    planned = {cell["layout_id"] for cell in cells
               if cell["prior_margin_m"] > 0 and cell["predicted_transition"] != "no_plan"}
    ranked = sorted(
        (item for item in layouts if item["layout_id"] in planned),
        key=lambda item: (item["escape_distance_m"][item["direction_toward"]],
                          item["layout_id"]))
    chosen = [ranked[0], ranked[len(ranked) // 2], ranked[-1]]
    ids = []
    for item in chosen:
        for margin in MARGINS_M:
            for condition in SMOKE_CONDITIONS:
                ids.append(cell_id(item["layout_id"], margin, condition))
    return sorted(ids)


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
    return parser


def main() -> None:
    args = _parser().parse_args()
    source = StudyDesign.read_frozen(args.source_design)
    report = json.loads(args.manipulation_check.read_text(encoding="utf-8"))
    if report["design_hash"] != source.design_hash:
        raise ValueError("manipulation check does not belong to the source design")
    design, predictions, smoke = build(source, report, args.catalog, args.processes)
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

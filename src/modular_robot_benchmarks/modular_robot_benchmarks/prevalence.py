"""Exploratory: how often does a geometry-only or feasibility-only site fail?

The confirmatory families are built so that one omitted condition can matter;
they measure what the information is worth *when* a constraint binds, not how
often one does. This study asks the second question on a layout population that
targets no method (`studies/prevalence/declaration.md`, declared before any
layout of the declared sample was generated): generator-3 rooms with randomly
drawn furniture and a randomly placed fiducial station.

For each layout, the planner alone (no simulation, so no robot outcome) plans
`geometry_coupled`, `feasibility_coupled` and `sensing_feasibility_coupled`,
then re-tests each control's chosen site under the full transition test. A
site the full test rejects is one where the executor would meet a collision in
a module or verification sweep, or refuse docking. Nothing here enters the
confirmatory study.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from random import Random

from morphology_planner import (
    CoupledTransitionPolicy, FiducialStation, HybridState, load_catalog,
    make_method_planner, station_sensing_provider,
)
from morphology_planner.grid import OCCUPIED, OccupancyGrid
from morphology_planner.planner import NoPathError
from morphology_planner.transition_validation import Box3

from .confirmatory_scenarios import (
    STATION_HEIGHT_M, STATION_WALL_OFFSET_M, WIDE_LAYOUT_ROOMS, _border, _wide_layout,
)

DEFAULT_CATALOG = Path("src/modular_robot_description/config/morphologies.yaml")
TRANSITION = "compact_to_narrow"
CONTROLS = ("geometry_coupled", "feasibility_coupled")
TREATMENT = "sensing_feasibility_coupled"
RESOLUTION = 0.1
DOOR_CELLS = 7
# Furniture never covers the spawn or the doorway, so the layout stays a
# passage problem; within that every placement is uniform in the start room.
KEEP_CLEAR_M = 0.5
TALL_COUNTS, RAISED_COUNTS = (0, 1, 2), (0, 1, 2, 3)
TALL_FOOTPRINT_M = ((0.3, 0.9), (0.2, 0.5))
TALL_HEIGHT_M = (1.0, 2.0)
RAISED_FOOTPRINT_M = ((0.3, 1.0), (0.1, 0.4))
# Underside above the 0.12 m lidar plane, so the 2D map cannot see it, and
# below the 0.30 m tallest body, so it can meet a pod.
RAISED_UNDERSIDE_M = (0.13, 0.26)
RAISED_THICKNESS_M = (0.03, 0.10)


def _box_clear(x0, y0, x1, y1, keep_clear) -> bool:
    return all(x1 < cx - KEEP_CLEAR_M or x0 > cx + KEEP_CLEAR_M
               or y1 < cy - KEEP_CLEAR_M or y0 > cy + KEEP_CLEAR_M
               for cx, cy in keep_clear)


def make_prevalence_layout(world_seed: int) -> dict:
    """One layout: a generator-3 room, furniture, and a station."""
    rng = Random(world_seed)
    grid, wall_x, door_center, start, goal = _wide_layout(
        rng, RESOLUTION, DOOR_CELLS, *WIDE_LAYOUT_ROOMS[3])
    _border(grid)
    door_low = door_center - DOOR_CELLS // 2
    for y in range(1, grid.height - 1):
        if not door_low <= y < door_low + DOOR_CELLS:
            grid.set_value(wall_x, y, OCCUPIED)
    room_x, room_y = wall_x * RESOLUTION, grid.height * RESOLUTION
    keep_clear = [((start[0] + 0.5) * RESOLUTION, (start[1] + 0.5) * RESOLUTION),
                  (room_x, (door_center + 0.5) * RESOLUTION)]
    boxes: list[Box3] = []

    def place(name, footprint, z_center, height, mark_grid):
        # Backed onto one of the start room's four walls, uniform along it: a
        # furnished room. Free-standing placement in a 3.9 x 3.3 m room left no
        # 1 m transition disk for any method (smoke test, master seed 1).
        for _ in range(50):
            length, depth = rng.uniform(*footprint[0]), rng.uniform(*footprint[1])
            wall = rng.choice(("west", "east", "south", "north"))
            if wall in ("west", "east"):
                sx, sy = depth, length
                x0 = RESOLUTION if wall == "west" else room_x - depth
                y0 = rng.uniform(RESOLUTION, room_y - RESOLUTION - sy)
            else:
                sx, sy = length, depth
                x0 = rng.uniform(RESOLUTION, room_x - sx)
                y0 = RESOLUTION if wall == "south" else room_y - RESOLUTION - depth
            if not _box_clear(x0, y0, x0 + sx, y0 + sy, keep_clear):
                continue
            boxes.append(Box3(name, (x0 + sx / 2, y0 + sy / 2, z_center), (sx, sy, height)))
            if mark_grid:
                for cx in range(int(x0 / RESOLUTION), int((x0 + sx) / RESOLUTION) + 1):
                    for cy in range(int(y0 / RESOLUTION), int((y0 + sy) / RESOLUTION) + 1):
                        grid.set_value(cx, cy, OCCUPIED)
            return

    for index in range(rng.choice(TALL_COUNTS)):
        height = rng.uniform(*TALL_HEIGHT_M)
        place(f"tall_{index}", TALL_FOOTPRINT_M, height / 2, height, True)
    for index in range(rng.choice(RAISED_COUNTS)):
        underside = rng.uniform(*RAISED_UNDERSIDE_M)
        thickness = rng.uniform(*RAISED_THICKNESS_M)
        place(f"raised_{index}", RAISED_FOOTPRINT_M, underside + thickness / 2,
              thickness, False)
    wall = rng.choice(("south", "north", "west"))
    if wall == "west":
        position = (STATION_WALL_OFFSET_M, rng.uniform(0.3, room_y - 0.3), STATION_HEIGHT_M)
    else:
        y = STATION_WALL_OFFSET_M if wall == "south" else room_y - STATION_WALL_OFFSET_M
        position = (rng.uniform(0.3, room_x - 0.3), y, STATION_HEIGHT_M)
    return {"world_seed": world_seed, "grid": grid, "start": start, "goal": goal,
            "environment": tuple(boxes), "stations": (FiducialStation("station", position),),
            "furniture": {"tall": sum(b.name.startswith("tall") for b in boxes),
                          "raised": sum(b.name.startswith("raised") for b in boxes)}}


def _stage(reason: str) -> str:
    if "connector_not_visible" in reason or "covariance_too_large" in reason:
        return "observability"
    if "post_transition_collision" in reason:
        return "verification"
    if reason in ("planar_clearance", "target_footprint"):
        return "planar"
    return "module_trajectory"


def evaluate_layout(layout: dict, catalog, heading_bins: int = 8,
                    epsilon: float = 2.5) -> dict:
    """Plan each method; re-test each control's site under the full test."""
    grid, environment, stations = layout["grid"], layout["environment"], layout["stations"]

    def provider():
        return station_sensing_provider(catalog, stations, environment, grid, heading_bins)

    transition = next(t for t in catalog.transitions if t.id == TRANSITION)
    full = CoupledTransitionPolicy(TREATMENT, catalog, grid, heading_bins, environment,
                                   sensing_provider=provider())
    result = {"world_seed": layout["world_seed"], "furniture": layout["furniture"],
              "methods": {}}
    for method in (*CONTROLS, TREATMENT):
        planner = make_method_planner(method, catalog, grid, heading_bins=heading_bins,
                                      environment=environment, sensing_provider=provider())
        try:
            plan = planner.plan(HybridState(*layout["start"], 0, "compact_diff"),
                                layout["goal"], epsilon)
        except (NoPathError, ValueError) as error:
            result["methods"][method] = {"planned": False, "error": type(error).__name__}
            continue
        segment = next((s for s in plan.segments if s.transition_id == TRANSITION), None)
        entry = {"planned": True, "site": None}
        if segment is not None:
            state = segment.source
            entry["site"] = [state.x, state.y, state.heading]
            if method in CONTROLS:
                full.decisions.clear()
                feasible = full(transition, state)
                reasons = full.decisions[-1].reasons
                entry["full_test_feasible"] = feasible
                entry["failed_stages"] = sorted({_stage(r) for r in reasons})
        result["methods"][method] = entry
    return result


def wilson_interval(successes: int, trials: int, z: float = 1.959964) -> list[float] | None:
    if trials == 0:
        return None
    p = successes / trials
    denominator = 1 + z * z / trials
    centre = (p + z * z / (2 * trials)) / denominator
    half = z * ((p * (1 - p) / trials + z * z / (4 * trials * trials)) ** 0.5) / denominator
    return [max(0.0, centre - half), min(1.0, centre + half)]


def summarize(results: list[dict]) -> dict:
    """The declared rates: share of planned control sites the full test rejects."""
    summary = {"layouts": len(results)}
    treatment_planned = [r["methods"][TREATMENT].get("planned", False) for r in results]
    for control in CONTROLS:
        sited = [r for r in results if r["methods"][control].get("site") is not None]
        rejected = [r for r in sited if not r["methods"][control]["full_test_feasible"]]
        stages = {}
        for r in rejected:
            for stage in r["methods"][control]["failed_stages"]:
                stages[stage] = stages.get(stage, 0) + 1
        summary[control] = {
            "planned_with_transition": len(sited),
            "site_rejected_by_full_test": len(rejected),
            "rejected_fraction": len(rejected) / len(sited) if sited else None,
            "rejected_fraction_wilson95": wilson_interval(len(rejected), len(sited)),
            "rejected_by_stage": stages,
            "rejected_and_treatment_plans": sum(
                r["methods"][TREATMENT].get("planned", False) for r in rejected),
            "unplanned": sum(not r["methods"][control]["planned"] for r in results),
        }
    summary[TREATMENT] = {"planned": sum(treatment_planned),
                          "unplanned": len(results) - sum(treatment_planned)}
    by_count: dict[str, dict] = {}
    for r in results:
        key = f"tall{r['furniture']['tall']}_raised{r['furniture']['raised']}"
        cell = by_count.setdefault(key, {"layouts": 0, "geometry_sited": 0,
                                         "geometry_rejected": 0})
        cell["layouts"] += 1
        entry = r["methods"]["geometry_coupled"]
        if entry.get("site") is not None:
            cell["geometry_sited"] += 1
            cell["geometry_rejected"] += int(not entry["full_test_feasible"])
    summary["by_furniture_count"] = dict(sorted(by_count.items()))
    return summary


def world_seeds(master_seed: int, count: int) -> list[int]:
    rng = Random(master_seed)
    return [rng.randrange(2 ** 31) for _ in range(count)]


def _worker(args):
    seed, catalog_path = args
    catalog = load_catalog(str(catalog_path)).supported_experiment_subset()
    return evaluate_layout(make_prevalence_layout(seed), catalog)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--master-seed", type=int, required=True)
    parser.add_argument("--layouts", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--workers", type=int, default=1)
    args = parser.parse_args()
    seeds = world_seeds(args.master_seed, args.layouts)
    # Each finished layout is appended to a checkpoint, so an interrupted run
    # resumes without redoing (or reordering) any layout.
    checkpoint = args.output.with_name(args.output.name + ".partial.jsonl")
    done = {}
    if checkpoint.exists():
        for line in checkpoint.read_text(encoding="utf-8").splitlines():
            if line.strip():
                entry = json.loads(line)
                done[entry["world_seed"]] = entry
    jobs = [(seed, args.catalog) for seed in seeds if seed not in done]
    checkpoint.parent.mkdir(parents=True, exist_ok=True)
    with checkpoint.open("a", encoding="utf-8") as stream:
        def keep(entry):
            done[entry["world_seed"]] = entry
            stream.write(json.dumps(entry, sort_keys=True) + "\n")
            stream.flush()
            print(f"layout {len(done)}/{len(seeds)}", flush=True)
        if args.workers > 1:
            from multiprocessing import Pool
            with Pool(args.workers) as pool:
                for entry in pool.imap_unordered(_worker, jobs, chunksize=1):
                    keep(entry)
        else:
            for job in jobs:
                keep(_worker(job))
    results = [done[seed] for seed in seeds]
    report = {"schema_version": 1, "purpose": "exploratory_prevalence",
              "master_seed": args.master_seed, "layouts": args.layouts,
              "summary": summarize(results), "results": results}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report["summary"], indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

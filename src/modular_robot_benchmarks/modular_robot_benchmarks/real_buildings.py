"""Exploratory: how often does an omitted transition condition bind in real buildings?

The generated-room prevalence study (`studies/prevalence/`) answered this for one
room generator. This study asks it of scanned real buildings (HM3D): missions
drawn in the buildings' own free space that the robot can only complete by
changing shape, with a fiducial station beside the narrow passage that forces
the change. As before, the planner alone plans each coupled method and each
control's chosen transition site is re-tested under the full transition test.
No simulation runs, so there is no robot outcome. Declared in
`studies/real_buildings/declaration.md` before the sample was read.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from math import hypot
from pathlib import Path
from random import Random

import numpy as np

from morphology_planner import (
    CoupledTransitionPolicy, FiducialStation, HybridState, load_catalog,
    make_method_planner, station_sensing_provider,
)
from morphology_planner.planner import NoPathError

from .building_mesh import (
    BOX_BAND_M, RESOLUTION, VOXEL_Z, build_levels, crop_level, read_glb_triangles,
)
from .lattice_reach import backtrack, build_model, drive_cell_masks, reach
from .prevalence import CONTROLS, TREATMENT, _stage, wilson_interval

DEFAULT_CATALOG = Path("src/modular_robot_description/config/morphologies.yaml")
HEADING_BINS = 8
EPSILON = 2.5
START_MORPHOLOGY, NARROW = "compact_diff", "narrow_tandem"
MISSIONS_PER_BUILDING = 3
DRAW_ATTEMPTS = 400
ROUTE_STEPS = (20, 250)         # shortest lattice route length, start to goal
WINDOW_MARGIN_M = 3.0           # planning window around the route
STATION_HEIGHT_M = 1.80         # as in every study layout
STATION_RADIUS_M = 3.0          # station within this of the passage cell
STATION_REACH_M = 0.5           # station cell within this of the start's region
NEIGHBOURS = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))


def _dilate(mask: np.ndarray, cells: int, within: np.ndarray) -> np.ndarray:
    """Dilation by `cells` 8-connected steps, never leaving `within`.

    Growing through free cells only keeps the station in the start's own
    space: a plain dilation crossed 0.1--0.3 m interior walls and put it in
    the next room (development, minival).
    """
    out = mask & within
    for _ in range(cells):
        grown = out.copy()
        grown[1:] |= out[:-1]
        grown[:-1] |= out[1:]
        grown[:, 1:] |= out[:, :-1]
        grown[:, :-1] |= out[:, 1:]
        grown[1:, 1:] |= out[:-1, :-1]
        grown[:-1, :-1] |= out[1:, 1:]
        grown[1:, :-1] |= out[:-1, 1:]
        grown[:-1, 1:] |= out[1:, :-1]
        out = grown & within
    return out


def building_seed(master_seed: int, building: str) -> int:
    digest = hashlib.sha256(f"{master_seed}:{building}".encode()).hexdigest()
    return int(digest[:12], 16)


def _route_metres(path) -> float:
    return sum(hypot(b[3] - a[3], b[2] - a[2]) for a, b in zip(path, path[1:])) * RESOLUTION


def draw_missions(levels, catalog, rng: Random, accept=None) -> tuple[list[dict], dict]:
    """Missions that need a shape change, with a station beside the passage.

    On the planner's own lattice (`lattice_reach`): a start is a uniformly
    drawn (cell, heading) where the compact body is free; a goal is a cell the
    narrow body reaches from it through a geometry-feasible transition, within
    ROUTE_STEPS lattice steps, that the compact body reaches at no heading. The
    passage cell is the first cell on a shortest lattice route that the compact
    body cannot reach; the station stands on a uniformly drawn free,
    wall-adjacent cell within STATION_REACH_M, through free cells, of the
    compact-reachable cells
    and STATION_RADIUS_M of the passage, at STATION_HEIGHT_M, with nothing
    scanned at that height in its cell. A drawn mission enters the sample only
    if `accept(level, mission)` holds (the caller plans it).
    """
    frames = []
    for level in levels:
        height, width = level.occupancy.shape
        _, _, drive = crop_level(level, 0, 0, width, height)
        masks = drive_cell_masks(level.occupancy.shape, drive, catalog, RESOLUTION)
        frames.append((level, build_model(level.occupancy, masks, catalog, HEADING_BINS,
                                          RESOLUTION)))
    starts = [(index, h, int(y), int(x)) for index, (_, model) in enumerate(frames)
              for h in range(HEADING_BINS)
              for y, x in zip(*np.nonzero(model.state_free[START_MORPHOLOGY, h]))]
    stats = {"levels": len(levels), "start_states": len(starts), "attempts": 0,
             "no_goal": 0, "no_station": 0, "geometry_unplanned": 0}
    missions: list[dict] = []
    forward = [t for t in catalog.transitions if t.source == START_MORPHOLOGY]
    while starts and len(missions) < MISSIONS_PER_BUILDING and stats["attempts"] < DRAW_ATTEMPTS:
        stats["attempts"] += 1
        index, sh, sy, sx = rng.choice(starts)
        level, model = frames[index]
        depth = reach(model, (START_MORPHOLOGY, sh, sy, sx), forward, ROUTE_STEPS[1])
        compact_cells = np.zeros(level.occupancy.shape, bool)
        for h in range(HEADING_BINS):
            compact_cells |= depth[START_MORPHOLOGY, h] >= 0
        narrow_depth = np.full(level.occupancy.shape, np.iinfo(np.int16).max, np.int32)
        for h in range(HEADING_BINS):
            d = depth[NARROW, h].astype(np.int32)
            narrow_depth = np.where((d >= 0) & (d < narrow_depth), d, narrow_depth)
        goal_mask = ((narrow_depth >= ROUTE_STEPS[0]) & (narrow_depth <= ROUTE_STEPS[1])
                     & ~compact_cells)
        goals = sorted(zip(*np.nonzero(goal_mask)))
        if not goals:
            stats["no_goal"] += 1
            continue
        gy, gx = (int(v) for v in rng.choice(goals))
        gh = min((h for h in range(HEADING_BINS) if depth[NARROW, h][gy, gx] >= 0),
                 key=lambda h: depth[NARROW, h][gy, gx])
        path = backtrack(model, depth, (NARROW, gh, gy, gx), forward)
        passage = next((p[2], p[3]) for p in path if not compact_cells[p[2], p[3]])
        occupancy = level.occupancy
        radius = STATION_RADIUS_M / RESOLUTION
        station_z = int((STATION_HEIGHT_M - BOX_BAND_M[0]) / VOXEL_Z)
        near_region = _dilate(compact_cells, round(STATION_REACH_M / RESOLUTION), ~occupancy)
        candidates = sorted(
            (int(y), int(x)) for y, x in zip(*np.nonzero(near_region & ~occupancy))
            if (y - passage[0]) ** 2 + (x - passage[1]) ** 2 <= radius * radius
            and any(occupancy[y + dy, x + dx] for dx, dy in NEIGHBOURS[:4]
                    if 0 <= y + dy < occupancy.shape[0] and 0 <= x + dx < occupancy.shape[1])
            and not level.voxels[y, x, max(0, station_z - 1):station_z + 2].any())
        if not candidates:
            stats["no_station"] += 1
            continue
        station = rng.choice(candidates)
        cells = [(p[2], p[3]) for p in path] + [station]
        margin = round(WINDOW_MARGIN_M / RESOLUTION)
        y0 = max(0, min(c[0] for c in cells) - margin)
        x0 = max(0, min(c[1] for c in cells) - margin)
        y1 = min(occupancy.shape[0], max(c[0] for c in cells) + margin + 1)
        x1 = min(occupancy.shape[1], max(c[1] for c in cells) + margin + 1)
        missions.append({
            "level": level.index, "floor_z": round(level.floor_z, 3),
            "window": [x0, y0, x1, y1],
            "start": [sx - x0, sy - y0, sh], "goal": [gx - x0, gy - y0],
            "passage": [passage[1] - x0, passage[0] - y0],
            "station_cell": [station[1] - x0, station[0] - y0],
            "route_steps": len(path) - 1, "route_m": round(_route_metres(path), 2),
        })
        if accept is not None and not accept(level, missions[-1]):
            missions.pop()
            stats["geometry_unplanned"] += 1
    return missions, stats


def evaluate_mission(level, mission: dict, catalog) -> dict:
    """Plan each coupled method; re-test every transition a control plans."""
    # Walls and everything else the scan plane maps are grid-only for module
    # sweeps and driving, as in every generated layout: the 1.0 m planar
    # clearance disk is what keeps a transition off them (catalog,
    # post_transition_verification). A sight line is blocked by anything, so
    # the station predictor alone sees every scanned box.
    grid, sight_boxes, unmapped = crop_level(level, *mission["window"])
    sx, sy = mission["station_cell"]
    stations = (FiducialStation("station", (*grid.cell_center(sx, sy), STATION_HEIGHT_M)),)

    def provider():
        return station_sensing_provider(catalog, stations, sight_boxes, grid, HEADING_BINS)

    transitions = {t.id: t for t in catalog.transitions}
    full = CoupledTransitionPolicy(TREATMENT, catalog, grid, HEADING_BINS, unmapped,
                                   sensing_provider=provider())
    result = {"sight_boxes": len(sight_boxes), "unmapped_boxes": len(unmapped), "methods": {}}
    start = HybridState(*mission["start"], START_MORPHOLOGY)
    # geometry_coupled checks the least, so where it cannot plan no coupled
    # method can: such a draw is no mission, and is redrawn (declared).
    for method in (*CONTROLS, TREATMENT):
        if method != CONTROLS[0] and not result["methods"][CONTROLS[0]]["planned"]:
            break
        planner = make_method_planner(method, catalog, grid, heading_bins=HEADING_BINS,
                                      environment=unmapped, sensing_provider=provider())
        try:
            plan = planner.plan(start, tuple(mission["goal"]), EPSILON)
        except (NoPathError, ValueError) as error:
            result["methods"][method] = {"planned": False, "error": type(error).__name__}
            continue
        sites = []
        for segment in plan.segments:
            if segment.kind != "reconfigure":
                continue
            state = segment.source
            site = {"transition": segment.transition_id,
                    "site": [state.x, state.y, state.heading]}
            if method in CONTROLS:
                full.decisions.clear()
                site["full_test_feasible"] = full(transitions[segment.transition_id], state)
                site["failed_stages"] = sorted({_stage(r) for r in full.decisions[-1].reasons})
            sites.append(site)
        result["methods"][method] = {"planned": True, "sites": sites}
    return result


def evaluate_building(path: Path, master_seed: int, catalog) -> dict:
    name = path.stem
    rng = Random(building_seed(master_seed, name))
    levels = build_levels(read_glb_triangles(path))

    def accept(level, mission):
        mission["result"] = evaluate_mission(level, mission, catalog)
        return mission["result"]["methods"][CONTROLS[0]]["planned"]

    missions, stats = draw_missions(levels, catalog, rng, accept)
    return {"building": name, "draw": stats, "missions": missions}


def _mission_rejected(entry: dict) -> bool | None:
    """None if the control planned no transition; else whether any site failed."""
    if not entry.get("planned") or not entry["sites"]:
        return None
    return any(not site["full_test_feasible"] for site in entry["sites"])


def summarize(buildings: list[dict], bootstrap: int = 2000, seed: int = 0) -> dict:
    """The declared rates, by first mission per building and over all missions."""
    summary = {"buildings": len(buildings),
               "buildings_with_mission": sum(bool(b["missions"]) for b in buildings),
               "missions": sum(len(b["missions"]) for b in buildings)}
    rng = np.random.default_rng(seed)
    for control in CONTROLS:
        first = [_mission_rejected(b["missions"][0]["result"]["methods"][control])
                 for b in buildings if b["missions"]]
        first = [value for value in first if value is not None]
        per_building = []
        stages: dict[str, int] = {}
        rejected_and_treatment_plans = 0
        for b in buildings:
            values = []
            for mission in b["missions"]:
                methods = mission["result"]["methods"]
                value = _mission_rejected(methods[control])
                if value is None:
                    continue
                values.append(value)
                if value:
                    rejected_and_treatment_plans += methods[TREATMENT].get("planned", False)
                    for site in methods[control]["sites"]:
                        for stage in site["failed_stages"]:
                            stages[stage] = stages.get(stage, 0) + 1
            if values:
                per_building.append((sum(values), len(values)))
        counts = np.array(per_building, float).reshape(-1, 2)
        pooled = counts[:, 0].sum() / counts[:, 1].sum() if len(counts) else None
        interval = None
        if len(counts):
            draws = rng.integers(0, len(counts), (bootstrap, len(counts)))
            resampled = counts[draws].sum(axis=1)
            rates = resampled[:, 0] / resampled[:, 1]
            interval = [float(np.quantile(rates, 0.025)), float(np.quantile(rates, 0.975))]
        summary[control] = {
            "first_mission": {
                "with_transition": len(first), "rejected": int(sum(first)),
                "rejected_fraction": sum(first) / len(first) if first else None,
                "wilson95": wilson_interval(int(sum(first)), len(first))},
            "all_missions": {
                "with_transition": int(counts[:, 1].sum()) if len(counts) else 0,
                "rejected": int(counts[:, 0].sum()) if len(counts) else 0,
                "rejected_fraction": pooled,
                "building_bootstrap95": interval},
            "rejected_site_stages": stages,
            "rejected_and_treatment_plans": int(rejected_and_treatment_plans),
            "unplanned": sum(not m["result"]["methods"][control].get("planned", False)
                             for b in buildings for m in b["missions"]),
        }
    summary[TREATMENT] = {"unplanned": sum(
        not m["result"]["methods"][TREATMENT].get("planned", False)
        for b in buildings for m in b["missions"])}
    return summary


def _worker(args):
    path, master_seed, catalog_path = args
    catalog = load_catalog(str(catalog_path)).supported_experiment_subset()
    try:
        return evaluate_building(Path(path), master_seed, catalog)
    except Exception as error:  # recorded, never dropped silently
        return {"building": Path(path).stem, "error": f"{type(error).__name__}: {error}",
                "draw": None, "missions": []}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--buildings", type=Path, required=True,
                        help="text file listing one GLB path per line, in sample order")
    parser.add_argument("--data-root", type=Path, default=None,
                        help="directory the listed paths are relative to")
    parser.add_argument("--master-seed", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--workers", type=int, default=1)
    args = parser.parse_args()
    paths = [str(args.data_root / line.strip()) if args.data_root else line.strip()
             for line in args.buildings.read_text(encoding="utf-8").splitlines()
             if line.strip()]
    checkpoint = args.output.with_name(args.output.name + ".partial.jsonl")
    done = {}
    if checkpoint.exists():
        for line in checkpoint.read_text(encoding="utf-8").splitlines():
            if line.strip():
                entry = json.loads(line)
                done[entry["building"]] = entry
    jobs = [(p, args.master_seed, args.catalog) for p in paths if Path(p).stem not in done]
    checkpoint.parent.mkdir(parents=True, exist_ok=True)
    with checkpoint.open("a", encoding="utf-8") as stream:
        def keep(entry):
            done[entry["building"]] = entry
            stream.write(json.dumps(entry, sort_keys=True) + "\n")
            stream.flush()
            print(f"building {len(done)}/{len(paths)} {entry['building']} "
                  f"missions={len(entry['missions'])}{' ERROR' if 'error' in entry else ''}",
                  flush=True)
        if args.workers > 1:
            from multiprocessing import Pool
            with Pool(args.workers) as pool:
                for entry in pool.imap_unordered(_worker, jobs, chunksize=1):
                    keep(entry)
        else:
            for job in jobs:
                keep(_worker(job))
    buildings = [done[Path(p).stem] for p in paths]
    report = {"schema_version": 1, "purpose": "exploratory_real_buildings",
              "master_seed": args.master_seed, "buildings": len(paths),
              "summary": summarize(buildings), "results": buildings}
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report["summary"], indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

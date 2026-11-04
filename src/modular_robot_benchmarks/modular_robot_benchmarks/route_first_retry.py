"""Exploratory: is `route_first_adaptation` weak only because it never retries?

The pilot found the sequential rung plans almost no layout: its one shortest
route reaches the doorway with no site that admits `compact_to_narrow`. A
reviewer can fairly say a practical sequential planner would try another
route. This variant keeps the sequential structure -- a morphology-agnostic
route first, then morphology assignment along that fixed route -- and on an
adaptation failure penalizes the cells near every failed route and plans a new
one, up to `max_routes` routes (`studies/route_first_retry/declaration.md`).

Planner only: no simulation and so no robot outcome. It is not a method of the
confirmatory study and changes nothing in it.
"""

from __future__ import annotations

import argparse
import json
import time
from collections import Counter
from pathlib import Path

from morphology_planner import (
    HybridState, load_catalog, make_method_planner, plan_signature,
    station_sensing_provider,
)
from morphology_planner.planner import HybridPlan, NoPathError
from morphology_planner.route_first import RouteFirstAdaptationPlanner, SpatialState

from .confirmatory_scenarios import make_confirmatory_scenario
from .design import StudyDesign, layout_generator_for
from .manipulation_check import QUALIFYING_TRANSITION, layout_seeds

DEFAULT_CATALOG = Path("src/modular_robot_description/config/morphologies.yaml")
FAMILIES = ("reconfiguration_workspace", "docking_observability",
            "combined_constraints")
TREATMENT = "sensing_feasibility_coupled"
SUPERSET = "studies/confirmatory_v3/manipulation_check_superset40_{}.json"
# Superset layouts 0-19 are the frozen design's; 20-39 are outside it and are
# the only layouts the variant's parameters may be tuned on.
TUNING_INDICES = range(20, 40)


class RouteFirstRetryPlanner(RouteFirstAdaptationPlanner):
    """Route-first adaptation that replans the route after an adaptation failure.

    Each failed route adds one to a count on every cell within
    `penalty_radius` cells (Chebyshev) of it; a spatial edge into a cell costs
    `1 + penalty_weight * count` times its geometric cost. Costs only grow, so
    the spatial heuristic stays admissible. The adaptation stage is unchanged
    and still never alters a route.
    """

    def __init__(self, *args, max_routes: int = 10, penalty_radius: int = 2,
                 penalty_weight: float = 1.0, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.max_routes = max_routes
        self.penalty_radius = penalty_radius
        self.penalty_weight = penalty_weight
        self._penalty: Counter = Counter()
        self.attempts: list[dict] = []

    def plan(self, start: HybridState, goal_xy: tuple[int, int],
             epsilon: float = 1.0) -> HybridPlan:
        if start.morphology not in self.catalog.morphologies:
            raise ValueError(f"unknown start morphology: {start.morphology}")
        start = self.hybrid.repaired_start(start)
        self._penalty = Counter()
        self.attempts = []
        seen: set[tuple] = set()
        expanded = 0
        for _ in range(self.max_routes):
            route, spatial_expanded = self._spatial_route(
                SpatialState(start.x, start.y, start.heading), goal_xy, epsilon)
            expanded += spatial_expanded
            cells = tuple((pose.x, pose.y) for pose in route)
            attempt = {"route_cells": len(cells), "repeat": cells in seen}
            seen.add(cells)
            try:
                plan = self._adapt(route, start.morphology, epsilon, expanded)
            except NoPathError:
                self.attempts.append({**attempt, "adapted": False})
                self._penalize(cells)
                continue
            self.attempts.append({**attempt, "adapted": True})
            return plan
        raise NoPathError(
            f"none of {self.max_routes} spatial routes can be adapted with "
            "feasible transitions")

    def _penalize(self, cells: tuple[tuple[int, int], ...]) -> None:
        r = self.penalty_radius
        near = {(x + dx, y + dy) for x, y in cells
                for dx in range(-r, r + 1) for dy in range(-r, r + 1)}
        self._penalty.update(near)

    def _spatial_neighbors(self, state: SpatialState) -> dict[SpatialState, float]:
        neighbors = super()._spatial_neighbors(state)
        if not self._penalty:
            return neighbors
        return {target: cost * (1.0 + self.penalty_weight
                                * self._penalty.get((target.x, target.y), 0))
                for target, cost in neighbors.items()}


def _summary(plan: HybridPlan) -> dict:
    segment = next((value for value in plan.segments
                    if value.transition_id == QUALIFYING_TRANSITION), None)
    return {
        "cost": round(plan.total_cost, 4),
        "route_signature": plan_signature(plan),
        "transition_site": ([segment.source.x, segment.source.y]
                            if segment is not None else None),
    }


def layout_record(family: str, index: int, world_seed: int, catalog,
                  generator: int, heading_bins: int, epsilon: float,
                  max_routes: int, penalty_radius: int,
                  penalty_weight: float) -> dict:
    """The variant's and the treatment's planner decisions for one layout."""
    scenario = make_confirmatory_scenario(
        family, index, world_seed, generator=generator)
    start = HybridState(*scenario.start, 0, "compact_diff")
    treatment = make_method_planner(
        TREATMENT, catalog, scenario.grid, heading_bins=heading_bins,
        environment=scenario.transition_obstacles,
        sensing_provider=station_sensing_provider(
            catalog, scenario.fiducial_stations, scenario.transition_obstacles,
            scenario.grid, heading_bins))
    record = {"family": family, "layout_index": index, "world_seed": world_seed}
    try:
        record[TREATMENT] = {"planned": True, **_summary(
            treatment.plan(start, scenario.goal, epsilon))}
    except (NoPathError, ValueError) as error:
        record[TREATMENT] = {"planned": False, "error": f"{type(error).__name__}: {error}"}
    # The variant validates transitions with the treatment's own policy, as
    # `route_first_adaptation` does (methods.make_method_planner).
    variant = RouteFirstRetryPlanner(
        catalog, scenario.grid, heading_bins=heading_bins,
        transition_validator=treatment.transition_policy,
        drive_obstacles=scenario.transition_obstacles,
        max_routes=max_routes, penalty_radius=penalty_radius,
        penalty_weight=penalty_weight)
    began = time.perf_counter()
    try:
        plan = variant.plan(start, scenario.goal, epsilon)
        outcome = {"planned": True, **_summary(plan)}
    except (NoPathError, ValueError) as error:
        outcome = {"planned": False, "error": f"{type(error).__name__}: {error}"}
    outcome["seconds"] = round(time.perf_counter() - began, 2)
    outcome["attempts"] = variant.attempts
    record["route_first_retry"] = outcome
    return record


def tuning_seeds() -> list[tuple[str, int, int]]:
    seeds = []
    for family in FAMILIES:
        superset = json.loads(Path(SUPERSET.format(family)).read_text(encoding="utf-8"))
        seeds.extend((family, entry["layout_index"], entry["world_seed"])
                     for entry in superset["layouts"]
                     if entry["layout_index"] in TUNING_INDICES)
    return seeds


def design_seeds(design: StudyDesign) -> list[tuple[str, int, int]]:
    return [(family, index, seed) for family in FAMILIES
            for index, seed in layout_seeds(design, family)]


def report(records: list[dict], max_routes: int) -> dict:
    """Plan rate by number of routes allowed, and the declared comparisons."""
    planned = [r for r in records if r["route_first_retry"]["planned"]]
    by_k = {k: sum(1 for r in planned
                   if len(r["route_first_retry"]["attempts"]) <= k)
            for k in range(1, max_routes + 1)}
    both = [r for r in planned if r[TREATMENT]["planned"]]
    return {
        "layouts": len(records),
        "planned_within_k_routes": by_k,
        "treatment_planned": sum(1 for r in records if r[TREATMENT]["planned"]),
        "both_planned": len(both),
        "cost_ratio_to_treatment": sorted(
            round(r["route_first_retry"]["cost"] / r[TREATMENT]["cost"], 4)
            for r in both),
        "same_site_as_treatment": sum(
            1 for r in both if r["route_first_retry"]["transition_site"]
            == r[TREATMENT]["transition_site"]),
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--sample", choices=("tuning", "design"), required=True)
    parser.add_argument("--design", type=Path,
                        default=Path("studies/confirmatory_v3/design.json"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--heading-bins", type=int, default=8)
    parser.add_argument("--epsilon", type=float, default=2.5)
    parser.add_argument("--max-routes", type=int, default=10)
    parser.add_argument("--penalty-radius", type=int, default=2)
    parser.add_argument("--penalty-weight", type=float, default=1.0)
    return parser


def main() -> None:
    args = _parser().parse_args()
    design = StudyDesign.read_frozen(args.design)
    generator = layout_generator_for(design.design_kind)
    seeds = tuning_seeds() if args.sample == "tuning" else design_seeds(design)
    catalog = load_catalog(str(args.catalog)).supported_experiment_subset()
    # A per-layout checkpoint beside the output, so an interrupted run resumes.
    checkpoint = args.output.with_name(args.output.name + ".partial.jsonl")
    done = {}
    if checkpoint.exists():
        for line in checkpoint.read_text(encoding="utf-8").splitlines():
            if line.strip():
                entry = json.loads(line)
                done[(entry["family"], entry["layout_index"])] = entry
    args.output.parent.mkdir(parents=True, exist_ok=True)
    records = []
    with checkpoint.open("a", encoding="utf-8") as sink:
        for number, (family, index, seed) in enumerate(seeds, 1):
            record = done.get((family, index))
            if record is None:
                record = layout_record(
                    family, index, seed, catalog, generator, args.heading_bins,
                    args.epsilon, args.max_routes, args.penalty_radius,
                    args.penalty_weight)
                sink.write(json.dumps(record, sort_keys=True) + "\n")
                sink.flush()
            records.append(record)
            print(f"layout {number}/{len(seeds)}", flush=True)
    output = {
        "schema_version": 1,
        "purpose": "exploratory_route_first_retry",
        "sample": args.sample,
        "design_hash": design.design_hash,
        "layout_generator": generator,
        "parameters": {"heading_bins": args.heading_bins, "epsilon": args.epsilon,
                       "max_routes": args.max_routes,
                       "penalty_radius": args.penalty_radius,
                       "penalty_weight": args.penalty_weight},
        "summary": report(records, args.max_routes),
        "layouts": records,
    }
    args.output.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    checkpoint.unlink()
    print(json.dumps(output["summary"], indent=2))


if __name__ == "__main__":
    main()

from __future__ import annotations

from dataclasses import dataclass, replace
import hashlib
import json
from typing import Literal, Mapping

from .catalog import Catalog
from .cost_model import EdgeCostModel
from .grid import OccupancyGrid
from .planner import HybridPlan, HybridState, MorphologyAStar
from .route_first import RouteFirstAdaptationPlanner
from .transition_policy import CoupledTransitionPolicy, PodSensingState, SensingProvider
from .transition_validation import Box3, TransitionTrajectoryValidator


PlannerMethod = Literal[
    "route_first_adaptation",
    "geometry_coupled",
    "feasibility_coupled",
    "sensing_feasibility_coupled",
]


@dataclass
class MethodPlanner:
    method: PlannerMethod
    planner: MorphologyAStar | RouteFirstAdaptationPlanner
    transition_policy: CoupledTransitionPolicy
    topology_revision: int = 0
    sensing_revision: int = 0

    def plan(self, start: HybridState, goal_xy: tuple[int, int],
             epsilon: float = 1.0) -> HybridPlan:
        return replace(
            self.planner.plan(start, goal_xy, epsilon),
            method_id=self.method,
            topology_revision=self.topology_revision,
            sensing_revision=self.sensing_revision,
        )

    def transition_decision_records(self) -> list[dict]:
        return [decision.to_record() for decision in self.transition_policy.decisions]

    def aggregated_transition_decisions(self) -> list[dict]:
        return aggregate_decision_records(self.transition_decision_records())


def aggregate_decision_records(records: list[dict]) -> list[dict]:
    """Collapse per-state edge decisions into counted distinct outcomes.

    Search evaluates the same transition at many states; terminal records keep
    each distinct method/transition/feasibility/reason combination once.
    """
    counts: dict[tuple, int] = {}
    for record in records:
        key = (record["method"], record["transition_id"], bool(record["feasible"]),
               tuple(sorted(record["reasons"])))
        counts[key] = counts.get(key, 0) + 1
    return [
        {"method": method, "transition_id": transition_id, "feasible": feasible,
         "reasons": list(reasons), "count": count}
        for (method, transition_id, feasible, reasons), count in sorted(counts.items())
    ]


def plan_signature(plan: HybridPlan) -> str:
    """Stable signature for paired route/transition manipulation checks."""
    value = {
        "states": [[state.x, state.y, state.heading, state.morphology]
                   for state in plan.states],
        "transitions": [
            [segment.transition_id, segment.source.x, segment.source.y,
             segment.source.heading]
            for segment in plan.segments if segment.kind == "reconfigure"
        ],
    }
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def make_method_planner(
    method: PlannerMethod,
    catalog: Catalog,
    grid: OccupancyGrid,
    heading_bins: int = 16,
    environment: tuple[Box3, ...] = (),
    static_modules: tuple[Box3, ...] = (),
    sensing: Mapping[str, PodSensingState] | None = None,
    sensing_provider: SensingProvider | None = None,
    trajectory_validator: TransitionTrajectoryValidator | None = None,
    cost_model: EdgeCostModel | None = None,
    topology_revision: int = 0,
    sensing_revision: int = 0,
) -> MethodPlanner:
    """Construct one frozen ablation over common search and execution inputs."""
    if method == "route_first_adaptation":
        policy_method = "sensing_feasibility_coupled"
    elif method in {
        "geometry_coupled", "feasibility_coupled",
        "sensing_feasibility_coupled",
    }:
        policy_method = method
    else:
        raise ValueError(f"unsupported planner method: {method}")
    policy = CoupledTransitionPolicy(
        policy_method, catalog, grid, heading_bins, environment, static_modules,
        sensing, sensing_provider, trajectory_validator,
    )
    planner_type = (
        RouteFirstAdaptationPlanner
        if method == "route_first_adaptation" else MorphologyAStar
    )
    # Every method drives around the same raised obstacles; only transition
    # validation distinguishes them.
    planner = planner_type(
        catalog, grid, heading_bins=heading_bins,
        transition_validator=policy, cost_model=cost_model,
        drive_obstacles=environment,
    )
    return MethodPlanner(
        method, planner, policy, topology_revision, sensing_revision)

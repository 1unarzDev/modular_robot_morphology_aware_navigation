from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
from itertools import count
from math import hypot, pi

from .catalog import Catalog
from .cost_model import EdgeCostModel
from .grid import OccupancyGrid
from .planner import (
    HybridPlan,
    HybridSegment,
    HybridState,
    MorphologyAStar,
    NoPathError,
    TransitionValidator,
)


@dataclass(frozen=True, order=True)
class SpatialState:
    x: int
    y: int
    heading: int


class RouteFirstAdaptationPlanner:
    """Sequential baseline: freeze a geometric route, then assign morphologies.

    The first search admits a spatial primitive when any catalog morphology can
    execute it and never evaluates a reconfiguration edge. The second search is
    constrained to that exact route and inserts feasible morphology changes.
    Failure in the adaptation stage does not trigger a different spatial route;
    this is the intended distinction from joint pose-morphology search.
    """

    def __init__(
        self,
        catalog: Catalog,
        grid: OccupancyGrid,
        heading_bins: int = 16,
        transition_validator: TransitionValidator | None = None,
        cost_model: EdgeCostModel | None = None,
    ) -> None:
        self.hybrid = MorphologyAStar(
            catalog, grid, heading_bins, transition_validator, cost_model)
        self.catalog = catalog
        self.grid = grid
        self.heading_bins = heading_bins

    def plan(self, start: HybridState, goal_xy: tuple[int, int],
             epsilon: float = 1.0) -> HybridPlan:
        if start.morphology not in self.catalog.morphologies:
            raise ValueError(f"unknown start morphology: {start.morphology}")
        if not self.hybrid._state_is_free(start):
            raise NoPathError("start state is in collision")
        route, spatial_expanded = self._spatial_route(
            SpatialState(start.x, start.y, start.heading), goal_xy, epsilon)
        return self._adapt(route, start.morphology, epsilon, spatial_expanded)

    def _spatial_route(self, start: SpatialState, goal_xy: tuple[int, int],
                       epsilon: float) -> tuple[tuple[SpatialState, ...], int]:
        serial = count()
        queue: list[tuple[float, int, SpatialState]] = []
        cost = {start: 0.0}
        previous: dict[SpatialState, SpatialState] = {}
        heappush(queue, (epsilon * self._spatial_heuristic(start, goal_xy),
                         next(serial), start))
        expanded = 0
        while queue:
            _, _, current = heappop(queue)
            expanded += 1
            if (current.x, current.y) == goal_xy:
                route = [current]
                while route[-1] in previous:
                    route.append(previous[route[-1]])
                route.reverse()
                return tuple(route), expanded
            for target, edge_cost in self._spatial_neighbors(current).items():
                candidate = cost[current] + edge_cost
                if candidate >= cost.get(target, float("inf")):
                    continue
                cost[target] = candidate
                previous[target] = current
                priority = candidate + epsilon * self._spatial_heuristic(target, goal_xy)
                heappush(queue, (priority, next(serial), target))
        raise NoPathError("no morphology-agnostic spatial route")

    def _spatial_neighbors(self, state: SpatialState) -> dict[SpatialState, float]:
        neighbors: dict[SpatialState, float] = {}
        for morphology in self.catalog.morphologies.values():
            source = HybridState(state.x, state.y, state.heading, morphology.id)
            for target, _ in self.hybrid._motion_neighbors(source, morphology):
                spatial = SpatialState(target.x, target.y, target.heading)
                distance = hypot(target.x - source.x, target.y - source.y) * self.grid.resolution
                turn = abs((target.heading - source.heading) % self.heading_bins)
                turn = min(turn, self.heading_bins - turn) * 2.0 * pi / self.heading_bins
                geometric = distance + 0.1 * turn
                neighbors[spatial] = min(neighbors.get(spatial, float("inf")), geometric)
        return neighbors

    def _spatial_heuristic(self, state: SpatialState, goal: tuple[int, int]) -> float:
        return hypot(goal[0] - state.x, goal[1] - state.y) * self.grid.resolution

    def _adapt(self, route: tuple[SpatialState, ...], start_morphology: str,
               epsilon: float, spatial_expanded: int) -> HybridPlan:
        start = (0, start_morphology)
        serial = count()
        queue: list[tuple[float, int, tuple[int, str]]] = [(0.0, next(serial), start)]
        costs = {start: 0.0}
        previous: dict[tuple[int, str], tuple[tuple[int, str], HybridSegment]] = {}
        expanded = 0
        goal: tuple[int, str] | None = None
        while queue:
            queued, _, current = heappop(queue)
            if queued > costs[current] + 1e-9:
                continue
            expanded += 1
            route_index, morphology_id = current
            pose = route[route_index]
            source = HybridState(pose.x, pose.y, pose.heading, morphology_id)
            if route_index == len(route) - 1:
                goal = current
                break
            for transition in self.catalog.outgoing(morphology_id):
                if not self.hybrid.transition_validator(transition, source):
                    continue
                target_state = HybridState(pose.x, pose.y, pose.heading, transition.target)
                segment = HybridSegment(
                    "reconfigure", source, target_state,
                    self.hybrid._transition_cost(transition), transition.id)
                self._relax(current, (route_index, transition.target), segment,
                            costs, previous, queue, serial)
            next_pose = route[route_index + 1]
            expected = HybridState(next_pose.x, next_pose.y, next_pose.heading, morphology_id)
            for target, segment in self.hybrid._motion_neighbors(
                    source, self.catalog.morphologies[morphology_id]):
                if target == expected:
                    self._relax(current, (route_index + 1, morphology_id), segment,
                                costs, previous, queue, serial)
                    break
        if goal is None:
            raise NoPathError("fixed spatial route cannot be adapted with feasible transitions")
        segments: list[HybridSegment] = []
        cursor = goal
        while cursor in previous:
            cursor, segment = previous[cursor]
            segments.append(segment)
        segments.reverse()
        states = [segments[0].source] if segments else [HybridState(
            route[0].x, route[0].y, route[0].heading, start_morphology)]
        states.extend(segment.target for segment in segments)
        return HybridPlan(
            tuple(states), tuple(segments), costs[goal],
            spatial_expanded + expanded, epsilon, self.grid.revision,
        )

    @staticmethod
    def _relax(source, target, segment, costs, previous, queue, serial) -> None:
        candidate = costs[source] + segment.cost
        if candidate >= costs.get(target, float("inf")):
            return
        costs[target] = candidate
        previous[target] = (source, segment)
        heappush(queue, (candidate, next(serial), target))

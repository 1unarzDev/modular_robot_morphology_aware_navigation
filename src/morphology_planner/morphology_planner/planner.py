from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
from itertools import count
from math import ceil, cos, hypot, pi, sin
from typing import Callable, Literal

from .catalog import Catalog, Morphology, Transition
from .cost_model import AnalyticCostModel, EdgeCostModel
from .grid import OccupancyGrid


@dataclass(frozen=True, order=True)
class HybridState:
    x: int
    y: int
    heading: int
    morphology: str


@dataclass(frozen=True)
class HybridSegment:
    kind: Literal["traverse", "reconfigure"]
    source: HybridState
    target: HybridState
    cost: float
    transition_id: str = ""


@dataclass(frozen=True)
class HybridPlan:
    states: tuple[HybridState, ...]
    segments: tuple[HybridSegment, ...]
    total_cost: float
    expanded: int
    epsilon: float
    map_revision: int
    method_id: str = ""
    topology_revision: int = 0
    sensing_revision: int = 0


class NoPathError(RuntimeError):
    pass


TransitionValidator = Callable[[Transition, HybridState], bool]


class MorphologyAStar:
    """Weighted hybrid A* over grid pose and a finite morphology graph.

    Reconfiguration edges are validated only after their source reaches the open
    list, keeping expensive swept-volume checks out of speculative expansion.
    Calling ``plan_anytime`` decreases epsilon and returns the best available
    plan; a changed grid revision naturally produces a repaired plan on the next
    call.
    """

    def __init__(
        self,
        catalog: Catalog,
        grid: OccupancyGrid,
        heading_bins: int = 16,
        transition_validator: TransitionValidator | None = None,
        cost_model: EdgeCostModel | None = None,
    ) -> None:
        if heading_bins < 4 or heading_bins % 4:
            raise ValueError("heading_bins must be a multiple of four")
        self.catalog = catalog
        self.grid = grid
        self.heading_bins = heading_bins
        self.transition_validator = transition_validator or self._validate_transition
        self.cost_model = cost_model or AnalyticCostModel()
        self._state_collision_cache: dict[HybridState, bool] = {}
        self._traversal_collision_cache: dict[tuple[HybridState, HybridState], bool] = {}

    def plan_anytime(
        self,
        start: HybridState,
        goal_xy: tuple[int, int],
        epsilons: tuple[float, ...] = (2.5, 1.75, 1.0),
    ) -> tuple[HybridPlan, ...]:
        plans = []
        incumbent = float("inf")
        for epsilon in epsilons:
            try:
                plan = self.plan(start, goal_xy, epsilon=epsilon, upper_bound=incumbent)
            except NoPathError:
                # The incumbent bound can rule out every equal-cost path. Keep
                # the best plan already found rather than turning refinement
                # into a planning failure.
                if plans:
                    continue
                raise
            if plan.total_cost <= incumbent:
                plans.append(plan)
                incumbent = plan.total_cost
        return tuple(plans)

    def plan(
        self,
        start: HybridState,
        goal_xy: tuple[int, int],
        epsilon: float = 1.0,
        upper_bound: float = float("inf"),
    ) -> HybridPlan:
        if start.morphology not in self.catalog.morphologies:
            raise ValueError(f"unknown start morphology: {start.morphology}")
        if epsilon < 1.0:
            raise ValueError("epsilon must be at least one")
        if not self._state_is_free(start):
            raise NoPathError("start state is in collision")

        serial = count()
        open_heap: list[tuple[float, int, HybridState]] = []
        g_score = {start: 0.0}
        predecessor: dict[HybridState, tuple[HybridState, HybridSegment]] = {}
        heappush(open_heap, (epsilon * self._heuristic(start, goal_xy), next(serial), start))
        expanded = 0

        while open_heap:
            queued_f, _, current = heappop(open_heap)
            current_g = g_score[current]
            expected_f = current_g + epsilon * self._heuristic(current, goal_xy)
            if queued_f > expected_f + 1e-9:
                continue
            if current_g + self._heuristic(current, goal_xy) > upper_bound:
                continue
            expanded += 1
            if (current.x, current.y) == goal_xy:
                return self._reconstruct(current, predecessor, current_g, expanded, epsilon)

            for target, segment in self._neighbors(current):
                tentative = current_g + segment.cost
                if tentative >= g_score.get(target, float("inf")) or tentative > upper_bound:
                    continue
                g_score[target] = tentative
                predecessor[target] = (current, segment)
                priority = tentative + epsilon * self._heuristic(target, goal_xy)
                heappush(open_heap, (priority, next(serial), target))

        raise NoPathError("no collision-free hybrid plan")

    def _neighbors(self, state: HybridState):
        morphology = self.catalog.morphologies[state.morphology]
        yield from self._motion_neighbors(state, morphology)
        for transition in self.catalog.outgoing(state.morphology):
            if not self.transition_validator(transition, state):
                continue
            target = HybridState(state.x, state.y, state.heading, transition.target)
            cost = self._transition_cost(transition)
            yield target, HybridSegment("reconfigure", state, target, cost, transition.id)

    def _motion_neighbors(self, state: HybridState, morphology: Morphology):
        yaw = self._yaw(state.heading)
        mode = morphology.locomotion_mode
        actions: list[tuple[int, int, int]] = []
        forward = self._quantized_direction(yaw)
        actions.append((forward[0], forward[1], 0))
        actions.append((-forward[0], -forward[1], 0))
        actions.extend([(0, 0, 1), (0, 0, -1)])
        if mode == "omnidirectional":
            lateral = self._quantized_direction(yaw + pi / 2)
            actions.extend([(lateral[0], lateral[1], 0), (-lateral[0], -lateral[1], 0)])
        if mode in {"ackermann", "articulated"}:
            actions.extend([(forward[0], forward[1], 1), (forward[0], forward[1], -1)])

        for dx, dy, dheading in actions:
            target = HybridState(
                state.x + dx,
                state.y + dy,
                (state.heading + dheading) % self.heading_bins,
                state.morphology,
            )
            if target == state or not self._traversal_is_free(state, target):
                continue
            distance = hypot(dx, dy) * self.grid.resolution
            angle = abs(dheading) * 2 * pi / self.heading_bins
            estimate = self.cost_model.traversal(morphology, distance, angle)
            cost = estimate.objective_cost(self.catalog.objective)
            wx, wy = self.grid.cell_center(target.x, target.y)
            unknown_fraction = self.grid.footprint_unknown_fraction(
                wx, wy, self._yaw(target.heading), morphology.footprint
            )
            cost += self.catalog.objective.lambda_risk * self.catalog.objective.unknown_space_risk * unknown_fraction
            yield target, HybridSegment("traverse", state, target, cost)

    def _state_is_free(self, state: HybridState) -> bool:
        cached = self._state_collision_cache.get(state)
        if cached is not None:
            return cached
        if not self.grid.in_bounds(state.x, state.y):
            return False
        wx, wy = self.grid.cell_center(state.x, state.y)
        morphology = self.catalog.morphologies[state.morphology]
        result = self.grid.footprint_is_free(
            wx, wy, self._yaw(state.heading), morphology.footprint)
        self._state_collision_cache[state] = result
        return result

    def _traversal_is_free(self, source: HybridState, target: HybridState) -> bool:
        """Conservatively sample the swept footprint for one lattice primitive."""
        cache_key = (source, target)
        cached = self._traversal_collision_cache.get(cache_key)
        if cached is not None:
            return cached
        if source.morphology != target.morphology:
            return False
        source_x, source_y = self.grid.cell_center(source.x, source.y)
        target_x, target_y = self.grid.cell_center(target.x, target.y)
        source_yaw = self._yaw(source.heading)
        delta_heading = (target.heading - source.heading) % self.heading_bins
        if delta_heading > self.heading_bins // 2:
            delta_heading -= self.heading_bins
        delta_yaw = delta_heading * 2.0 * pi / self.heading_bins
        morphology = self.catalog.morphologies[source.morphology]
        swept_distance = max(
            hypot(target_x - source_x, target_y - source_y),
            abs(delta_yaw) * morphology.radius,
        )
        samples = max(1, ceil(swept_distance / (0.25 * self.grid.resolution)))
        for index in range(samples + 1):
            fraction = index / samples
            wx = source_x + fraction * (target_x - source_x)
            wy = source_y + fraction * (target_y - source_y)
            yaw = source_yaw + fraction * delta_yaw
            if not self.grid.footprint_is_free(wx, wy, yaw, morphology.footprint):
                self._traversal_collision_cache[cache_key] = False
                return False
        self._traversal_collision_cache[cache_key] = True
        return True

    def _validate_transition(self, transition: Transition, state: HybridState) -> bool:
        wx, wy = self.grid.cell_center(state.x, state.y)
        if not self.grid.disk_is_free(wx, wy, transition.swept_radius):
            return False
        target = HybridState(state.x, state.y, state.heading, transition.target)
        return self._state_is_free(target)

    def _transition_cost(self, transition: Transition) -> float:
        return self.cost_model.reconfiguration(transition).objective_cost(self.catalog.objective)

    def _heuristic(self, state: HybridState, goal_xy: tuple[int, int]) -> float:
        distance = hypot(goal_xy[0] - state.x, goal_xy[1] - state.y) * self.grid.resolution
        maximum_speed = max(m.limits.linear for m in self.catalog.morphologies.values())
        return distance / maximum_speed

    def _yaw(self, heading: int) -> float:
        return heading * 2 * pi / self.heading_bins

    @staticmethod
    def _quantized_direction(yaw: float) -> tuple[int, int]:
        dx, dy = round(cos(yaw)), round(sin(yaw))
        return int(dx), int(dy)

    def _reconstruct(self, goal, predecessor, cost, expanded, epsilon) -> HybridPlan:
        states = [goal]
        segments = []
        cursor = goal
        while cursor in predecessor:
            previous, segment = predecessor[cursor]
            states.append(previous)
            segments.append(segment)
            cursor = previous
        states.reverse()
        segments.reverse()
        return HybridPlan(
            states=tuple(states), segments=tuple(segments), total_cost=cost,
            expanded=expanded, epsilon=epsilon, map_revision=self.grid.revision,
        )

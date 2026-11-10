"""Site exclusion for the pre-detach-check baseline (proposal, predetach_check_v1).

The checked baseline is `feasibility_coupled` with sensing in the loop and no
prediction model: on arriving at a planned transition site, before anything is
detached, the navigator asks whether the station *currently* sees every pod the
transition will move (the pods are still at their source-morphology ports). If
any is unseen the site is rejected without transforming, and the planner is
asked again with that rejection as a constraint. This module is the planner
half: it turns rejections into a transition-edge exclusion.

What a rejection may use is what the robot holds: the site it stood at (its own
plan), which pods its estimator reported unobserved, the catalog geometry, and
the station position declared with the map (the same prior
`station_sensing_provider` reads). It never reads the occluder geometry: the
`ray_wedge` rule extends the observed shadow along the station's sight line,
it does not intersect anything with the screen.

Rules (each rejection carries its own, so a mission's history is explicit):

* ``radius``: exclude every pod-moving transition, at every heading, whose site
  centre lies within ``radius_m`` of the rejected site centre.
* ``ray_wedge``: for each unseen pod, the wedge with apex at the station, axis
  towards that pod's source-port position at the rejected site, half-angle
  ``wedge_deg``, from that pod's range outwards. A candidate is excluded when any
  pod it would move has its source-port position inside a wedge. Source ports
  are what the check will test at the candidate, so the rule excludes exactly
  the candidates whose check is predicted, by the observed shadow alone, to
  fail again.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from math import acos, cos, pi, radians, sin, sqrt
from typing import Iterable

from .catalog import Catalog, Transition
from .grid import OccupancyGrid
from .observability import FiducialStation
from .planner import HybridState
from .transition_policy import TransitionDecision


CHECKED_METHODS = ("feasibility_checked", "feasibility_probed")
EXCLUSION_RULES = ("radius", "ray_wedge")
DEFAULT_RADIUS_M = 0.3
DEFAULT_WEDGE_DEG = 10.0


@dataclass(frozen=True)
class SiteRejection:
    """One failed pre-detach check, as the navigator reports it."""

    transition_id: str
    x: float
    y: float
    yaw: float
    unseen_pods: tuple[str, ...]
    rule: str = "radius"
    radius_m: float = DEFAULT_RADIUS_M
    wedge_deg: float = DEFAULT_WEDGE_DEG
    # `feasibility_probed` only: pods whose *docked* position the compact body
    # probed and the station did not see. Their wedges are cast through the
    # target-morphology port, and test candidates' target ports.
    unseen_docked_pods: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if self.rule not in EXCLUSION_RULES:
            raise ValueError(f"unknown exclusion rule: {self.rule}")
        if self.radius_m < 0.0 or not 0.0 < self.wedge_deg < 90.0:
            raise ValueError("exclusion radius/wedge out of range")

    def to_dict(self) -> dict:
        return {"transition_id": self.transition_id, "x": self.x, "y": self.y,
                "yaw": self.yaw, "unseen_pods": list(self.unseen_pods),
                "rule": self.rule, "radius_m": self.radius_m,
                "wedge_deg": self.wedge_deg,
                "unseen_docked_pods": list(self.unseen_docked_pods)}

    @classmethod
    def from_dict(cls, value: dict) -> "SiteRejection":
        return cls(str(value["transition_id"]), float(value["x"]), float(value["y"]),
                   float(value["yaw"]),
                   tuple(str(pod) for pod in value.get("unseen_pods", ())),
                   str(value.get("rule", "radius")),
                   float(value.get("radius_m", DEFAULT_RADIUS_M)),
                   float(value.get("wedge_deg", DEFAULT_WEDGE_DEG)),
                   tuple(str(pod) for pod in value.get("unseen_docked_pods", ())))


def parse_site_exclusions(payload: str) -> tuple[SiteRejection, ...]:
    """The planner request's `site_exclusions_json`: empty, or a JSON list."""
    if not payload:
        return ()
    value = json.loads(payload)
    if not isinstance(value, list):
        raise ValueError("site_exclusions_json must be a JSON list")
    return tuple(SiteRejection.from_dict(entry) for entry in value)


def dump_site_exclusions(rejections: Iterable[SiteRejection]) -> str:
    return json.dumps([value.to_dict() for value in rejections], sort_keys=True)


def port_point(catalog: Catalog, morphology_id: str, pod: str,
               x: float, y: float, yaw: float) -> tuple[float, float, float]:
    """A pod's port position in the map frame, planner convention (z = h/2)."""
    px, py, _ = catalog.morphologies[morphology_id].pod_poses[pod]
    height = catalog.module_sizes[pod][2]
    return (x + cos(yaw) * px - sin(yaw) * py,
            y + sin(yaw) * px + cos(yaw) * py,
            height / 2.0)


def _angle(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    na = sqrt(sum(v * v for v in a))
    nb = sqrt(sum(v * v for v in b))
    if na < 1e-12 or nb < 1e-12:
        return 0.0
    dot = sum(p * q for p, q in zip(a, b)) / (na * nb)
    return acos(max(-1.0, min(1.0, dot)))


@dataclass(frozen=True)
class _Wedge:
    apex: tuple[float, float, float]
    axis: tuple[float, float, float]
    range_m: float
    half_angle: float
    docked: bool = False


class SiteExclusionFilter:
    """Whether a (transition, state) candidate is excluded by past rejections."""

    def __init__(self, catalog: Catalog, grid: OccupancyGrid, heading_bins: int,
                 stations: tuple[FiducialStation, ...],
                 rejections: tuple[SiteRejection, ...]) -> None:
        self.catalog = catalog
        self.grid = grid
        self.heading_bins = heading_bins
        self.rejections = tuple(rejections)
        self._radii = tuple((r.x, r.y, r.radius_m) for r in self.rejections
                            if r.rule == "radius")
        wedges = []
        transitions = {t.id: t for t in catalog.transitions}
        for rejection in self.rejections:
            if rejection.rule != "ray_wedge":
                continue
            transition = transitions[rejection.transition_id]
            points = [(port_point(catalog, transition.source, pod, rejection.x,
                                  rejection.y, rejection.yaw), False)
                      for pod in rejection.unseen_pods]
            points += [(port_point(catalog, transition.target, pod, rejection.x,
                                   rejection.y, rejection.yaw), True)
                       for pod in rejection.unseen_docked_pods]
            for point, docked in points:
                # Every declared station: the rejection says none of them saw
                # the pod, so the shadow extends behind it from each.
                for station in stations:
                    axis = tuple(p - s for p, s in zip(point, station.position))
                    wedges.append(_Wedge(station.position, axis,
                                         sqrt(sum(v * v for v in axis)),
                                         radians(rejection.wedge_deg), docked))
        self._wedges = tuple(wedges)

    def __bool__(self) -> bool:
        return bool(self.rejections)

    def excluded_by(self, transition: Transition, state: HybridState) -> str | None:
        if not transition.moved_pods:
            return None
        wx, wy = self.grid.cell_center(state.x, state.y)
        for x, y, radius in self._radii:
            if (wx - x) ** 2 + (wy - y) ** 2 <= radius * radius + 1e-12:
                return "predetach_excluded:radius"
        if self._wedges:
            yaw = state.heading * 2.0 * pi / self.heading_bins
            for pod in transition.moved_pods:
                source = port_point(self.catalog, transition.source, pod, wx, wy, yaw)
                target = port_point(self.catalog, transition.target, pod, wx, wy, yaw)
                for wedge in self._wedges:
                    point = target if wedge.docked else source
                    ray = tuple(p - a for p, a in zip(point, wedge.apex))
                    distance = sqrt(sum(v * v for v in ray))
                    if (distance >= wedge.range_m - 1e-9
                            and _angle(ray, wedge.axis) <= wedge.half_angle + 1e-12):
                        return "predetach_excluded:ray_wedge"
        return None


class ExcludingTransitionPolicy:
    """A transition policy with pre-detach rejections applied first.

    Wraps the shipped `CoupledTransitionPolicy` unchanged: a candidate that no
    rejection excludes is decided by the wrapped policy exactly as before, and
    an excluded one is recorded in the same decision list with its own reason.
    """

    def __init__(self, policy, exclusions: SiteExclusionFilter) -> None:
        self.policy = policy
        self.exclusions = exclusions

    @property
    def decisions(self) -> list[TransitionDecision]:
        return self.policy.decisions

    @property
    def method(self) -> str:
        return self.policy.method

    def __call__(self, transition: Transition, state: HybridState) -> bool:
        reason = self.exclusions.excluded_by(transition, state)
        if reason is None:
            return self.policy(transition, state)
        self.policy.decisions.append(TransitionDecision(
            self.policy.method, transition.id, state, False, (reason,), 0))
        return False

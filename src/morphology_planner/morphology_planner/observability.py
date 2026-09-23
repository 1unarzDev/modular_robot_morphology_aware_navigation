"""Station-based connector observability (ADR 0006).

Precision docking depends on an external workspace fiducial station: a pod is
well observed only where the station can actually see it. That makes
observability a location-dependent property with a physical cause, which
occluding the onboard connector cameras cannot be -- those are mounted at the
core origin and the pods relocate radially outward, so every sight-line-blocking
placement falls inside the robot's own footprint.

This module is pure geometry and carries no ROS or simulator dependency. The
simulator synthesizes station observations from it, and the planner predicts
them from it using only priors it already holds (the station pose declared with
the map, and the `transition_environment` boxes). Prediction and measurement are
therefore independent computations of the same physical fact.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt

from .transition_validation import Box3


# Covariance trace a station-refined observation achieves, as a function of
# range from the observer. Matches the onboard connector camera's existing
# model (`sensing_node.py`): 2 * (1e-6 + 1e-5 r^2) + (1e-6 + 2e-5 r^2).
PRECISE_BASE_TRACE = 3e-6
PRECISE_RANGE_COEFFICIENT = 4e-5

# Covariance trace attributed to a pod the station cannot see. The onboard
# connector cameras still report it, but a short-range relative sensor with no
# absolute reference does not support precision docking. 0.04 is the value the
# confirmatory scenarios already declared for a poorly observed region, and is
# 2.7x the 0.015 acceptance gate in both the planner predicate and
# `docking_acceptance`.
STATION_UNOBSERVED_TRACE = 0.04


@dataclass(frozen=True)
class FiducialStation:
    """A fixed workspace observer declared with the world, in the map frame."""

    name: str
    position: tuple[float, float, float]
    max_range: float = 8.0

    def range_to(self, point: tuple[float, float, float]) -> float:
        return sqrt(sum((a - b) ** 2 for a, b in zip(self.position, point)))


def precise_trace(range_m: float) -> float:
    return PRECISE_BASE_TRACE + PRECISE_RANGE_COEFFICIENT * range_m * range_m


def segment_intersects_box(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    box: Box3,
) -> bool:
    """Slab test for a segment against an axis-aligned box.

    Touching a face is not an intersection, so a sight line that grazes an
    obstacle edge stays clear, following the same epsilon convention as
    `_overlap` in `transition_validation`.
    """
    low = tuple(box.center[i] - box.size[i] / 2.0 for i in range(3))
    high = tuple(box.center[i] + box.size[i] / 2.0 for i in range(3))
    enter, exit_ = 0.0, 1.0
    for axis in range(3):
        origin = start[axis]
        delta = end[axis] - origin
        if abs(delta) < 1e-12:
            if origin <= low[axis] + 1e-9 or origin >= high[axis] - 1e-9:
                return False
            continue
        first = (low[axis] - origin) / delta
        second = (high[axis] - origin) / delta
        if first > second:
            first, second = second, first
        enter = max(enter, first)
        exit_ = min(exit_, second)
        if enter >= exit_ - 1e-9:
            return False
    return True


def station_observes(
    station: FiducialStation,
    point: tuple[float, float, float],
    obstacles: tuple[Box3, ...] = (),
) -> bool:
    """Whether the station has an unoccluded sight line to `point` in range."""
    if station.range_to(point) > station.max_range:
        return False
    return not any(
        segment_intersects_box(station.position, point, obstacle)
        for obstacle in obstacles
    )


def observed_by_any(
    stations: tuple[FiducialStation, ...],
    point: tuple[float, float, float],
    obstacles: tuple[Box3, ...] = (),
) -> FiducialStation | None:
    """The nearest station with a clear line to `point`, or None."""
    visible = [
        station for station in stations
        if station_observes(station, point, obstacles)
    ]
    if not visible:
        return None
    return min(visible, key=lambda station: station.range_to(point))


def load_fiducial_stations(path: str) -> tuple[FiducialStation, ...]:
    """Stations declared in a scenario manifest, beside its 3D obstacles.

    An empty path, or a manifest without the key, means no station is declared
    and observability is not station-gated -- which is the behaviour every
    world had before ADR 0006.
    """
    if not path:
        return ()
    import json

    with open(path, encoding="utf-8") as stream:
        manifest = json.load(stream)
    return tuple(
        FiducialStation(
            str(station["name"]),
            tuple(float(value) for value in station["position"]),
            float(station.get("max_range", 8.0)),
        )
        for station in manifest.get("fiducial_stations", ())
    )

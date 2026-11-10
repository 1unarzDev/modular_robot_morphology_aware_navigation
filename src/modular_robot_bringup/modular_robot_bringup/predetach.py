"""Robot-side half of the pre-detach-check baseline (proposal, predetach_check_v1).

Pure logic, no ROS, so it is host-testable; `hybrid_navigator` feeds it.

The only input about visibility is the estimator's per-pod
`RelativePoseEstimate` stream (`relative_pose_estimate`): its
`connector_visible` is true only while a station (`fiducial`) observation of
that pod is younger than the estimator's 0.5 s staleness window
(`reconfiguration_executor.sensing.RelativePoseEstimator`). Nothing here reads
simulator state, the occluder geometry or evaluator truth; the ground-truth
guard covers this package.

Decision rule. After the robot is aligned at the site and stationary, the
navigator dwells `dwell_s` (default 1.0 s, which must exceed the 0.5 s
staleness window) and reads the estimates stamped after
`window_start = dwell start + stale_window_s`, so every estimate it reads was
computed only from observations taken while the robot stood at the site. A
pod is *seen* when it has at least one such estimate and the latest one reports
`connector_visible`. A pod with no fresh estimate is unseen (fail closed).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from math import cos, sin


CHECKED_METHODS = ("feasibility_checked", "feasibility_probed")
PROBED_METHOD = "feasibility_probed"
EXCLUSION_RULES = ("radius", "ray_wedge")
ESTIMATOR_STALE_WINDOW_S = 0.5
DEFAULT_DWELL_S = 1.0
DEFAULT_MAX_ATTEMPTS = 8
# A probe is a straight move of the still-compact body along its own heading;
# beyond this the pose may leave the transition's swept-clearance disk.
DEFAULT_MAX_PROBE_OFFSET_M = 0.5
EVENT_TOPIC = "predetach_check_events"


def validate_dwell(dwell_s: float, stale_window_s: float = ESTIMATOR_STALE_WINDOW_S) -> float:
    if not dwell_s > stale_window_s:
        raise ValueError(
            f"predetach dwell {dwell_s} s must exceed the estimator's "
            f"{stale_window_s} s staleness window")
    return float(dwell_s)


@dataclass
class EstimateWindow:
    """Recent per-pod (stamp, connector_visible) samples from the estimator."""

    horizon_s: float = 10.0
    _samples: dict[str, deque] = field(default_factory=dict)

    def add(self, pod_id: str, stamp_s: float, connector_visible: bool,
            sources: tuple[str, ...] = ()) -> None:
        samples = self._samples.setdefault(pod_id, deque())
        samples.append((float(stamp_s), bool(connector_visible), tuple(sources)))
        while samples and samples[0][0] < stamp_s - self.horizon_s:
            samples.popleft()

    def assess(self, pods, window_start_s: float) -> dict[str, dict]:
        """Per pod: seen, the fresh sample count, and how many were visible."""
        result = {}
        for pod in pods:
            fresh = [s for s in self._samples.get(pod, ()) if s[0] >= window_start_s]
            result[pod] = {
                "seen": bool(fresh) and fresh[-1][1],
                "fresh_samples": len(fresh),
                "visible_samples": sum(1 for s in fresh if s[1]),
                "last_sources": list(fresh[-1][2]) if fresh else [],
            }
        return result


def unseen(assessment: dict[str, dict]) -> tuple[str, ...]:
    return tuple(pod for pod, value in assessment.items() if not value["seen"])


@dataclass(frozen=True)
class Probe:
    """Stand the source body `offset_m` along its heading from the site, where
    each `probe_pod`'s source port coincides with `docked_pod`'s target port."""

    offset_m: float
    pairs: tuple[tuple[str, str], ...]   # (probe_pod, docked_pod)


def probe_plan(source_ports: dict[str, tuple[float, float, float]],
               target_ports: dict[str, tuple[float, float, float]],
               moved_pods, max_offset_m: float = DEFAULT_MAX_PROBE_OFFSET_M,
               tolerance_m: float = 1e-6) -> tuple[tuple[Probe, ...], tuple[str, ...]]:
    """Probe poses that put some pod's fiducial on every moved pod's docked point.

    Only pure translations along the body's heading are used: the body keeps the
    site heading, so the probe needs no turn and stays on the site's line. For
    each moved pod, the probe pod with the same lateral offset and the smallest
    |offset| is chosen. Returns the probes (sorted by offset) and the moved
    pods no probe can reach (they fall back to the compact-port check).
    """
    by_offset: dict[float, list[tuple[str, str]]] = {}
    unreachable = []
    for docked in moved_pods:
        tx, ty, _ = target_ports[docked]
        best = None
        for probe, (sx, sy, _) in sorted(source_ports.items()):
            if abs(sy - ty) > tolerance_m:
                continue
            offset = round(tx - sx, 6)
            if abs(offset) > max_offset_m + tolerance_m:
                continue
            if best is None or abs(offset) < abs(best[0]) - tolerance_m:
                best = (offset, probe)
        if best is None:
            unreachable.append(docked)
            continue
        by_offset.setdefault(best[0], []).append((best[1], docked))
    probes = tuple(Probe(offset, tuple(pairs))
                   for offset, pairs in sorted(by_offset.items()))
    return probes, tuple(unreachable)


def offset_pose(x: float, y: float, yaw: float, offset_m: float) -> tuple[float, float, float]:
    return (x + offset_m * cos(yaw), y + offset_m * sin(yaw), yaw)


@dataclass
class CheckLedger:
    """What the record must show: every check, rejection and replan, in order."""

    method: str
    rule: str
    radius_m: float
    wedge_deg: float
    dwell_s: float
    max_attempts: int
    events: list[dict] = field(default_factory=list)
    rejections: list[dict] = field(default_factory=list)

    def __post_init__(self) -> None:
        if self.method not in CHECKED_METHODS:
            raise ValueError(f"not a checked method: {self.method}")
        if self.rule not in EXCLUSION_RULES:
            raise ValueError(f"unknown exclusion rule: {self.rule}")
        validate_dwell(self.dwell_s)
        if self.max_attempts < 1:
            raise ValueError("max_attempts must be at least 1")

    @property
    def rejection_count(self) -> int:
        return len(self.rejections)

    @property
    def cap_reached(self) -> bool:
        return self.rejection_count >= self.max_attempts

    def record_check(self, time_s: float, transition_id: str,
                     site: tuple[float, float, float],
                     actual: tuple[float, float, float] | None,
                     compact: dict[str, dict], probes: list[dict]) -> dict:
        unseen_source = unseen(compact)
        unseen_docked = tuple(sorted({
            pair["docked_pod"] for probe in probes for pair in probe["pairs"]
            if not pair["seen"]}))
        passed = not unseen_source and not unseen_docked
        event = {
            "event": "check", "index": len([e for e in self.events
                                            if e["event"] == "check"]),
            "time_s": time_s, "transition_id": transition_id,
            "site": {"x": site[0], "y": site[1], "yaw": site[2]},
            "actual_pose": (None if actual is None else
                            {"x": actual[0], "y": actual[1], "yaw": actual[2]}),
            "pods": compact, "probes": probes,
            "unseen_pods": list(unseen_source),
            "unseen_docked_pods": list(unseen_docked),
            "passed": passed,
        }
        self.events.append(event)
        if not passed:
            rejection = {
                "transition_id": transition_id, "x": site[0], "y": site[1],
                "yaw": site[2], "unseen_pods": list(unseen_source),
                "unseen_docked_pods": list(unseen_docked), "rule": self.rule,
                "radius_m": self.radius_m, "wedge_deg": self.wedge_deg,
            }
            self.rejections.append(rejection)
            self.events.append({"event": "rejection", "time_s": time_s,
                                "index": self.rejection_count - 1, **rejection})
        return event

    def record(self, event: str, time_s: float, **detail) -> dict:
        value = {"event": event, "time_s": time_s, **detail}
        self.events.append(value)
        return value

    def parameters(self) -> dict:
        return {"method": self.method, "rule": self.rule, "radius_m": self.radius_m,
                "wedge_deg": self.wedge_deg, "dwell_s": self.dwell_s,
                "max_attempts": self.max_attempts,
                "stale_window_s": ESTIMATOR_STALE_WINDOW_S}

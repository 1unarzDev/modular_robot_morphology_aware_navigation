from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml


@dataclass(frozen=True)
class Limits:
    linear: float
    angular: float
    acceleration: float
    lateral: float = 0.0
    min_turning_radius: float = 0.0


@dataclass(frozen=True)
class Morphology:
    id: str
    locomotion_mode: str
    footprint: tuple[tuple[float, float], ...]
    height: float
    limits: Limits
    controller_id: str
    experiment_supported: bool = False
    pod_poses: Mapping[str, tuple[float, float, float]] = field(default_factory=dict)

    @property
    def radius(self) -> float:
        return max((x * x + y * y) ** 0.5 for x, y in self.footprint)


@dataclass(frozen=True)
class Transition:
    id: str
    source: str
    target: str
    time: float
    energy: float
    failure_probability: float
    swept_radius: float
    moved_pods: tuple[str, ...]
    pod_waypoints: Mapping[str, tuple[tuple[float, float, float], ...]] = field(default_factory=dict)


@dataclass(frozen=True)
class Objective:
    lambda_energy: float
    lambda_risk: float
    unknown_space_risk: float


@dataclass(frozen=True)
class Catalog:
    morphologies: Mapping[str, Morphology]
    transitions: tuple[Transition, ...]
    objective: Objective
    module_sizes: Mapping[str, tuple[float, float, float]] = field(default_factory=dict)
    module_collision_boxes: Mapping[
        str, tuple[tuple[str, tuple[float, float, float], tuple[float, float, float]], ...]
    ] = field(default_factory=dict)

    def outgoing(self, morphology_id: str) -> tuple[Transition, ...]:
        return tuple(t for t in self.transitions if t.source == morphology_id)

    def supported_experiment_subset(self) -> "Catalog":
        """Return only morphologies backed by the current simulated mechanics."""
        morphologies = {
            key: value for key, value in self.morphologies.items()
            if value.experiment_supported
        }
        transitions = tuple(
            transition for transition in self.transitions
            if transition.source in morphologies and transition.target in morphologies
        )
        if not morphologies:
            raise ValueError("catalog has no experiment-supported morphologies")
        return Catalog(
            morphologies, transitions, self.objective, self.module_sizes,
            self.module_collision_boxes,
        )


def _positive(value: Any, field: str) -> float:
    number = float(value)
    if not isfinite(number) or number <= 0:
        raise ValueError(f"{field} must be finite and positive")
    return number


def _finite_tuple(value: Any, length: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != length:
        raise ValueError(f"{field} must contain exactly {length} values")
    result = tuple(float(item) for item in value)
    if not all(isfinite(item) for item in result):
        raise ValueError(f"{field} must contain only finite values")
    return result


def load_catalog(path: str | Path) -> Catalog:
    with Path(path).open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    if raw.get("schema_version") != 1:
        raise ValueError("unsupported morphology catalog schema")

    morphologies: dict[str, Morphology] = {}
    for morphology_id, value in raw["morphologies"].items():
        footprint = tuple((float(p[0]), float(p[1])) for p in value["footprint"])
        if len(footprint) < 3:
            raise ValueError(f"{morphology_id}: footprint needs at least three vertices")
        limits = value["limits"]
        morphologies[morphology_id] = Morphology(
            id=morphology_id,
            locomotion_mode=str(value["locomotion_mode"]),
            footprint=footprint,
            height=_positive(value["height"], f"{morphology_id}.height"),
            limits=Limits(
                linear=_positive(limits["linear"], f"{morphology_id}.linear"),
                angular=_positive(limits["angular"], f"{morphology_id}.angular"),
                acceleration=_positive(limits["acceleration"], f"{morphology_id}.acceleration"),
                lateral=float(limits.get("lateral", 0.0)),
                min_turning_radius=float(limits.get("min_turning_radius", 0.0)),
            ),
            controller_id=str(value["controller_id"]),
            experiment_supported=value.get("experiment_support") == "simulated",
            pod_poses={
                pod: _finite_tuple(pose, 3, f"{morphology_id}.pods.{pod}")
                for pod, pose in value.get("pods", {}).items()
            },
        )

    transitions = []
    seen_ids: set[str] = set()
    for value in raw.get("transitions", []):
        transition_id = str(value["id"])
        if transition_id in seen_ids:
            raise ValueError(f"duplicate transition id: {transition_id}")
        seen_ids.add(transition_id)
        source, target = str(value["from"]), str(value["to"])
        if source not in morphologies or target not in morphologies:
            raise ValueError(f"{transition_id}: unknown endpoint")
        moved_pods = tuple(value.get("moved_pods", []))
        unknown_pods = set(moved_pods) - set(morphologies[source].pod_poses)
        if unknown_pods:
            raise ValueError(f"{transition_id}: unknown moved pods: {sorted(unknown_pods)}")
        waypoint_values = value.get("pod_waypoints", {})
        if set(waypoint_values) - set(moved_pods):
            raise ValueError(f"{transition_id}: waypoints supplied for an unmoved pod")
        missing_target_poses = set(moved_pods) - set(morphologies[target].pod_poses)
        if missing_target_poses:
            raise ValueError(f"{transition_id}: target poses missing for {sorted(missing_target_poses)}")
        probability = float(value["failure_probability"])
        if not 0.0 <= probability < 1.0:
            raise ValueError(f"{transition_id}: invalid failure probability")
        transitions.append(Transition(
            id=transition_id,
            source=source,
            target=target,
            time=_positive(value["time"], f"{transition_id}.time"),
            energy=_positive(value["energy"], f"{transition_id}.energy"),
            failure_probability=probability,
            swept_radius=_positive(value["swept_radius"], f"{transition_id}.swept_radius"),
            moved_pods=moved_pods,
            pod_waypoints={
                pod: tuple(
                    _finite_tuple(pose, 3, f"{transition_id}.pod_waypoints.{pod}")
                    for pose in poses
                )
                for pod, poses in waypoint_values.items()
            },
        ))

    objective = raw["objective"]
    return Catalog(
        morphologies=morphologies,
        transitions=tuple(transitions),
        objective=Objective(
            lambda_energy=float(objective["lambda_energy_s_per_joule"]),
            lambda_risk=float(objective["lambda_risk_seconds"]),
            unknown_space_risk=float(objective["unknown_space_risk"]),
        ),
        module_sizes={
            module: tuple(
                _positive(item, f"inventory.{module}.size") for item in
                _finite_tuple(value["size"], 3, f"inventory.{module}.size")
            )
            for module, value in raw["inventory"].items()
        },
        module_collision_boxes={
            module: tuple(
                (
                    str(box.get("name", f"part_{index}")),
                    _finite_tuple(box.get("center", (0.0, 0.0, 0.0)), 3,
                                  f"inventory.{module}.collision_boxes.{index}.center"),
                    tuple(_positive(item, f"inventory.{module}.collision_boxes.{index}.size")
                          for item in _finite_tuple(
                              box["size"], 3,
                              f"inventory.{module}.collision_boxes.{index}.size")),
                )
                for index, box in enumerate(value.get("collision_boxes", ()))
            )
            for module, value in raw["inventory"].items()
        },
    )

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from pathlib import Path
from typing import Any, Mapping

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

    def outgoing(self, morphology_id: str) -> tuple[Transition, ...]:
        return tuple(t for t in self.transitions if t.source == morphology_id)


def _positive(value: Any, field: str) -> float:
    number = float(value)
    if not isfinite(number) or number <= 0:
        raise ValueError(f"{field} must be finite and positive")
    return number


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
            moved_pods=tuple(value.get("moved_pods", [])),
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
    )


from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
from random import Random
from typing import Iterable


METHODS = (
    "route_first_adaptation",
    "geometry_coupled",
    "feasibility_coupled",
    "sensing_feasibility_coupled",
)
CONFIRMATORY_FAMILIES = (
    "reconfiguration_workspace",
    "docking_observability",
    "combined_constraints",
)
SENSING_FAMILIES = frozenset({"docking_observability", "combined_constraints"})


def _seed(master_seed: int, *parts: object) -> int:
    payload = "|".join((str(master_seed), *(str(part) for part in parts)))
    return int.from_bytes(hashlib.sha256(payload.encode("utf-8")).digest()[:8], "big")


@dataclass(frozen=True)
class TrialSpec:
    trial_id: str
    family: str
    layout_id: str
    replicate: int
    method: str
    method_order: int
    world_seed: int
    sensing_seed: int
    friction_seed: int
    fault_seed: int


@dataclass(frozen=True)
class StudyDesign:
    schema_version: int
    design_kind: str
    layouts_per_family: int
    replicates: int
    master_seed: int
    methods: tuple[str, ...]
    families: tuple[str, ...]
    trials: tuple[TrialSpec, ...]

    @property
    def expected_trials(self) -> int:
        return len(self.trials)

    def canonical_json(self) -> str:
        return json.dumps(asdict(self), sort_keys=True, separators=(",", ":"))

    @property
    def design_hash(self) -> str:
        return hashlib.sha256(self.canonical_json().encode("utf-8")).hexdigest()


def generate_design(
    layouts_per_family: int = 12,
    replicates: int = 3,
    master_seed: int = 20260911,
    families: Iterable[str] = CONFIRMATORY_FAMILIES,
    design_kind: str = "confirmatory",
) -> StudyDesign:
    families = tuple(families)
    if layouts_per_family <= 0 or replicates <= 0:
        raise ValueError("layouts and replicates must be positive")
    trials: list[TrialSpec] = []
    for family in families:
        for layout_index in range(layouts_per_family):
            layout_id = f"{family}-{layout_index:02d}"
            world_seed = _seed(master_seed, design_kind, family, layout_index, "world")
            for replicate in range(replicates):
                sensing_seed = _seed(master_seed, family, layout_index, replicate, "sensing")
                friction_seed = _seed(master_seed, family, layout_index, replicate, "friction")
                fault_seed = _seed(master_seed, family, layout_index, replicate, "fault")
                order = list(METHODS)
                Random(_seed(master_seed, family, layout_index, replicate, "order")).shuffle(order)
                for order_index, method in enumerate(order):
                    trials.append(TrialSpec(
                        trial_id=f"{layout_id}-r{replicate}-{method}", family=family,
                        layout_id=layout_id, replicate=replicate, method=method,
                        method_order=order_index, world_seed=world_seed,
                        sensing_seed=sensing_seed, friction_seed=friction_seed,
                        fault_seed=fault_seed,
                    ))
    return StudyDesign(1, design_kind, layouts_per_family, replicates, master_seed,
                       METHODS, families, tuple(trials))

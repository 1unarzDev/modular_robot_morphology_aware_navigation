from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
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
ROUNDTRIP_DESIGN_KIND = "engineering_roundtrip"
FAULT_MATRIX_DESIGN_KIND = "engineering_fault_matrix"


@dataclass(frozen=True)
class FaultCase:
    """One declared executor fault and the topology reconciliation it must yield."""

    name: str
    injection: str
    expected_reconciled_morphology: str | None


# Faults target the first planned transition, compact_to_narrow, whose pods
# move in the order pod_0, pod_2, pod_1, pod_3, pod_4, pod_5. Failures before
# the first physical detach leave a valid compact topology; failures after a
# detach leave a partial topology that reconciliation must refuse.
FAULT_MATRIX = (
    FaultCase("detach", "detach:pod_0", "compact_diff"),
    FaultCase("stale_observation", "stale_feedback:pod_0", "compact_diff"),
    FaultCase("cancellation", "cancellation:pod_0", "compact_diff"),
    FaultCase("relocation", "relocation:pod_0", None),
    FaultCase("latch", "latch:pod_0", None),
    # pod_0 and pod_2 are latched at narrow ports when pod_1 fails to latch.
    FaultCase("partial_topology", "latch:pod_1", None),
    FaultCase("commit", "manager_commit", "narrow_tandem"),
)


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

    def write_frozen(self, path: str | Path) -> Path:
        """Create an immutable design file, or accept an identical existing file."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(
            {"design_hash": self.design_hash, "design": asdict(self)},
            sort_keys=True, indent=2,
        ) + "\n"
        try:
            stream = path.open("x", encoding="utf-8")
        except FileExistsError:
            if path.read_text(encoding="utf-8") != payload:
                raise FileExistsError(f"refusing to overwrite frozen design: {path}")
            return path
        with stream:
            stream.write(payload)
        return path

    @classmethod
    def read_frozen(cls, path: str | Path) -> "StudyDesign":
        with Path(path).open(encoding="utf-8") as stream:
            envelope = json.load(stream)
        value = dict(envelope["design"])
        value["methods"] = tuple(value["methods"])
        value["families"] = tuple(value["families"])
        value["trials"] = tuple(TrialSpec(**trial) for trial in value["trials"])
        design = cls(**value)
        if envelope.get("design_hash") != design.design_hash:
            raise ValueError("frozen design hash does not match its contents")
        if design.design_kind.startswith("engineering_"):
            # Engineering qualification exercises one execution path; it is
            # never a method comparison.
            if not design.methods or not set(design.methods) <= set(METHODS):
                raise ValueError("frozen design has an unsupported method set")
        elif set(design.methods) != set(METHODS):
            raise ValueError("frozen design has an unsupported method set")
        return design


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


def generate_engineering_design(
    design_kind: str,
    family: str,
    layout_index: int,
    method: str,
    runs: int | None = None,
    master_seed: int = 20260912,
) -> StudyDesign:
    """Single-layout, single-method runs with independent plant seeds per run."""
    if design_kind == FAULT_MATRIX_DESIGN_KIND:
        if runs not in (None, len(FAULT_MATRIX)):
            raise ValueError("fault matrix runs must equal the declared fault count")
        runs = len(FAULT_MATRIX)
    elif design_kind != ROUNDTRIP_DESIGN_KIND:
        raise ValueError(f"unknown engineering design kind: {design_kind}")
    elif runs is None:
        runs = 20
    if runs <= 0:
        raise ValueError("engineering runs must be positive")
    if family not in CONFIRMATORY_FAMILIES:
        raise ValueError(f"unknown confirmatory family: {family}")
    if method not in METHODS:
        raise ValueError(f"unknown method: {method}")
    if not 0 <= layout_index < 12:
        raise ValueError("layout index must be in [0, 12)")
    layout_id = f"{family}-{layout_index:02d}"
    world_seed = _seed(master_seed, design_kind, family, layout_index, "world")
    trials = tuple(TrialSpec(
        trial_id=f"{design_kind}-{layout_id}-r{run:02d}", family=family,
        layout_id=layout_id, replicate=run, method=method, method_order=0,
        world_seed=world_seed,
        sensing_seed=_seed(master_seed, design_kind, family, layout_index, run, "sensing"),
        friction_seed=_seed(master_seed, design_kind, family, layout_index, run, "friction"),
        fault_seed=_seed(master_seed, design_kind, family, layout_index, run, "fault"),
    ) for run in range(runs))
    return StudyDesign(1, design_kind, 1, runs, master_seed, (method,), (family,), trials)

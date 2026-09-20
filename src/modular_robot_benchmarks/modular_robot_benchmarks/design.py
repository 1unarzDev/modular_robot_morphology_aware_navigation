from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
from random import Random
from typing import Iterable, Sequence


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
FAULT_MATRIX_DESIGN_KIND = "engineering_fault_matrix"
# Workshop diagnostic study: one passage environment in three variants.
WORKSHOP_DESIGN_KIND = "workshop_diagnostic"
WORKSHOP_VARIANTS = ("workshop_blocked_a", "workshop_blocked_b", "workshop_neutral")
WORKSHOP_METHODS = ("route_first_adaptation", "geometry_coupled", "feasibility_coupled")


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
        if design.design_kind.startswith(("engineering_", "workshop_")):
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


def generate_workshop_design(
    replicates: int = 2,
    world_seed_key: int = 0,
    master_seed: int = 20260914,
) -> StudyDesign:
    """Variants x methods x paired disturbance seeds on one base environment.

    Every trial shares the world seed; each replicate's plant seeds are shared
    across variants and methods, so comparisons are paired.
    """
    world_seed = _seed(master_seed, WORKSHOP_DESIGN_KIND, world_seed_key, "world")
    trials = []
    for replicate in range(replicates):
        sensing_seed = _seed(master_seed, WORKSHOP_DESIGN_KIND, replicate, "sensing")
        friction_seed = _seed(master_seed, WORKSHOP_DESIGN_KIND, replicate, "friction")
        fault_seed = _seed(master_seed, WORKSHOP_DESIGN_KIND, replicate, "fault")
        for variant in WORKSHOP_VARIANTS:
            order = list(WORKSHOP_METHODS)
            Random(_seed(master_seed, variant, replicate, "order")).shuffle(order)
            for order_index, method in enumerate(order):
                trials.append(TrialSpec(
                    trial_id=f"{variant}-00-r{replicate}-{method}", family=variant,
                    layout_id=f"{variant}-00", replicate=replicate, method=method,
                    method_order=order_index, world_seed=world_seed,
                    sensing_seed=sensing_seed, friction_seed=friction_seed,
                    fault_seed=fault_seed,
                ))
    return StudyDesign(1, WORKSHOP_DESIGN_KIND, 1, replicates, master_seed,
                       WORKSHOP_METHODS, WORKSHOP_VARIANTS, tuple(trials))


def fault_case_for_run(run: int) -> FaultCase:
    """Map a fault-matrix run index onto its declared case.

    Runs cycle through `FAULT_MATRIX` so that realization ``k`` of case ``c``
    is run ``k * len(FAULT_MATRIX) + c``. With one realization this is the
    identity, which keeps single-cell designs unchanged.
    """
    return FAULT_MATRIX[run % len(FAULT_MATRIX)]


def generate_fault_matrix_campaign(
    family: str,
    layout_indices: Sequence[int],
    method: str,
    realizations: int = 1,
    master_seed: int = 20260912,
) -> StudyDesign:
    """Repeat the declared fault matrix across layouts and fault realizations.

    Each (layout, realization, case) cell is one single-method run with its own
    plant and fault seeds. Roadmap Gate 0 step 2 requires the matrix to hold
    across layouts, not only on the single engineering layout it first passed.
    """
    design_kind = FAULT_MATRIX_DESIGN_KIND
    if family not in CONFIRMATORY_FAMILIES:
        raise ValueError(f"unknown confirmatory family: {family}")
    if method not in METHODS:
        raise ValueError(f"unknown method: {method}")
    layout_indices = tuple(layout_indices)
    if not layout_indices:
        raise ValueError("a fault-matrix campaign needs at least one layout")
    if len(set(layout_indices)) != len(layout_indices):
        raise ValueError("layout indices must be distinct")
    if any(not 0 <= index < 12 for index in layout_indices):
        raise ValueError("layout index must be in [0, 12)")
    if realizations < 1:
        raise ValueError("a fault-matrix campaign needs at least one realization")
    runs = len(FAULT_MATRIX) * realizations
    trials: list[TrialSpec] = []
    for layout_index in layout_indices:
        layout_id = f"{family}-{layout_index:02d}"
        world_seed = _seed(master_seed, design_kind, family, layout_index, "world")
        trials.extend(TrialSpec(
            trial_id=f"{design_kind}-{layout_id}-r{run:02d}", family=family,
            layout_id=layout_id, replicate=run, method=method, method_order=0,
            world_seed=world_seed,
            sensing_seed=_seed(master_seed, design_kind, family, layout_index, run, "sensing"),
            friction_seed=_seed(master_seed, design_kind, family, layout_index, run, "friction"),
            fault_seed=_seed(master_seed, design_kind, family, layout_index, run, "fault"),
        ) for run in range(runs))
    return StudyDesign(1, design_kind, len(layout_indices), runs, master_seed,
                       (method,), (family,), tuple(trials))


def generate_fault_matrix_design(
    family: str,
    layout_index: int,
    method: str,
    master_seed: int = 20260912,
) -> StudyDesign:
    """One single-method run per declared fault, with independent plant seeds.

    This is the single-layout, single-realization campaign; its design hash is
    unchanged by the cross-layout generalization.
    """
    return generate_fault_matrix_campaign(
        family, (layout_index,), method, 1, master_seed)

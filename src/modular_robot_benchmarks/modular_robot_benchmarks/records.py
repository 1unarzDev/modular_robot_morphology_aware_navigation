from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import os
from pathlib import Path
from typing import Any, Iterable

from .design import TrialSpec


FAILURE_REASONS = frozenset({
    "completed", "planning_failure", "process_crash", "stale_topic",
    "localization_lost", "collision", "unsafe_topology", "docking_failure",
    "controller_failure", "timeout", "cancelled", "infrastructure_failure",
})


@dataclass(frozen=True)
class TrialManifest:
    commit: str
    configuration_hash: str
    design_hash: str
    software_versions: dict[str, str]
    parameters: dict[str, Any]


@dataclass(frozen=True)
class TrialRecord:
    schema_version: int
    spec: TrialSpec
    manifest: TrialManifest
    terminal_status: str
    completed: bool
    simulated_duration_s: float
    wall_duration_s: float
    deadline_s: float = 300.0
    planning_latency_s: float = 0.0
    expanded_states: int = 0
    mechanical_work_j: float = 0.0
    collision_count: int = 0
    minimum_clearance_m: float | None = None
    reconfiguration_attempts: int = 0
    reconfiguration_failures: int = 0
    recovery_actions: int = 0
    localization_error_m: list[float] = field(default_factory=list)
    covariance_trace: list[float] = field(default_factory=list)
    predicted_transition_probabilities: list[float] = field(default_factory=list)
    observed_transition_outcomes: list[int] = field(default_factory=list)
    planned_route_signature: str = ""
    planned_transition_sites: list[dict[str, Any]] = field(default_factory=list)
    transition_edge_decisions: list[dict[str, Any]] = field(default_factory=list)
    topology_history: list[dict[str, Any]] = field(default_factory=list)
    execution_state_history: list[dict[str, Any]] = field(default_factory=list)
    final_execution_state: str = "STOPPED"
    unrecovered_fault: bool = False
    map_revision: int = 0
    topology_revision: int = 0
    sensing_revision: int = 0
    notes: list[str] = field(default_factory=list)

    def validate(self) -> None:
        if self.schema_version != 1:
            raise ValueError("unsupported trial record schema")
        if self.terminal_status not in FAILURE_REASONS:
            raise ValueError(f"unknown terminal status: {self.terminal_status}")
        if self.deadline_s <= 0 or self.simulated_duration_s < 0 or self.wall_duration_s < 0:
            raise ValueError("durations must be nonnegative and deadline positive")
        if self.completed != (self.terminal_status == "completed"):
            raise ValueError("completed flag and terminal status disagree")
        if self.completed and (self.collision_count != 0 or self.unrecovered_fault
                               or self.final_execution_state != "READY"):
            raise ValueError("completion requires collision-free READY topology with no fault")
        if len(self.predicted_transition_probabilities) != len(self.observed_transition_outcomes):
            raise ValueError("transition predictions and outcomes are not paired")
        if any(not 0.0 <= value <= 1.0 for value in self.predicted_transition_probabilities):
            raise ValueError("transition probabilities must be in [0, 1]")
        if any(value not in (0, 1) for value in self.observed_transition_outcomes):
            raise ValueError("transition outcomes must be binary")
        for decision in self.transition_edge_decisions:
            required = {"transition_id", "feasible", "reasons"}
            if not required <= set(decision):
                raise ValueError("transition edge decision is missing audit fields")
            if not isinstance(decision["feasible"], bool):
                raise ValueError("transition edge feasibility must be boolean")
            if not isinstance(decision["reasons"], list):
                raise ValueError("transition edge reasons must be a list")

    @property
    def deadline_penalized_time_s(self) -> float:
        return min(self.simulated_duration_s, self.deadline_s) if self.completed else self.deadline_s

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        return asdict(self)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "TrialRecord":
        value = dict(value)
        value["spec"] = TrialSpec(**value["spec"])
        value["manifest"] = TrialManifest(**value["manifest"])
        record = cls(**value)
        record.validate()
        return record


class TrialStore:
    """Append-only terminal record store used to resume interrupted batches."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)

    def path_for(self, trial_id: str) -> Path:
        allowed = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_"
        if not trial_id or any(character not in allowed for character in trial_id):
            raise ValueError("unsafe trial id")
        return self.root / f"{trial_id}.json"

    def contains(self, trial_id: str) -> bool:
        return self.path_for(trial_id).is_file()

    def write_terminal(self, record: TrialRecord) -> Path:
        record.validate()
        self.root.mkdir(parents=True, exist_ok=True)
        path = self.path_for(record.spec.trial_id)
        payload = json.dumps(record.to_dict(), sort_keys=True, indent=2) + "\n"
        try:
            descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        except FileExistsError:
            if path.read_text(encoding="utf-8") != payload:
                raise FileExistsError(f"refusing to overwrite immutable record: {path}")
            return path
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        return path

    def load_all(self) -> list[TrialRecord]:
        records = []
        for path in sorted(self.root.glob("*.json")):
            with path.open(encoding="utf-8") as stream:
                records.append(TrialRecord.from_dict(json.load(stream)))
        return records

    def pending(self, specs: Iterable[TrialSpec]) -> list[TrialSpec]:
        return [spec for spec in specs if not self.contains(spec.trial_id)]

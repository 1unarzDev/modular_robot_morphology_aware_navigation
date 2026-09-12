from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Mapping


class ExecutionState(IntEnum):
    READY = 0
    TRANSITIONING = 1
    RECOVERY_REQUIRED = 2
    STOPPED = 3


@dataclass(frozen=True)
class ObservedConnection:
    pod_id: str
    parent_port: str


class TopologyStateMachine:
    """Authoritative observed topology and transition interlock."""

    def __init__(self, catalog: Mapping, initial_morphology: str) -> None:
        self.catalog = catalog
        self.morphology_id = initial_morphology
        self.execution_state = ExecutionState.READY
        self.topology_revision = 1
        self.sensing_revision = 0
        self.active_transition: str | None = None
        self.connections = self._nominal_connections(initial_morphology)

    def _nominal_connections(self, morphology: str) -> dict[str, ObservedConnection]:
        return {pod: ObservedConnection(pod, f"{morphology}/{pod}")
                for pod in self.catalog["morphologies"][morphology]["pods"]}

    def begin(self, transition: Mapping, expected_source: str, expected_revision: int) -> None:
        if self.execution_state != ExecutionState.READY:
            raise ValueError(f"system is {self.execution_state.name}")
        if expected_source != self.morphology_id or transition["from"] != self.morphology_id:
            raise ValueError(f"stale source morphology; observed {self.morphology_id}")
        if expected_revision != self.topology_revision:
            raise ValueError(f"stale topology revision; observed {self.topology_revision}")
        self.execution_state = ExecutionState.TRANSITIONING
        self.active_transition = str(transition["id"])

    def observe(self, pod_id: str, parent_port: str | None, latched: bool) -> None:
        previous = self.connections.get(pod_id)
        current = ObservedConnection(pod_id, parent_port) if latched and parent_port else None
        if previous == current:
            return
        if current is None:
            self.connections.pop(pod_id, None)
        else:
            self.connections[pod_id] = current
        self.topology_revision += 1

    def fail(self) -> None:
        if self.execution_state == ExecutionState.TRANSITIONING:
            self.execution_state = ExecutionState.RECOVERY_REQUIRED
            self.active_transition = None

    def commit(self, transition: Mapping) -> None:
        if self.execution_state != ExecutionState.TRANSITIONING:
            raise ValueError("no active transition")
        if self.active_transition != transition["id"]:
            raise ValueError("transition does not match active request")
        expected = self._nominal_connections(str(transition["to"]))
        if self.connections != expected:
            raise ValueError("observed topology does not match target morphology")
        self.morphology_id = str(transition["to"])
        self.active_transition = None
        self.execution_state = ExecutionState.READY

    def reconcile(self) -> str:
        for morphology in self.catalog["morphologies"]:
            if self.connections == self._nominal_connections(morphology):
                self.morphology_id = morphology
                self.active_transition = None
                self.execution_state = ExecutionState.READY
                return morphology
        self.execution_state = ExecutionState.RECOVERY_REQUIRED
        raise ValueError("observed topology is not a known safe morphology")

    @property
    def drive_allowed(self) -> bool:
        return self.execution_state == ExecutionState.READY

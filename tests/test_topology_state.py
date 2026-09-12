from pathlib import Path

import pytest
import yaml

from morphology_manager.state import ExecutionState, TopologyStateMachine


CATALOG = Path(__file__).parents[1] / "src/modular_robot_description/config/morphologies.yaml"


def machine():
    with CATALOG.open(encoding="utf-8") as stream:
        catalog = yaml.safe_load(stream)
    return TopologyStateMachine(catalog, "compact_diff"), catalog


def test_stale_and_concurrent_transitions_are_rejected():
    state, catalog = machine()
    transition = next(t for t in catalog["transitions"] if t["id"] == "compact_to_narrow")
    with pytest.raises(ValueError, match="stale topology"):
        state.begin(transition, "compact_diff", 0)
    state.begin(transition, "compact_diff", state.topology_revision)
    with pytest.raises(ValueError, match="TRANSITIONING"):
        state.begin(transition, "compact_diff", state.topology_revision)


@pytest.mark.parametrize("failed_stage", range(7))
def test_every_partial_failure_preserves_observed_topology_and_inhibits_drive(failed_stage):
    state, catalog = machine()
    transition = next(t for t in catalog["transitions"] if t["id"] == "compact_to_narrow")
    state.begin(transition, "compact_diff", state.topology_revision)
    for index, pod in enumerate(transition["moved_pods"]):
        if index >= failed_stage:
            break
        state.observe(pod, None, False)
        state.observe(pod, f"narrow_tandem/{pod}", True)
    state.fail()
    assert state.execution_state == ExecutionState.RECOVERY_REQUIRED
    assert not state.drive_allowed
    assert len(state.connections) == 6
    if failed_stage:
        assert any(c.parent_port.startswith("narrow_tandem/") for c in state.connections.values())


def test_recovery_requires_exact_known_topology():
    state, catalog = machine()
    transition = next(t for t in catalog["transitions"] if t["id"] == "compact_to_narrow")
    state.begin(transition, "compact_diff", state.topology_revision)
    state.observe("pod_0", None, False); state.fail()
    with pytest.raises(ValueError, match="not a known safe"):
        state.reconcile()
    state.observe("pod_0", "compact_diff/pod_0", True)
    assert state.reconcile() == "compact_diff"
    assert state.execution_state == ExecutionState.READY and state.drive_allowed


def test_commit_requires_complete_observed_target_topology():
    state, catalog = machine()
    transition = next(t for t in catalog["transitions"] if t["id"] == "compact_to_narrow")
    state.begin(transition, "compact_diff", state.topology_revision)
    for pod in transition["moved_pods"]:
        state.observe(pod, None, False)
        state.observe(pod, f"narrow_tandem/{pod}", True)
    revision = state.topology_revision
    state.commit(transition)
    assert state.morphology_id == "narrow_tandem"
    assert state.topology_revision == revision
    assert state.drive_allowed

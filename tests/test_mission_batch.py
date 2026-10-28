from pathlib import Path

from modular_robot_benchmarks.mission_batch import (
    classify_terminal, configuration_hash, retryable_infrastructure_failure,
)


def test_configuration_hash_covers_files_and_exact_parameters(tmp_path):
    first = tmp_path / "a"; first.write_text("one")
    second = tmp_path / "b"; second.write_text("two")
    baseline = configuration_hash([first, second], {"deadline": 300})
    assert baseline == configuration_hash([second, first], {"deadline": 300})
    assert baseline != configuration_hash([first, second], {"deadline": 301})
    second.write_text("changed")
    assert baseline != configuration_hash([first, second], {"deadline": 300})


def test_execution_failure_is_not_hidden_by_failed_replan():
    message = "execution failed; replanning failed: fixed route cannot be adapted"
    assert classify_terminal(False, message, False) == "controller_failure"


def test_reconfiguration_failure_and_unsafe_topology_have_distinct_labels():
    assert classify_terminal(
        False, "reconfiguration failed; docking evidence rejected", False
    ) == "docking_failure"
    assert classify_terminal(
        False, "reconfiguration failed; observed topology requires recovery", False
    ) == "unsafe_topology"
    assert classify_terminal(
        False, "reconfiguration committed; costmaps did not acknowledge footprint", False
    ) == "infrastructure_failure"
    assert classify_terminal(
        False, "reconfiguration committed; post-transition assembled motion "
        "qualification failed; drive inhibited", False
    ) == "motion_qualification_failure"


def test_only_live_launch_infrastructure_failures_are_retried():
    assert retryable_infrastructure_failure("infrastructure_failure", None)
    assert not retryable_infrastructure_failure("process_crash", 1)
    assert not retryable_infrastructure_failure("infrastructure_failure", 1)
    assert not retryable_infrastructure_failure("docking_failure", None)


def _loss(onset, ongoing=False):
    return {"onset_s": onset, "end_s": onset + 1.0, "peak_error_m": 0.5,
            "samples": 3, "ongoing": ongoing}


def test_evaluator_contact_on_a_completed_mission_is_a_collision():
    from modular_robot_benchmarks.mission_batch import evaluator_outcome

    assert evaluator_outcome("completed", True, [{"time_s": 4.0}], [])[:2] == (
        "collision", False)
    assert evaluator_outcome("completed", True, [], []) == ("completed", True, "")
    # A failed mission with contact keeps its status, as before.
    assert evaluator_outcome("controller_failure", False, [{"time_s": 4.0}], [])[:2] == (
        "controller_failure", False)


def test_completed_mission_still_lost_at_the_end_is_localization_loss():
    from modular_robot_benchmarks.mission_batch import evaluator_outcome

    # A recovered episode does not undo a completion.
    assert evaluator_outcome("completed", True, [], [_loss(10.0)])[:2] == ("completed", True)
    assert evaluator_outcome("completed", True, [], [_loss(10.0, ongoing=True)])[:2] == (
        "localization_lost", False)
    # Both apply: the earlier event names the outcome.
    assert evaluator_outcome(
        "completed", True, [{"time_s": 5.0}], [_loss(10.0, ongoing=True)])[0] == "collision"
    assert evaluator_outcome(
        "completed", True, [{"time_s": 15.0}], [_loss(10.0, ongoing=True)])[0] == (
        "localization_lost")


def test_loss_before_any_contact_names_a_failed_robot_outcome():
    from modular_robot_benchmarks.mission_batch import evaluator_outcome

    status, completed, note = evaluator_outcome("timeout", False, [], [_loss(20.0)])
    assert (status, completed) == ("localization_lost", False)
    assert "before the timeout" in note
    assert evaluator_outcome(
        "controller_failure", False, [{"time_s": 25.0}], [_loss(20.0)])[0] == (
        "localization_lost")
    assert evaluator_outcome(
        "controller_failure", False, [{"time_s": 15.0}], [_loss(20.0)])[0] == (
        "controller_failure")


def test_infrastructure_and_planning_statuses_are_never_reattributed():
    from modular_robot_benchmarks.mission_batch import evaluator_outcome

    for status in ("infrastructure_failure", "process_crash", "planning_failure"):
        assert evaluator_outcome(status, False, [{"time_s": 1.0}], [_loss(0.5, True)]) == (
            status, False, "")

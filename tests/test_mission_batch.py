from pathlib import Path

from modular_robot_benchmarks.mission_batch import classify_terminal, configuration_hash


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

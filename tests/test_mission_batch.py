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


def test_unsettled_planner_revisions_are_infrastructure():
    assert classify_terminal(
        False, "planner revisions did not settle: sensing signature changed "
        "before planning", False
    ) == "infrastructure_failure"


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
    # Contact names a failed robot outcome too: it is the earliest cause.
    status, completed, note = evaluator_outcome(
        "controller_failure", False, [{"time_s": 4.0}], [])
    assert (status, completed) == ("collision", False)
    assert "before the controller_failure" in note


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
        "controller_failure", False, [{"time_s": 15.0}], [_loss(20.0)])[0] == "collision"
    # A failed mission counts a recovered episode; a completed one does not.
    assert evaluator_outcome("docking_failure", False, [], [_loss(5.0)])[0] == (
        "localization_lost")
    assert evaluator_outcome("docking_failure", False, [], []) == (
        "docking_failure", False, "")


def test_infrastructure_and_planning_statuses_are_never_reattributed():
    from modular_robot_benchmarks.mission_batch import evaluator_outcome

    for status in ("infrastructure_failure", "process_crash", "planning_failure"):
        assert evaluator_outcome(status, False, [{"time_s": 1.0}], [_loss(0.5, True)]) == (
            status, False, "")


def test_a_clock_jump_during_the_mission_invalidates_it():
    from modular_robot_benchmarks.mission_batch import (
        evaluator_outcome, mission_clock_discontinuity)

    before = {"from_s": 2.0, "to_s": 40.0, "jump_s": 38.0, "wall_gap_s": 70.0,
              "during_mission": False}
    during = {"from_s": 60.0, "to_s": 180.0, "jump_s": 120.0, "wall_gap_s": 230.0,
              "during_mission": True}
    assert mission_clock_discontinuity({}) == ""
    assert mission_clock_discontinuity({"clock_discontinuities": [before]}) == ""
    reason = mission_clock_discontinuity({"clock_discontinuities": [before, during]})
    assert "jumped 120.0 s" in reason
    # And once infrastructure, evaluator evidence never re-attributes it.
    assert evaluator_outcome("infrastructure_failure", False, [{"time_s": 70.0}], []) == (
        "infrastructure_failure", False, "")


def test_the_batch_checks_the_clock_before_scoring_the_robot():
    from pathlib import Path

    batch = (Path(__file__).resolve().parents[1] / "src/modular_robot_benchmarks/"
             "modular_robot_benchmarks/mission_batch.py").read_text(encoding="utf-8")
    loop = batch.split("teardown_wall_s = time.monotonic() - teardown_start", 1)[1]
    assert loop.index("mission_clock_discontinuity(") < loop.index("evaluator_outcome(")
    trial = (Path(__file__).resolve().parents[1] / "src/modular_robot_benchmarks/"
             "modular_robot_benchmarks/mission_trial.py").read_text(encoding="utf-8")
    assert '"clock_discontinuities": list(node.clock_discontinuities)' in trial


def test_a_planner_that_finds_no_plan_is_a_planning_failure_not_a_docking_failure():
    # Seen in the pilot: route-first's NoPathError names "feasible transitions".
    for message in ("fixed spatial route cannot be adapted with feasible transitions",
                    "no collision-free hybrid plan",
                    "no morphology-agnostic spatial route"):
        assert classify_terminal(False, message, False) == "planning_failure"
    # An execution failure whose replan then found no plan stays an execution failure.
    assert classify_terminal(
        False, "execution failed; replanning failed: no collision-free hybrid plan", False
    ) == "controller_failure"

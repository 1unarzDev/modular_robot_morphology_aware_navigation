"""A transport failure is not a robot outcome, and stale segments cause them.

Cover for the single failure in the 2026-09-30 round trip: nine Nav2 nodes all
failed to open `fastrtps_port18186`, the trial recorded `controller_failure` --
a robot outcome -- and the robot had in fact reached its site inside tolerance
and never moved a pod.
"""

from pathlib import Path

from modular_robot_benchmarks.mission_batch import (
    clear_unused_dds_segments, retryable_infrastructure_failure,
    transport_initialization_failures, unused_dds_segments,
)


REAL_FAILURE = (
    "[hybrid_navigator-13] 2026-10-01 01:17:29.270 [RTPS_TRANSPORT_SHM Error] "
    "[Failed init_port fastrtps_port18186: open_and_lock_file failed "
    "-> Function init_port]"
)


def test_the_real_transport_failure_is_detected_with_its_port(tmp_path):
    log = tmp_path / "launch.log"
    log.write_text(REAL_FAILURE, encoding="utf-8")
    assert transport_initialization_failures(log) == {"fastrtps_port18186"}


def test_a_healthy_launch_log_reports_nothing(tmp_path):
    log = tmp_path / "launch.log"
    log.write_text(
        "[controller_server-14] [INFO] Reached the goal!" + chr(10)
        + "[hybrid_navigator-13] [WARN] execution failed; replanning attempt 1"
        + chr(10) + "[bt_navigator-19] [INFO] Creating bond to lifecycle manager.",
        encoding="utf-8")
    assert transport_initialization_failures(log) == set()


def test_a_missing_log_is_not_an_error(tmp_path):
    assert transport_initialization_failures(tmp_path / "absent.log") == set()


def test_a_transport_failure_without_a_named_port_still_reports(tmp_path):
    log = tmp_path / "launch.log"
    log.write_text("[planner_server-16] open_and_lock_file failed", encoding="utf-8")
    assert transport_initialization_failures(log) == {"unnamed_port"}


def test_a_transport_failure_is_retryable_once_classified():
    # The launch process stays alive when only the transport failed, which is
    # what makes the trial eligible for the bounded retry.
    assert retryable_infrastructure_failure("infrastructure_failure", None) is True
    assert retryable_infrastructure_failure("controller_failure", None) is False


def test_the_trial_loop_reclassifies_only_unfinished_trials():
    source = (Path(__file__).parents[1]
              / "src/modular_robot_benchmarks/modular_robot_benchmarks/mission_batch.py"
              ).read_text()
    assert "if failed_ports and not observation.completed:" in source
    # The reclassification has to precede the retry decision, or the trial is
    # recorded as a robot outcome and never retried.
    assert source.index("failed_ports = transport_initialization_failures") <         source.index("retryable_infrastructure_failure(" + chr(10))


def test_a_segment_a_live_process_holds_open_is_never_cleared(tmp_path):
    # This test process is the live participant: a segment it holds open must
    # survive, because deleting one underneath a participant is the opposite
    # fault and has already cost a campaign here.
    held = tmp_path / "fastrtps_port18186"
    held.write_bytes(b"segment")
    free = tmp_path / "fastrtps_port19999"
    free.write_bytes(b"segment")
    with held.open("rb"):
        assert unused_dds_segments(tmp_path) == [free]
        cleared = clear_unused_dds_segments(tmp_path)
    assert cleared == ["fastrtps_port19999"]
    assert held.exists()
    assert not free.exists()


def test_a_directory_without_segments_is_a_no_op(tmp_path):
    assert unused_dds_segments(tmp_path) == []
    assert clear_unused_dds_segments(tmp_path) == []


def test_only_fastrtps_segments_are_considered(tmp_path):
    unrelated = tmp_path / "sem.something_else"
    unrelated.write_bytes(b"x")
    assert unused_dds_segments(tmp_path) == []
    assert unrelated.exists()

from math import pi

from modular_robot_benchmarks.motion_metrics import summarize


def _stage(name, start, end):
    rows = []
    for time_s, (x, y, yaw) in enumerate((start, end)):
        rows.append({
            "stage": name, "time_s": float(time_s), "x": x, "y": y,
            "yaw": yaw, "pod_commands": {"pod_0": 0.1},
        })
    return rows


def test_qualification_uses_body_frame_and_accepts_bounded_yaw_translation():
    samples = []
    samples += _stage("positive_yaw", (0.0, 0.0, 0.0), (0.02, 0.0, 0.5))
    samples += _stage("negative_yaw", (0.0, 0.0, 0.5), (0.01, 0.0, 0.0))
    # Starting at pi/2 means forward travel is positive world y.
    samples += _stage("straight", (0.0, 0.0, pi / 2), (0.0, 0.5, pi / 2))
    result = summarize(samples)
    assert result["rep_103_signs_pass"]
    assert result["straight_motion_pass"]
    assert result["yaw_translation_pass"]
    assert result["qualification_pass"]


def test_qualification_rejects_translation_during_nominal_yaw():
    samples = []
    samples += _stage("positive_yaw", (0.0, 0.0, 0.0), (0.4, 0.2, 0.15))
    samples += _stage("negative_yaw", (0.0, 0.0, 0.15), (0.0, 0.0, -0.15))
    samples += _stage("straight", (0.0, 0.0, 0.0), (0.5, 0.0, 0.0))
    result = summarize(samples)
    assert not result["yaw_translation_pass"]
    assert not result["qualification_pass"]

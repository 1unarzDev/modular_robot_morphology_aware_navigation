from math import pi

from modular_robot_benchmarks.rigidity import qualify_pod_rigidity


def _samples(drift=0.0, omit=()):
    values = []
    for time_s in (1.0, 1.5, 2.0):
        yaw = 0.2 * time_s
        for index in range(6):
            pod = f"pod_{index}"
            if pod in omit:
                continue
            # A rigid array rotating in the world frame.
            local_x, local_y = 0.2 * index, -0.1 * (index % 2)
            from math import cos, sin
            x = cos(yaw) * local_x - sin(yaw) * local_y
            y = sin(yaw) * local_x + cos(yaw) * local_y
            values.append({
                "time_s": time_s + index * 0.002, "pod": pod,
                "world_x": x + (drift if pod == "pod_4" and time_s == 2.0 else 0.0),
                "world_y": y, "world_z": 0.0, "world_roll": 0.0,
                "world_pitch": 0.0, "world_yaw": yaw,
            })
    return values


def test_rigidity_accepts_constant_relative_se3_during_world_motion():
    result = qualify_pod_rigidity(_samples(), 0.9, 2.1)
    assert result["passed"]
    assert all(value["synchronized_samples"] == 3 for value in result["pods"].values())


def test_rigidity_rejects_one_pod_drift_and_missing_evidence():
    drifted = qualify_pod_rigidity(_samples(drift=0.03), 0.9, 2.1)
    assert not drifted["passed"] and not drifted["pods"]["pod_4"]["passed"]
    missing = qualify_pod_rigidity(_samples(omit=("pod_5",)), 0.9, 2.1)
    assert not missing["passed"] and missing["pods"]["pod_5"]["synchronized_samples"] == 0

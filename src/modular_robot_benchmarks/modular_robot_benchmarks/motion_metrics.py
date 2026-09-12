from math import atan2, cos, sin


def summarize(samples: list[dict]) -> dict:
    summaries = {}
    for stage in ("positive_yaw", "negative_yaw", "straight"):
        values = [value for value in samples if value["stage"] == stage]
        if not values:
            raise RuntimeError(f"no samples for {stage}")
        dx = values[-1]["x"] - values[0]["x"]
        dy = values[-1]["y"] - values[0]["y"]
        start_yaw = values[0]["yaw"]
        forward = cos(start_yaw) * dx + sin(start_yaw) * dy
        lateral = -sin(start_yaw) * dx + cos(start_yaw) * dy
        summaries[stage] = {
            "duration_s": values[-1]["time_s"] - values[0]["time_s"],
            "delta_x": values[-1]["x"] - values[0]["x"],
            "delta_y": values[-1]["y"] - values[0]["y"],
            "delta_yaw": _angle_difference(values[-1]["yaw"], values[0]["yaw"]),
            "body_forward_displacement_m": forward,
            "body_lateral_displacement_m": lateral,
            "translation_m": (dx * dx + dy * dy) ** 0.5,
            "peak_absolute_pod_commands": {
                pod: max(abs(value["pod_commands"][pod]) for value in values)
                for pod in values[0]["pod_commands"]
            },
        }
    positive_ok = summaries["positive_yaw"]["delta_yaw"] > 0.1
    negative_ok = summaries["negative_yaw"]["delta_yaw"] < -0.1
    straight_ok = (
        summaries["straight"]["body_forward_displacement_m"] > 0.1
        and abs(summaries["straight"]["body_lateral_displacement_m"]) < 0.10
        and abs(summaries["straight"]["delta_yaw"]) < 0.20)
    yaw_translation_ok = all(
        summaries[stage]["translation_m"] < 0.08
        for stage in ("positive_yaw", "negative_yaw"))
    return {
        "schema_version": 1, "purpose": "engineering_qualification",
        "rep_103_signs_pass": positive_ok and negative_ok,
        "straight_motion_pass": straight_ok,
        "yaw_translation_pass": yaw_translation_ok,
        "qualification_pass": (
            positive_ok and negative_ok and straight_ok and yaw_translation_ok),
        "stages": summaries, "sample_count": len(samples), "samples": samples,
    }


def _angle_difference(target: float, source: float) -> float:
    return atan2(sin(target - source), cos(target - source))

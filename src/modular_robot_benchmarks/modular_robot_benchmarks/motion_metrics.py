from math import atan2, cos, degrees, hypot, sin
from statistics import median


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


def heading_local_rates(samples: list[dict], stages: tuple[str, ...],
                        quantity: str, spin_up_s: float = 0.5,
                        window_s: float = 0.2) -> list[tuple[float, float]]:
    """Local motion rate against the heading the robot actually held.

    A stage total divided by its duration is not a per-heading measurement once
    authority varies with heading: a 4 s rotation stage turns the body up to
    1 rad, so the total integrates authority over the arc traversed, and the
    stages compound because each starts where the last ended. That skews totals
    toward whichever turn direction leads into a favourable heading, which is
    how a 180-degree-periodic effect first read as asymmetric
    (``studies/engineering/heading_traction_curve.json``).

    Returns ``(heading_degrees, rate / commanded_rate)`` per finite-difference
    window, taking the heading at the middle of the window. ``spin_up_s`` of
    each stage is discarded because the command ramp is not the steady rate;
    measure the ramp separately rather than averaging it in.
    """
    points: list[tuple[float, float]] = []
    for stage in stages:
        for block in _stage_blocks(samples, stage):
            points.extend(_windows(block, quantity, spin_up_s, window_s))
    return points


def bin_by_heading(points: list[tuple[float, float]], width_deg: float = 10.0,
                   fold_deg: float | None = 180.0) -> dict:
    """Bin ``heading_local_rates`` output, folding if a period is claimed.

    Friction axes are axes, so an unattached-``fdir1`` effect has a period of
    180 degrees. ``fold_deg`` of ``None`` leaves the bins unfolded, which is how
    the period is measured rather than assumed.
    """
    table: dict[int, list[float]] = {}
    for heading, fraction in points:
        folded = heading % fold_deg if fold_deg else heading
        table.setdefault(int(folded // width_deg) * int(width_deg), []).append(
            fraction)
    return {
        str(key): {
            "n": len(values),
            "median_fraction_of_command": median(values),
            "min": min(values), "max": max(values),
        }
        for key, values in sorted(table.items())
    }


def time_to_reach(samples: list[dict], stage: str, target: float = 0.85,
                  window_s: float = 0.2) -> dict:
    """When a translation stage first reaches ``target`` of commanded speed.

    A Coulomb friction limit caps acceleration rather than constant-velocity
    rolling, so a heading that costs travel costs it in the ramp: measured
    speed after the ramp is the same at every heading while the ramp itself
    grows by more than an order of magnitude. A stage shorter than the ramp is
    therefore penalized proportionally more, which is why a stage total cannot
    be rescaled from one stage duration to another.
    """
    blocks = list(_stage_blocks(samples, stage))
    if not blocks:
        raise RuntimeError(f"no samples for {stage}")
    block = blocks[0]
    command = abs(block[0]["linear_command"])
    start = block[0]["time_s"]
    reached = None
    settled = []
    for elapsed, speed in _speeds(block, window_s):
        if command > 0 and speed >= target * command and reached is None:
            reached = elapsed
        if reached is not None:
            settled.append(speed)
    total = hypot(block[-1]["x"] - block[0]["x"], block[-1]["y"] - block[0]["y"])
    duration = block[-1]["time_s"] - start
    return {
        "time_to_target_s": reached,
        "median_speed_after_target": (
            median(settled) / command if settled and command else None),
        "mean_speed_fraction_of_command": (
            total / duration / command if duration and command else None),
        "total_translation_m": total,
    }


def _stage_blocks(samples: list[dict], stage: str):
    block: list[dict] = []
    for value in samples:
        if value["stage"] == stage:
            block.append(value)
        elif block:
            yield block
            block = []
    if block:
        yield block


def _unwrapped(block: list[dict]) -> list[float]:
    values = [block[0]["yaw"]]
    for value in block[1:]:
        values.append(values[-1] + _angle_difference(value["yaw"], values[-1]))
    return values


def _pairs(block: list[dict], spin_up_s: float, window_s: float):
    times = [value["time_s"] for value in block]
    for index in range(len(block)):
        if times[index] - times[0] < spin_up_s:
            continue
        end = index
        while end < len(block) - 1 and times[end] - times[index] < window_s:
            end += 1
        if times[end] - times[index] < 0.75 * window_s:
            continue
        yield index, end


def _windows(block: list[dict], quantity: str, spin_up_s: float,
             window_s: float):
    yaws = _unwrapped(block)
    for index, end in _pairs(block, spin_up_s, window_s):
        span = block[end]["time_s"] - block[index]["time_s"]
        if quantity == "yaw":
            rate = (yaws[end] - yaws[index]) / span
            command = abs(block[index]["angular_command"])
        else:
            rate = hypot(block[end]["x"] - block[index]["x"],
                         block[end]["y"] - block[index]["y"]) / span
            command = abs(block[index]["linear_command"])
        if command <= 0:
            continue
        heading = degrees(0.5 * (yaws[index] + yaws[end])) % 360.0
        yield heading, abs(rate) / command


def _speeds(block: list[dict], window_s: float):
    for index, end in _pairs(block, 0.0, window_s):
        span = block[end]["time_s"] - block[index]["time_s"]
        yield (block[index]["time_s"] - block[0]["time_s"],
               hypot(block[end]["x"] - block[index]["x"],
                     block[end]["y"] - block[index]["y"]) / span)


def _angle_difference(target: float, source: float) -> float:
    return atan2(sin(target - source), cos(target - source))

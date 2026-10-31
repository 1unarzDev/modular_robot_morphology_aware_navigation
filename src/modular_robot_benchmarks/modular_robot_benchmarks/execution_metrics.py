"""Does the robot execute the plan it was given? Measures from mission records.

Two engineering experiments of 2026-10-31 are judged by these
(`docs/current_status.md`): the narrow body's distance from its planned route
while driving, against ADR 0013's 0.10 m turning margin, and the largest single
in-place turn of the narrow body, the signature of a planned reverse the
controller executed as a turnaround (ADR 0014). Both read pod_0's 2 Hz drive
diagnostics, whose core pose is evaluator truth.
"""
import argparse
import glob
import json
import math
import os

FORWARD_MPS = 0.05
IN_PLACE_LINEAR_MPS = 0.01
IN_PLACE_ANGULAR_RADPS = 0.1
# planned_route concatenates every plan a mission made; a jump longer than any
# primitive starts the next one.
MAX_PRIMITIVE_M = 0.2


def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _samples(record):
    return sorted((sample for sample in record.get("controller_diagnostics", {}).get(
        "pod_drive_diagnostic_samples", []) if sample["pod"] == "pod_0"),
        key=lambda sample: sample["time_s"])


def _morphology_at(history, time_s):
    morphology = "compact_diff"
    for event in history:
        if event["time_s"] <= time_s:
            morphology = event["morphology"]
    return morphology


def _segment_distance(px, py, a, b):
    dx, dy = b["x"] - a["x"], b["y"] - a["y"]
    length = dx * dx + dy * dy
    t = 0.0 if length == 0 else max(0.0, min(1.0, (
        (px - a["x"]) * dx + (py - a["y"]) * dy) / length))
    return math.hypot(px - a["x"] - t * dx, py - a["y"] - t * dy)


def route_offsets(record, morphology="narrow_tandem"):
    """Distance of the true core from the nearest planned route segment, at each
    sample while in `morphology` and commanded forward."""
    route = record.get("planned_route", [])
    segments = [(a, b) for a, b in zip(route, route[1:])
                if math.hypot(b["x"] - a["x"], b["y"] - a["y"]) < MAX_PRIMITIVE_M]
    if not segments:
        return []
    history = sorted(record.get("topology_history", []), key=lambda e: e["time_s"])
    return [min(_segment_distance(s["core_world_x"], s["core_world_y"], a, b)
                for a, b in segments)
            for s in _samples(record)
            if s["body_linear_command_mps"] > FORWARD_MPS
            and _morphology_at(history, s["time_s"]) == morphology]


def in_place_turns(record, morphology="narrow_tandem"):
    """True yaw turned in each contiguous in-place turn in one direction."""
    history = sorted(record.get("topology_history", []), key=lambda e: e["time_s"])
    turns, run, sign = [], 0.0, 0
    samples = _samples(record)
    for first, second in zip(samples, samples[1:]):
        angular = first["body_angular_command_radps"]
        direction = (angular > IN_PLACE_ANGULAR_RADPS) - (angular < -IN_PLACE_ANGULAR_RADPS)
        if (direction and abs(first["body_linear_command_mps"]) < IN_PLACE_LINEAR_MPS
                and _morphology_at(history, first["time_s"]) == morphology):
            if direction != sign and run:
                turns.append(run)
                run = 0.0
            sign = direction
            run += _wrap(second["core_world_yaw"] - first["core_world_yaw"])
        elif run:
            turns.append(run)
            run, sign = 0.0, 0
    if run:
        turns.append(run)
    return turns


def summarize_record(record, morphology="narrow_tandem"):
    offsets = sorted(route_offsets(record, morphology))
    turns = in_place_turns(record, morphology)
    return {
        "trial_id": record["spec"]["trial_id"],
        "terminal_status": record["terminal_status"],
        "route_offset_samples": len(offsets),
        "route_offset_median_m": offsets[len(offsets) // 2] if offsets else None,
        "route_offset_max_m": offsets[-1] if offsets else None,
        "largest_in_place_turn_rad": max((abs(t) for t in turns), default=0.0),
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("raw", nargs="+", help="raw record directories")
    parser.add_argument("--morphology", default="narrow_tandem")
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    rows = []
    for directory in args.raw:
        for path in sorted(glob.glob(os.path.join(directory, "*.json"))):
            with open(path, encoding="utf-8") as handle:
                rows.append(summarize_record(json.load(handle), args.morphology))
    text = json.dumps({"morphology": args.morphology, "trials": rows},
                      indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            handle.write(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

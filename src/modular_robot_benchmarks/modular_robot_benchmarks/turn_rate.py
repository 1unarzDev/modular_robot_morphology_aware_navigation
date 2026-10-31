"""True versus commanded rate over in-place turns, from mission records.

This is the estimator behind the turn-rate table in ``docs/current_status.md``
(generator-3 section) and the acceptance rule of the narrow turn-rate
experiment declared there before any run: over consecutive 0.5 s evaluator
samples of ``pod_0`` held at one constant in-place command (|angular| >= 0.15
rad/s, |linear| < 0.01 m/s), the true core yaw change per second divided by the
command. The morphology is the one the topology history holds at the window
start. Quantiles index the sorted values at n//2, n//10 and 9n//10, as the
table was computed.
"""
import argparse
import collections
import glob
import json
import math
import os

MIN_ANGULAR_RADPS = 0.15
MAX_LINEAR_MPS = 0.01
WINDOW_S = (0.3, 0.7)

# Declared 2026-10-31 before either arm ran.
ACCEPT_MEDIAN = (0.8, 1.25)
ACCEPT_P10_MIN = 0.5


def _morphology_at(history, time_s):
    morphology = "compact_diff"
    for event in history:
        if event["time_s"] <= time_s:
            morphology = event["morphology"]
    return morphology


def in_place_turn_ratios(record: dict) -> list[tuple[str, float, float]]:
    """(morphology, |command|, true/command) for each constant-command window."""
    history = sorted(record.get("topology_history", []),
                     key=lambda event: event["time_s"])
    samples = sorted(
        (sample for sample in record.get("controller_diagnostics", {}).get(
            "pod_drive_diagnostic_samples", []) if sample["pod"] == "pod_0"),
        key=lambda sample: sample["time_s"])
    ratios = []
    for first, second in zip(samples, samples[1:]):
        command = first["body_angular_command_radps"]
        elapsed = second["time_s"] - first["time_s"]
        if (abs(command) < MIN_ANGULAR_RADPS
                or abs(first["body_linear_command_mps"]) >= MAX_LINEAR_MPS
                or not WINDOW_S[0] < elapsed < WINDOW_S[1]
                or abs(second["body_angular_command_radps"] - command) >= 1e-6):
            continue
        turned = (second["core_world_yaw"] - first["core_world_yaw"]
                  + math.pi) % (2 * math.pi) - math.pi
        ratios.append((_morphology_at(history, first["time_s"]),
                       round(abs(command), 2), turned / elapsed / command))
    return ratios


def quantiles(values: list[float]) -> dict:
    ordered = sorted(values)
    count = len(ordered)
    if not count:
        return {"n": 0, "median": None, "p10": None, "p90": None}
    return {"n": count, "median": ordered[count // 2],
            "p10": ordered[count // 10], "p90": ordered[9 * count // 10]}


def summarize(records: list[dict], morphology: str = "narrow_tandem") -> dict:
    by_command = collections.defaultdict(list)
    pooled = []
    for record in records:
        for name, command, ratio in in_place_turn_ratios(record):
            if name == morphology:
                by_command[command].append(ratio)
                pooled.append(ratio)
    overall = quantiles(pooled)
    accepted = (overall["n"] > 0
                and ACCEPT_MEDIAN[0] <= overall["median"] <= ACCEPT_MEDIAN[1]
                and overall["p10"] >= ACCEPT_P10_MIN)
    return {
        "morphology": morphology, "pooled": overall,
        "by_command": {f"{command:.2f}": quantiles(values)
                       for command, values in sorted(by_command.items())},
        "turn_rate_criteria_pass": accepted,
        "criteria": {"median": list(ACCEPT_MEDIAN), "p10_min": ACCEPT_P10_MIN},
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("raw", nargs="+", help="raw record directories")
    parser.add_argument("--morphology", default="narrow_tandem")
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    records = []
    for directory in args.raw:
        for path in sorted(glob.glob(os.path.join(directory, "*.json"))):
            with open(path, encoding="utf-8") as handle:
                records.append(json.load(handle))
    summary = summarize(records, args.morphology)
    summary["record_count"] = len(records)
    text = json.dumps(summary, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            handle.write(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

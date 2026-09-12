from __future__ import annotations

from collections import Counter, defaultdict
import csv
import json
from math import sqrt
from pathlib import Path
from random import Random
from statistics import fmean
from typing import Iterable

from .design import METHODS, SENSING_FAMILIES, StudyDesign
from .records import TrialRecord, TrialStore


PRIMARY_CONTRASTS = (
    ("sensing_feasibility_coupled", "geometry_coupled", None),
    ("sensing_feasibility_coupled", "feasibility_coupled", SENSING_FAMILIES),
)


def validate_records(records: list[TrialRecord], design: StudyDesign) -> None:
    by_id = Counter(record.spec.trial_id for record in records)
    duplicates = [trial_id for trial_id, count in by_id.items() if count > 1]
    if duplicates:
        raise ValueError(f"duplicate trial ids: {duplicates[:5]}")
    expected = {spec.trial_id: spec for spec in design.trials}
    missing = sorted(set(expected) - set(by_id))
    extra = sorted(set(by_id) - set(expected))
    if missing or extra:
        raise ValueError(f"incomplete design: {len(missing)} missing, {len(extra)} unexpected")
    manifest_keys = {(r.manifest.commit, r.manifest.configuration_hash, r.manifest.design_hash)
                     for r in records}
    if len(manifest_keys) != 1:
        raise ValueError("manifest commit/configuration/design hashes differ across trials")
    if next(iter(manifest_keys))[2] != design.design_hash:
        raise ValueError("record design hash does not match frozen design")
    for record in records:
        if record.spec != expected[record.spec.trial_id]:
            raise ValueError(f"trial specification changed: {record.spec.trial_id}")
        record.validate()
    paired = defaultdict(dict)
    for record in records:
        key = (record.spec.family, record.spec.layout_id, record.spec.replicate)
        paired[key][record.spec.method] = record
    incomplete = [key for key, values in paired.items() if set(values) != set(METHODS)]
    if incomplete:
        raise ValueError(f"unpaired method blocks: {incomplete[:5]}")
    for values in paired.values():
        seeds = {(r.spec.world_seed, r.spec.sensing_seed, r.spec.friction_seed, r.spec.fault_seed)
                 for r in values.values()}
        if len(seeds) != 1:
            raise ValueError("disturbance seeds differ within a paired block")


def _filtered(records: Iterable[TrialRecord], families: frozenset[str] | None) -> list[TrialRecord]:
    return [record for record in records if families is None or record.spec.family in families]


def completion_difference(records: Iterable[TrialRecord], treatment: str, control: str,
                          families: frozenset[str] | None = None) -> float:
    selected = _filtered(records, families)
    keyed = {(r.spec.layout_id, r.spec.replicate, r.spec.method): r for r in selected}
    pairs = {(r.spec.layout_id, r.spec.replicate) for r in selected if r.spec.method == treatment}
    differences = [float(keyed[(*key, treatment)].completed) -
                   float(keyed[(*key, control)].completed) for key in sorted(pairs)]
    if not differences:
        raise ValueError("contrast has no paired observations")
    return fmean(differences)


def _layout_bootstrap(records: list[TrialRecord], treatment: str, control: str,
                      families: frozenset[str] | None, draws: int, seed: int) -> tuple[float, float]:
    grouped: dict[str, dict[str, list[TrialRecord]]] = defaultdict(lambda: defaultdict(list))
    for record in _filtered(records, families):
        grouped[record.spec.family][record.spec.layout_id].append(record)
    rng, estimates = Random(seed), []
    for _ in range(draws):
        sample = []
        for layouts in grouped.values():
            ids = sorted(layouts)
            for _ in ids:
                sample.extend(layouts[rng.choice(ids)])
        estimates.append(completion_difference(sample, treatment, control))
    estimates.sort()
    return estimates[int(0.025 * (draws - 1))], estimates[int(0.975 * (draws - 1))]


def _layout_permutation_p(records: list[TrialRecord], treatment: str, control: str,
                          families: frozenset[str] | None, draws: int, seed: int) -> float:
    by_pair = defaultdict(dict)
    for record in _filtered(records, families):
        by_pair[(record.spec.family, record.spec.layout_id, record.spec.replicate)][record.spec.method] = record
    per_layout: dict[tuple[str, str], list[float]] = defaultdict(list)
    for (family, layout, _), values in by_pair.items():
        per_layout[(family, layout)].append(float(values[treatment].completed) - float(values[control].completed))
    effects = [fmean(values) for values in per_layout.values()]
    observed, rng, exceed = abs(fmean(effects)), Random(seed), 0
    for _ in range(draws):
        statistic = abs(fmean(effect * (-1 if rng.getrandbits(1) else 1) for effect in effects))
        exceed += statistic >= observed - 1e-12
    return (exceed + 1) / (draws + 1)


def _holm(p_values: list[float]) -> list[float]:
    ordered = sorted(enumerate(p_values), key=lambda pair: pair[1])
    adjusted, running, count = [0.0] * len(p_values), 0.0, len(p_values)
    for rank, (index, value) in enumerate(ordered):
        running = max(running, min(1.0, (count - rank) * value))
        adjusted[index] = running
    return adjusted


def analyze(records: list[TrialRecord], design: StudyDesign,
            bootstrap_draws: int = 10000, permutation_draws: int = 100000) -> dict:
    validate_records(records, design)
    method_rows = []
    for method in METHODS:
        values = [record for record in records if record.spec.method == method]
        successes = [record for record in values if record.completed]
        errors = [error for record in values for error in record.localization_error_m]
        method_rows.append({
            "method": method, "trials": len(values), "completed": len(successes),
            "completion_rate": fmean(float(r.completed) for r in values),
            "deadline_penalized_time_s": fmean(r.deadline_penalized_time_s for r in values),
            "successful_time_s": fmean(r.simulated_duration_s for r in successes) if successes else None,
            "mechanical_work_j": fmean(r.mechanical_work_j for r in values),
            "planning_latency_s": fmean(r.planning_latency_s for r in values),
            "docking_attempts": sum(r.reconfiguration_attempts for r in values),
            "localization_rmse_m": sqrt(fmean(error * error for error in errors)) if errors else None,
        })
    raw_p, contrasts = [], []
    for index, (treatment, control, families) in enumerate(PRIMARY_CONTRASTS):
        estimate = completion_difference(records, treatment, control, families)
        low, high = _layout_bootstrap(records, treatment, control, families,
                                      bootstrap_draws, 8100 + index)
        p_value = _layout_permutation_p(records, treatment, control, families,
                                        permutation_draws, 9100 + index)
        raw_p.append(p_value)
        contrasts.append({
            "treatment": treatment, "control": control,
            "families": sorted(families) if families else "all",
            "absolute_completion_rate_difference": estimate,
            "ci95_layout_bootstrap": [low, high], "permutation_p": p_value,
        })
    for contrast, adjusted in zip(contrasts, _holm(raw_p)):
        contrast["holm_adjusted_p"] = adjusted
    predictions = [(p, y) for r in records for p, y in zip(
        r.predicted_transition_probabilities, r.observed_transition_outcomes)]
    calibration = []
    for index in range(10):
        lower, upper = index / 10, (index + 1) / 10
        values = [(p, y) for p, y in predictions if lower <= p < upper or (upper == 1 and p == 1)]
        if values:
            calibration.append({"lower": lower, "upper": upper, "count": len(values),
                                "mean_prediction": fmean(p for p, _ in values),
                                "observed_frequency": fmean(y for _, y in values)})
    return {
        "schema_version": 1, "design_hash": design.design_hash,
        "trial_count": len(records), "methods": method_rows,
        "primary_contrasts": contrasts,
        "failure_taxonomy": dict(sorted(Counter(r.terminal_status for r in records if not r.completed).items())),
        "transition_brier_score": fmean((p - y) ** 2 for p, y in predictions) if predictions else None,
        "transition_calibration": calibration,
    }


def write_artifacts(result: dict, output: str | Path) -> None:
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "analysis.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    with (output / "method_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=result["methods"][0].keys())
        writer.writeheader(); writer.writerows(result["methods"])
    with (output / "primary_contrasts.csv").open("w", newline="", encoding="utf-8") as stream:
        fields = ("treatment", "control", "families", "absolute_completion_rate_difference",
                  "ci95_low", "ci95_high", "permutation_p", "holm_adjusted_p")
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for value in result["primary_contrasts"]:
            writer.writerow({"treatment": value["treatment"], "control": value["control"],
                "families": ";".join(value["families"]) if isinstance(value["families"], list) else value["families"],
                "absolute_completion_rate_difference": value["absolute_completion_rate_difference"],
                "ci95_low": value["ci95_layout_bootstrap"][0], "ci95_high": value["ci95_layout_bootstrap"][1],
                "permutation_p": value["permutation_p"], "holm_adjusted_p": value["holm_adjusted_p"]})
    lines = ["# Confirmatory analysis", "", f"Design hash: `{result['design_hash']}`", "",
             f"Terminal mission records: {result['trial_count']}", "", "## Completion and cost", "",
             "| Method | Completion | Deadline-penalized time (s) | Work (J) |", "|---|---:|---:|---:|"]
    for row in result["methods"]:
        lines.append(f"| {row['method']} | {row['completion_rate']:.3f} | {row['deadline_penalized_time_s']:.2f} | {row['mechanical_work_j']:.2f} |")
    lines.extend(["", "## Primary contrasts", "",
                  "| Treatment − control | Difference (95% CI) | Holm p |", "|---|---:|---:|"])
    for row in result["primary_contrasts"]:
        low, high = row["ci95_layout_bootstrap"]
        lines.append(f"| {row['treatment']} − {row['control']} | {row['absolute_completion_rate_difference']:.3f} [{low:.3f}, {high:.3f}] | {row['holm_adjusted_p']:.4g} |")
    (output / "results.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_analysis(raw: str | Path, design: StudyDesign, output: str | Path) -> dict:
    result = analyze(TrialStore(raw).load_all(), design)
    write_artifacts(result, output)
    return result

from __future__ import annotations

from collections import Counter, defaultdict
import csv
from itertools import product
import json
from math import sqrt
from pathlib import Path
from random import Random
from statistics import fmean, median
from typing import Callable, Iterable

from .design import METHODS, SENSING_FAMILIES, StudyDesign
from .records import TrialRecord, TrialStore


PRIMARY_CONTRASTS = (
    ("sensing_feasibility_coupled", "geometry_coupled", None),
    ("sensing_feasibility_coupled", "feasibility_coupled", SENSING_FAMILIES),
)
Metric = Callable[[TrialRecord], float]


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


def _paired_layout_effects(records: Iterable[TrialRecord], treatment: str, control: str,
                           families: frozenset[str] | None, metric: Metric) -> dict[str, list[float]]:
    by_block: dict[tuple[str, str, int], dict[str, TrialRecord]] = defaultdict(dict)
    for record in _filtered(records, families):
        by_block[(record.spec.family, record.spec.layout_id, record.spec.replicate)][record.spec.method] = record
    by_layout: dict[tuple[str, str], list[float]] = defaultdict(list)
    for (family, layout, _), values in by_block.items():
        if treatment not in values or control not in values:
            raise ValueError(f"contrast is unpaired in {family}/{layout}")
        by_layout[(family, layout)].append(metric(values[treatment]) - metric(values[control]))
    grouped: dict[str, list[float]] = defaultdict(list)
    for (family, _), effects in by_layout.items():
        grouped[family].append(fmean(effects))
    if not grouped:
        raise ValueError("contrast has no paired observations")
    return dict(grouped)


def _mean_effect(grouped: dict[str, list[float]]) -> float:
    return fmean(effect for effects in grouped.values() for effect in effects)


def completion_difference(records: Iterable[TrialRecord], treatment: str, control: str,
                          families: frozenset[str] | None = None) -> float:
    effects = _paired_layout_effects(records, treatment, control, families,
                                     lambda record: float(record.completed))
    return _mean_effect(effects)


def _layout_bootstrap(records: list[TrialRecord], treatment: str, control: str,
                      families: frozenset[str] | None, metric: Metric,
                      draws: int, seed: int) -> tuple[float, float]:
    """Stratified cluster bootstrap; duplicate sampled layouts retain multiplicity."""
    grouped = _paired_layout_effects(records, treatment, control, families, metric)
    rng, estimates = Random(seed), []
    for _ in range(draws):
        sampled = []
        for effects in grouped.values():
            sampled.extend(rng.choice(effects) for _ in effects)
        estimates.append(fmean(sampled))
    estimates.sort()
    return estimates[int(0.025 * (draws - 1))], estimates[int(0.975 * (draws - 1))]


def _layout_permutation_test(records: list[TrialRecord], treatment: str, control: str,
                             families: frozenset[str] | None, metric: Metric,
                             draws: int, seed: int) -> dict:
    effects = [value for values in _paired_layout_effects(
        records, treatment, control, families, metric).values() for value in values]
    observed = abs(fmean(effects))
    # The frozen contrasts contain 36 and 24 independent layouts. Enumerate
    # small pilots exactly; use a reproducible Monte Carlo test otherwise.
    if len(effects) <= 20:
        statistics = (
            abs(fmean(effect * sign for effect, sign in zip(effects, signs)))
            for signs in product((-1, 1), repeat=len(effects))
        )
        exceed = total = 0
        for statistic in statistics:
            total += 1
            exceed += statistic >= observed - 1e-12
        probability = exceed / total
        return {
            "p_value": probability,
            "method": "exact_sign_flip",
            "assignments": total,
            "monte_carlo_standard_error": None,
            # A two-sided sign test cannot attain a nonzero p below the two
            # equally extreme all-positive/all-negative assignments.
            "minimum_attainable_two_sided_p": min(1.0, 2.0 / total),
        }
    rng, exceed = Random(seed), 0
    for _ in range(draws):
        statistic = abs(fmean(effect * (-1 if rng.getrandbits(1) else 1) for effect in effects))
        exceed += statistic >= observed - 1e-12
    probability = (exceed + 1) / (draws + 1)
    return {
        "p_value": probability,
        "method": f"monte_carlo_sign_flip_{draws}_draws",
        "assignments": draws,
        "monte_carlo_standard_error": sqrt(
            probability * (1.0 - probability) / (draws + 1)),
        "minimum_attainable_two_sided_p": None,
    }


def _holm(p_values: list[float]) -> list[float]:
    ordered = sorted(enumerate(p_values), key=lambda pair: pair[1])
    adjusted, running, count = [0.0] * len(p_values), 0.0, len(p_values)
    for rank, (index, value) in enumerate(ordered):
        running = max(running, min(1.0, (count - rank) * value))
        adjusted[index] = running
    return adjusted


def _method_rate_ci(records: list[TrialRecord], method: str, draws: int, seed: int) -> list[float]:
    grouped: dict[str, dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    for record in records:
        if record.spec.method == method:
            grouped[record.spec.family][record.spec.layout_id].append(float(record.completed))
    rng, estimates = Random(seed), []
    for _ in range(draws):
        values = []
        for layouts in grouped.values():
            ids = sorted(layouts)
            values.extend(fmean(layouts[rng.choice(ids)]) for _ in ids)
        estimates.append(fmean(values))
    estimates.sort()
    return [estimates[int(0.025 * (draws - 1))], estimates[int(0.975 * (draws - 1))]]


def _calibration(predictions: list[tuple[float, int]]) -> list[dict]:
    rows = []
    for index in range(10):
        lower, upper = index / 10, (index + 1) / 10
        values = [(p, y) for p, y in predictions
                  if lower <= p < upper or (upper == 1 and p == 1)]
        if values:
            rows.append({"lower": lower, "upper": upper, "count": len(values),
                         "mean_prediction": fmean(p for p, _ in values),
                         "observed_frequency": fmean(y for _, y in values)})
    return rows


def _contrast(records: list[TrialRecord], treatment: str, control: str,
              families: frozenset[str] | None, metric: Metric, metric_name: str,
              draws: int, permutation_draws: int | None, seed: int) -> dict:
    grouped = _paired_layout_effects(records, treatment, control, families, metric)
    low, high = _layout_bootstrap(records, treatment, control, families, metric, draws, seed)
    row = {
        "outcome": metric_name, "treatment": treatment, "control": control,
        "families": sorted(families) if families else "all",
        "estimate_treatment_minus_control": _mean_effect(grouped),
        "ci95_layout_bootstrap": [low, high],
        "layout_count": sum(len(values) for values in grouped.values()),
        "family_effects": {family: fmean(values) for family, values in sorted(grouped.items())},
    }
    if permutation_draws is not None:
        test = _layout_permutation_test(
            records, treatment, control, families, metric, permutation_draws, seed + 1000)
        row["permutation_p"] = test.pop("p_value")
        row["permutation_method"] = test.pop("method")
        row.update(test)
    return row


def analyze(records: list[TrialRecord], design: StudyDesign,
            bootstrap_draws: int = 10000, permutation_draws: int = 100000) -> dict:
    if bootstrap_draws < 40 or permutation_draws < 100:
        raise ValueError("analysis draw counts are too small")
    validate_records(records, design)
    method_rows = []
    for index, method in enumerate(METHODS):
        values = [record for record in records if record.spec.method == method]
        successes = [record for record in values if record.completed]
        errors = [error for record in values for error in record.localization_error_m]
        clearances = [record.minimum_clearance_m for record in values
                      if record.minimum_clearance_m is not None]
        method_rows.append({
            "method": method, "trials": len(values), "completed": len(successes),
            "completion_rate": fmean(float(r.completed) for r in values),
            "completion_rate_ci95": _method_rate_ci(records, method, bootstrap_draws, 7100 + index),
            "deadline_penalized_time_s": fmean(r.deadline_penalized_time_s for r in values),
            "successful_time_s": fmean(r.simulated_duration_s for r in successes) if successes else None,
            "mechanical_work_j": fmean(r.mechanical_work_j for r in values),
            "planning_latency_s": fmean(r.planning_latency_s for r in values),
            "expanded_states": fmean(r.expanded_states for r in values),
            "docking_attempts": sum(r.reconfiguration_attempts for r in values),
            "reconfiguration_failures": sum(r.reconfiguration_failures for r in values),
            "recovery_actions": sum(r.recovery_actions for r in values),
            "motion_qualifications": sum(len(r.motion_qualifications) for r in values),
            "motion_qualification_failures": sum(
                not q["passed"] for r in values for q in r.motion_qualifications),
            "minimum_clearance_median_m": median(clearances) if clearances else None,
            "localization_rmse_m": sqrt(fmean(error * error for error in errors)) if errors else None,
        })

    primary = []
    for index, (treatment, control, families) in enumerate(PRIMARY_CONTRASTS):
        primary.append(_contrast(
            records, treatment, control, families, lambda r: float(r.completed),
            "completion_rate", bootstrap_draws, permutation_draws, 8100 + index,
        ))
    for row, adjusted in zip(primary, _holm([row["permutation_p"] for row in primary])):
        row["holm_adjusted_p"] = adjusted
        row["absolute_completion_rate_difference"] = row["estimate_treatment_minus_control"]

    secondary = []
    for index, (treatment, control, families) in enumerate(PRIMARY_CONTRASTS):
        secondary.append(_contrast(
            records, treatment, control, families, lambda r: r.deadline_penalized_time_s,
            "deadline_penalized_time_s", bootstrap_draws, None, 10100 + index,
        ))

    predictions_by_method: dict[str, list[tuple[float, int]]] = defaultdict(list)
    for record in records:
        predictions_by_method[record.spec.method].extend(zip(
            record.predicted_transition_probabilities, record.observed_transition_outcomes))
    predictions = [value for values in predictions_by_method.values() for value in values]
    rejection_by_method: dict[str, Counter] = defaultdict(Counter)
    edge_counts_by_method: dict[str, Counter] = defaultdict(Counter)
    for record in records:
        for decision in record.transition_edge_decisions:
            edge_counts_by_method[record.spec.method][
                "accepted" if decision["feasible"] else "rejected"
            ] += 1
            if not decision["feasible"]:
                for reason in decision["reasons"] or ["unspecified"]:
                    rejection_by_method[record.spec.method][reason] += 1

    route_disagreement = {}
    full_method = "sensing_feasibility_coupled"
    blocks: dict[tuple[str, str, int], dict[str, TrialRecord]] = defaultdict(dict)
    for record in records:
        blocks[(record.spec.family, record.spec.layout_id, record.spec.replicate)][record.spec.method] = record
    for method in METHODS:
        if method == full_method:
            continue
        comparable = [values for values in blocks.values()
                      if values[method].planned_route_signature
                      and values[full_method].planned_route_signature]
        route_disagreement[method] = {
            "paired_blocks_with_signatures": len(comparable),
            "different_route_or_transition_fraction": (
                fmean(values[method].planned_route_signature !=
                      values[full_method].planned_route_signature
                      for values in comparable) if comparable else None
            ),
        }
    brier_by_method = {
        method: fmean((p - y) ** 2 for p, y in values)
        for method, values in sorted(predictions_by_method.items()) if values
    }
    return {
        "schema_version": 2, "design_hash": design.design_hash,
        "trial_count": len(records), "layout_count": len({r.spec.layout_id for r in records}),
        "methods": method_rows, "primary_contrasts": primary,
        "secondary_contrasts": secondary,
        "failure_taxonomy": dict(sorted(Counter(
            r.terminal_status for r in records if not r.completed).items())),
        "transition_brier_score": fmean((p - y) ** 2 for p, y in predictions) if predictions else None,
        "transition_brier_by_method": brier_by_method,
        "transition_calibration": _calibration(predictions),
        "transition_calibration_by_method": {
            method: _calibration(values) for method, values in sorted(predictions_by_method.items()) if values
        },
        "transition_edge_counts_by_method": {
            method: dict(sorted(counts.items()))
            for method, counts in sorted(edge_counts_by_method.items())
        },
        "transition_rejection_reasons_by_method": {
            method: dict(sorted(counts.items()))
            for method, counts in sorted(rejection_by_method.items())
        },
        "route_decision_disagreement_vs_full": route_disagreement,
    }


def _write_contrasts(path: Path, rows: list[dict]) -> None:
    fields = ("outcome", "treatment", "control", "families", "layout_count",
              "estimate_treatment_minus_control", "ci95_low", "ci95_high",
              "permutation_p", "holm_adjusted_p")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for value in rows:
            writer.writerow({
                "outcome": value["outcome"], "treatment": value["treatment"],
                "control": value["control"],
                "families": ";".join(value["families"]) if isinstance(value["families"], list) else value["families"],
                "layout_count": value["layout_count"],
                "estimate_treatment_minus_control": value["estimate_treatment_minus_control"],
                "ci95_low": value["ci95_layout_bootstrap"][0],
                "ci95_high": value["ci95_layout_bootstrap"][1],
                "permutation_p": value.get("permutation_p", ""),
                "holm_adjusted_p": value.get("holm_adjusted_p", ""),
            })


def write_artifacts(result: dict, output: str | Path) -> None:
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "analysis.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    with (output / "method_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=result["methods"][0].keys())
        writer.writeheader(); writer.writerows(result["methods"])
    _write_contrasts(output / "primary_contrasts.csv", result["primary_contrasts"])
    _write_contrasts(output / "secondary_contrasts.csv", result["secondary_contrasts"])
    lines = ["# Confirmatory analysis", "", f"Design hash: `{result['design_hash']}`", "",
             f"Terminal mission records: {result['trial_count']}", "", "## Completion and cost", "",
             "| Method | Completion (cluster 95% CI) | Deadline-penalized time (s) | Work (J) |",
             "|---|---:|---:|---:|"]
    for row in result["methods"]:
        low, high = row["completion_rate_ci95"]
        lines.append(f"| {row['method']} | {row['completion_rate']:.3f} [{low:.3f}, {high:.3f}] | {row['deadline_penalized_time_s']:.2f} | {row['mechanical_work_j']:.2f} |")
    lines.extend(["", "## Primary completion contrasts", "",
                  "| Treatment − control | Difference (95% cluster CI) | Holm p |", "|---|---:|---:|"])
    for row in result["primary_contrasts"]:
        low, high = row["ci95_layout_bootstrap"]
        lines.append(f"| {row['treatment']} − {row['control']} | {row['estimate_treatment_minus_control']:.3f} [{low:.3f}, {high:.3f}] | {row['holm_adjusted_p']:.4g} |")
    lines.extend(["", "## Secondary deadline-penalized-time contrasts", "",
                  "Negative values favor the treatment. These intervals are descriptive and receive no confirmatory p value.", "",
                  "| Treatment − control | Difference in seconds (95% cluster CI) |", "|---|---:|"])
    for row in result["secondary_contrasts"]:
        low, high = row["ci95_layout_bootstrap"]
        lines.append(f"| {row['treatment']} − {row['control']} | {row['estimate_treatment_minus_control']:.2f} [{low:.2f}, {high:.2f}] |")
    lines.extend(["", "## Planner manipulation checks", "",
                  "These are descriptive checks of the planned ablations.", "",
                  "| Baseline compared with full method | Paired blocks | Different decision fraction |",
                  "|---|---:|---:|"])
    for method, value in result["route_decision_disagreement_vs_full"].items():
        fraction = value["different_route_or_transition_fraction"]
        rendered = "NA" if fraction is None else f"{fraction:.3f}"
        lines.append(
            f"| {method} | {value['paired_blocks_with_signatures']} | {rendered} |")
    lines.extend(["", "Structured transition rejection counts are retained in `analysis.json`."])
    (output / "results.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    from .plots import write_svg_figures
    write_svg_figures(result, output / "figures")


def run_analysis(raw: str | Path, design: StudyDesign, output: str | Path,
                 bootstrap_draws: int = 10000, permutation_draws: int = 100000) -> dict:
    result = analyze(TrialStore(raw).load_all(), design, bootstrap_draws, permutation_draws)
    write_artifacts(result, output)
    return result

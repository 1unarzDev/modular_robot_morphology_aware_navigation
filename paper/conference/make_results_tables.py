"""Tables: the confirmatory results, rendered from `morphology_study analyze`.

Reads only the analysis artifact, so every number in the paper's results tables
is a deterministic derivative of the accepted terminal records and none is
transcribed by hand.

    python paper/conference/make_results_tables.py \
        <analysis-output>/analysis.json paper/conference/results_tables.tex
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

METHODS = ("route_first_adaptation", "geometry_coupled",
           "feasibility_coupled", "sensing_feasibility_coupled")


def _method(name: str) -> str:
    return r"\method{" + name.replace("_", r"\_") + "}"


def _ci(low: float, high: float, digits: int) -> str:
    return f"[{low:.{digits}f}, {high:.{digits}f}]"


def _p(value: float) -> str:
    return r"$<$0.001" if value < 0.001 else f"{value:.3f}"


def _metres(value: float | None) -> str:
    return "--" if value is None else f"{value:.3f}"


def method_table(result: dict) -> str:
    rows = {row["method"]: row for row in result["methods"]}
    lines = [
        r"\begin{table}[t]", r"\centering",
        r"\caption{Mission outcomes by method over " + str(result["trial_count"])
        + r" terminal records on " + str(result["layout_count"]) + r" layouts. Completion"
        r" intervals resample whole layouts within family; failures are scored at 300\,s"
        r" in deadline-penalized time.}",
        r"\label{tab:methods_results}",
        r"\begin{tabular}{@{}lrrr@{}}", r"\toprule",
        r"Method & Completion (95\% CI) & Penalized time (s) & Work (J) \\", r"\midrule",
    ]
    for method in METHODS:
        row = rows[method]
        lines.append(
            f"{_method(method)} & {row['completed']}/{row['trials']} "
            f"{_ci(*row['completion_rate_ci95'], 2)} & "
            f"{row['deadline_penalized_time_s']:.1f} & {row['mechanical_work_j']:.0f} \\\\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    return "\n".join(lines)


def contrast_table(result: dict) -> str:
    secondary = {(row["treatment"], row["control"]): row for row in result["secondary_contrasts"]}
    hierarchical = {(row["treatment"], row["control"]): row
                    for row in result["hierarchical_logistic_sensitivity"]}
    lines = [
        r"\begin{table}[t]", r"\centering",
        r"\caption{Confirmatory contrasts: layout-balanced paired differences, treatment"
        r" minus control, with 95\% layout-bootstrap intervals and Holm-adjusted sign-flip"
        r" $p$ values. Penalized time is secondary and carries no $p$ value; the"
        r" hierarchical logistic model is a frozen sensitivity analysis.}",
        r"\label{tab:contrasts}",
        r"\begin{tabular}{@{}lrr@{}}", r"\toprule",
        r" & vs.\ \method{geometry\_coupled} & vs.\ \method{feasibility\_coupled} \\", r"\midrule",
    ]
    primary = result["primary_contrasts"]
    keys = [(row["treatment"], row["control"]) for row in primary]
    cells = {
        "Layouts": [str(row["layout_count"]) for row in primary],
        r"$\Delta$ completion": [f"{row['estimate_treatment_minus_control']:+.3f}" for row in primary],
        r"\quad 95\% CI": [_ci(*row["ci95_layout_bootstrap"], 3) for row in primary],
        r"\quad Holm $p$": [_p(row["holm_adjusted_p"]) for row in primary],
        r"$\Delta$ penalized time (s)": [
            f"{secondary[key]['estimate_treatment_minus_control']:+.1f} "
            f"{_ci(*secondary[key]['ci95_layout_bootstrap'], 1)}" for key in keys],
        r"Hierarchical $\Delta$": [
            f"{hierarchical[key]['marginal_probability_difference']:+.3f} "
            f"{_ci(*hierarchical[key]['ci95_layout_bootstrap'], 3)}" for key in keys],
    }
    for label, values in cells.items():
        lines.append(f"{label} & " + " & ".join(values) + r" \\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    return "\n".join(lines)


def secondary_table(result: dict) -> str:
    measures = result["secondary_measures"]
    lines = [
        r"\begin{table}[t]", r"\centering",
        r"\caption{Secondary measures with their denominators: launch attempts lost to"
        r" infrastructure (not mission outcomes), trials with evaluator contact or"
        r" localization loss, transitions that failed, and achieved site placement"
        r" (median / max evaluator-truth distance from the planned site; nominal target "
        + f"{measures['nominal_site_tolerance_m']:.2f}" + r"\,m).}",
        r"\label{tab:secondary}",
        r"\setlength{\tabcolsep}{3pt}",
        r"\begin{tabular}{@{}lrrrrr@{}}", r"\toprule",
        r"Method & Infra. & Contact & Loss & Trans.\ fail & Placement (m) \\", r"\midrule",
    ]
    for method in METHODS:
        row = measures["by_method"][method]
        lines.append(
            f"{_method(method)} & {row['infrastructure_attempts']}/{row['launch_attempts']} & "
            f"{row['trials_with_contact']}/{row['trials_with_contact_evaluated']} & "
            f"{row['trials_with_localization_loss']}/{row['trials_with_localization_loss_evaluated']} & "
            f"{row['transition_failures']}/{row['transitions_begun']} & "
            f"{_metres(row['site_placement_median_m'])} / {_metres(row['site_placement_max_m'])} \\\\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    return "\n".join(lines)


def render(result: dict) -> str:
    header = (f"% Generated by make_results_tables.py from analysis.json "
              f"(design {result['design_hash'][:12]}); do not edit by hand.")
    return "\n\n".join((header, method_table(result), contrast_table(result),
                        secondary_table(result))) + "\n"


def main() -> None:
    source, destination = Path(sys.argv[1]), Path(sys.argv[2])
    destination.write_text(render(json.loads(source.read_text(encoding="utf-8"))),
                           encoding="utf-8")


if __name__ == "__main__":
    main()

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


# The short labels of the results figure; the full names would overflow a column.
LABELS = {"route_first_adaptation": "route first", "geometry_coupled": "geometry",
          "feasibility_coupled": "feasibility",
          "sensing_feasibility_coupled": "sensing + feasibility"}


def _method(name: str) -> str:
    return LABELS[name]


def _number(value: float, digits: int) -> str:
    return ("$-$" if value < 0 else "") + f"{abs(value):.{digits}f}"


def _ci(low: float, high: float, digits: int) -> str:
    return f"[{_number(low, digits)}, {_number(high, digits)}]"


def _p(value: float) -> str:
    return r"$<$0.001" if value < 0.001 else f"{value:.3f}"


def _metres(value: float | None) -> str:
    return "--" if value is None else f"{value:.3f}"


def _signed(value: float, digits: int) -> str:
    return ("$-$" if value < 0 else "$+$") + f"{abs(value):.{digits}f}"


MACRO_METHODS = {"route_first_adaptation": "RouteFirst", "geometry_coupled": "Geometry",
                 "feasibility_coupled": "Feasibility", "sensing_feasibility_coupled": "Sensing"}
MACRO_FAMILIES = {"reconfiguration_workspace": "Workspace",
                  "docking_observability": "Docking", "combined_constraints": "Combined"}


def _camel(name: str) -> str:
    return "".join(part.capitalize() for part in name.split("_"))


SMALLEST_EFFECT = 0.18


def claim(estimate: float, holm_p: float, low: float, high: float,
          delta: float = SMALLEST_EFFECT) -> str:
    """The claim words the analysis plan's interpretation rules assign.

    Declared 2026-11-04 before any confirmatory outcome was read
    (docs/research/statistical_analysis_plan.md, "Interpretation rules"). An
    effect is established only when the Holm test and the interval agree.
    """
    if holm_p < 0.05 and low > 0:
        if low >= delta:
            return "an improvement of at least the smallest effect of interest"
        if high >= delta:
            return "an improvement, not shown to reach the smallest effect of interest"
        return "an improvement smaller than the smallest effect of interest"
    if holm_p < 0.05 and high < 0:
        return "a deficit: the treatment is worse"
    if high < delta:
        return ("no improvement established; an effect as large as the smallest"
                " effect of interest is excluded")
    return "inconclusive at this sample size"


def macros(result: dict) -> str:
    """Inline numbers for the Results prose, so none is transcribed by hand.

    Every macro is `\\res...`; contrasts are keyed `VsGeometry` / `VsFeasibility`
    by their control, methods by `MACRO_METHODS`, families by `MACRO_FAMILIES`.
    """
    values: dict[str, str] = {
        "Trials": str(result["trial_count"]), "Layouts": str(result["layout_count"]),
        "Brier": f"{result['transition_brier_score']:.3f}",
    }
    for row in result["methods"]:
        key = MACRO_METHODS[row["method"]]
        low, high = row["completion_rate_ci95"]
        values.update({
            f"Completed{key}": str(row["completed"]), f"Trials{key}": str(row["trials"]),
            f"Rate{key}": f"{row['completion_rate']:.2f}",
            f"RateLow{key}": f"{low:.2f}", f"RateHigh{key}": f"{high:.2f}",
            f"Time{key}": f"{row['deadline_penalized_time_s']:.1f}",
            f"Brier{key}": f"{result['transition_brier_by_method'][row['method']]:.3f}",
        })
    secondary = {row["control"]: row for row in result["secondary_contrasts"]
                 if row["treatment"] == "sensing_feasibility_coupled"}
    hierarchical = {row["control"]: row for row in result["hierarchical_logistic_sensitivity"]
                    if row["treatment"] == "sensing_feasibility_coupled"}
    for row in result["primary_contrasts"]:
        key = "Vs" + MACRO_METHODS[row["control"]]
        low, high = row["ci95_layout_bootstrap"]
        time_row, model = secondary[row["control"]], hierarchical[row["control"]]
        time_low, time_high = time_row["ci95_layout_bootstrap"]
        model_low, model_high = model["ci95_layout_bootstrap"]
        values.update({
            f"Layouts{key}": str(row["layout_count"]),
            f"Delta{key}": _signed(row["estimate_treatment_minus_control"], 3),
            f"CILow{key}": _signed(low, 3), f"CIHigh{key}": _signed(high, 3),
            f"RawP{key}": _p(row["permutation_p"]), f"HolmP{key}": _p(row["holm_adjusted_p"]),
            f"Claim{key}": claim(row["estimate_treatment_minus_control"],
                                 row["holm_adjusted_p"], low, high),
            f"TimeDelta{key}": _signed(time_row["estimate_treatment_minus_control"], 1),
            f"TimeCILow{key}": _signed(time_low, 1), f"TimeCIHigh{key}": _signed(time_high, 1),
            f"HierDelta{key}": _signed(model["marginal_probability_difference"], 3),
            f"HierCILow{key}": _signed(model_low, 3), f"HierCIHigh{key}": _signed(model_high, 3),
            f"HierConverged{key}": str(model["bootstrap_draws_converged"]),
            f"HierRequested{key}": str(model["bootstrap_draws_requested"]),
        })
        for family, effect in row["family_effects"].items():
            values[f"Family{key}{MACRO_FAMILIES[family]}"] = _signed(effect, 3)
    for status, count in result["failure_taxonomy"].items():
        values[f"Fail{_camel(status)}"] = str(count)
    by_method = result["secondary_measures"]["by_method"].values()
    for field, name in (("launch_attempts", "LaunchAttempts"),
                        ("infrastructure_attempts", "InfraAttempts"),
                        ("trials_retried", "TrialsRetried"),
                        ("trials_ending_infrastructure_failure", "InfraFailures")):
        values[name] = str(sum(row[field] for row in by_method))
    return "\n".join(rf"\newcommand{{\res{name}}}{{{value}}}"
                     for name, value in sorted(values.items()))


def method_table(result: dict) -> str:
    rows = {row["method"]: row for row in result["methods"]}
    lines = [
        r"\begin{table}[t]", r"\centering",
        r"\caption{Mission outcomes by method over " + str(result["trial_count"])
        + r" terminal records on " + str(result["layout_count"]) + r" layouts. Completion"
        r" intervals resample whole layouts within family; failures are scored at 300\,s"
        r" in deadline-penalized time.}",
        r"\label{tab:methods_results}",
        r"\begin{tabular}{@{}lrrrr@{}}", r"\toprule",
        r"Method & Completed & 95\% CI & Pen.\ time (s) & Work (J) \\", r"\midrule",
    ]
    for method in METHODS:
        row = rows[method]
        lines.append(
            f"{_method(method)} & {row['completed']}/{row['trials']} & "
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
        r" & vs.\ geometry & vs.\ feasibility \\", r"\midrule",
    ]
    primary = result["primary_contrasts"]
    keys = [(row["treatment"], row["control"]) for row in primary]
    cells = [
        ("Layouts", [str(row["layout_count"]) for row in primary]),
        (r"$\Delta$ completion", [_signed(row["estimate_treatment_minus_control"], 3)
                                  for row in primary]),
        (r"\quad 95\% CI", [_ci(*row["ci95_layout_bootstrap"], 3) for row in primary]),
        (r"\quad Holm $p$", [_p(row["holm_adjusted_p"]) for row in primary]),
        (r"$\Delta$ pen.\ time (s)", [
            _signed(secondary[key]["estimate_treatment_minus_control"], 1) for key in keys]),
        (r"\quad 95\% CI", [_ci(*secondary[key]["ci95_layout_bootstrap"], 1) for key in keys]),
        (r"Hierarchical $\Delta$", [
            _signed(hierarchical[key]["marginal_probability_difference"], 3) for key in keys]),
        (r"\quad 95\% CI", [_ci(*hierarchical[key]["ci95_layout_bootstrap"], 3) for key in keys]),
        (r"\quad converged refits", [
            f"{hierarchical[key]['bootstrap_draws_converged']}/"
            f"{hierarchical[key]['bootstrap_draws_requested']}" for key in keys]),
    ]
    for label, values in cells:
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
    return "\n\n".join((header, macros(result), method_table(result), contrast_table(result),
                        secondary_table(result))) + "\n"


def main() -> None:
    source, destination = Path(sys.argv[1]), Path(sys.argv[2])
    destination.write_text(render(json.loads(source.read_text(encoding="utf-8"))),
                           encoding="utf-8")


if __name__ == "__main__":
    main()

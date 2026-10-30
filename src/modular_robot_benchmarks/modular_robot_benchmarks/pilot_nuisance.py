"""Gate 3: nuisance parameters for the frozen power grid, estimated from the pilot.

Declared 2026-10-30, before any pilot record existed, so that nothing about
how the pilot is read can be chosen after seeing it. The analysis plan fixes
the smallest effect of interest (0.18) independently; the pilot supplies only
the three nuisance parameters of `power.estimate_power`, for each declared
contrast scope:

- control probability at an average-difficulty layout: the control method's
  conditional completion probability from the predeclared random-intercept
  logistic fit (`hierarchical._fit`), families weighted equally;
- layout logit standard deviation: that fit's random-intercept SD;
- paired-noise fraction: a moment estimator. In the power model, a paired
  block's two outcomes share one uniform draw with probability rho, so the
  probability they agree is rho * A + (1 - rho) * B, where A = E[1 - |p_t - p_c|]
  (shared draw) and B = E[p_t p_c + (1 - p_t)(1 - p_c)] (independent draws),
  both integrated over the fitted layout distribution. rho is the observed
  agreement solved against those, clipped to [0, 1].

The adverse grid values are declared here too: the control rate's 95% Wilson
bound nearer 0.5 (where a binary outcome is noisiest), max(2 sigma, sigma + 0.5,
1.0) for the layout SD (the floor because sigma is weakly identified), and max(0, rho - 0.25) for the pairing, since less pairing
means noisier paired differences. Pilot effect estimates are reported for
runtime and measurement validation only and never feed the grid.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import json
from math import pi, sqrt
from pathlib import Path
from statistics import fmean

import numpy as np

from .design import SENSING_FAMILIES, StudyDesign
from .hierarchical import _fit, _logistic
from .power import _wilson_interval, estimate_power_grid
from .records import TrialRecord, TrialStore


SMALLEST_EFFECT_OF_INTEREST = 0.18
# The analysis plan's two primary contrasts, as (name, treatment, control, families).
DECLARED_SCOPES = (
    ("all_families", "sensing_feasibility_coupled", "geometry_coupled", None),
    ("sensing_families", "sensing_feasibility_coupled", "feasibility_coupled",
     frozenset(SENSING_FAMILIES)),
)
QUADRATURE_POINTS = 15
# The layout SD is weakly identified: in simulation at the true 0.8 one seed in
# four collapsed the fitted SD to ~0 even with 150 layouts per family, and the
# pilot has 6. The adverse value therefore never falls below this.
ADVERSE_LAYOUT_SD_FLOOR = 1.0


def _scoped(records, families):
    return [record for record in records
            if families is None or record.spec.family in families]


def _family_log_odds(beta: np.ndarray, family_names: list[str], method_value: float):
    """Log-odds at a zero layout shift for each family, in `_fit`'s coding."""
    columns = family_names[1:]
    return [float(beta @ np.asarray([1.0, method_value,
                                     *(float(family == column) for column in columns)]))
            for family in family_names]


def paired_noise_fraction(blocks: list[tuple[int, int]], control_log_odds: list[float],
                          treatment_log_odds: list[float], layout_sd: float,
                          quadrature_points: int = QUADRATURE_POINTS) -> dict:
    """Moment estimate of the power model's pairing fraction.

    `blocks` holds one (control_completed, treatment_completed) pair per paired
    layout-replicate block. The log-odds lists hold one entry per family, at a
    zero layout shift; expectations integrate the Gaussian layout shift.
    """
    if not blocks:
        raise ValueError("no paired blocks to estimate pairing from")
    nodes, weights = np.polynomial.hermite.hermgauss(quadrature_points)
    shared = independent = 0.0
    for control, treatment in zip(control_log_odds, treatment_log_odds):
        for node, weight in zip(nodes, weights):
            shift = sqrt(2.0) * layout_sd * float(node)
            p_c, p_t = _logistic(control + shift), _logistic(treatment + shift)
            mass = float(weight) / sqrt(pi) / len(control_log_odds)
            shared += mass * (1.0 - abs(p_t - p_c))
            independent += mass * (p_t * p_c + (1.0 - p_t) * (1.0 - p_c))
    observed = fmean(float(c == t) for c, t in blocks)
    if shared - independent < 1e-9:
        rho = 1.0   # the model cannot tell pairing apart here; least favourable
    else:
        rho = min(1.0, max(0.0, (observed - independent) / (shared - independent)))
    return {"estimate": rho, "observed_agreement": observed,
            "agreement_if_shared": shared, "agreement_if_independent": independent,
            "blocks": len(blocks)}


def adverse_values(control_rate: float, control_successes: int, control_trials: int,
                   layout_sd: float, pairing: float) -> dict:
    """The declared adverse companion of each estimate (see module docstring)."""
    low, high = _wilson_interval(control_successes, control_trials)
    adverse_rate = low if abs(low - 0.5) < abs(high - 0.5) else high
    # A control rate the effect cannot sit on top of is reported, not hidden.
    ceiling = 1.0 - SMALLEST_EFFECT_OF_INTEREST - 0.01
    return {
        "control_rates": sorted({round(min(control_rate, ceiling), 4),
                                 round(min(adverse_rate, ceiling), 4)}),
        "control_rate_capped_for_effect": control_rate > ceiling or adverse_rate > ceiling,
        "layout_logit_sds": sorted({round(layout_sd, 4), round(max(
            2.0 * layout_sd, layout_sd + 0.5, ADVERSE_LAYOUT_SD_FLOOR), 4)}),
        "paired_noise_fractions": sorted({round(pairing, 4),
                                          round(max(0.0, pairing - 0.25), 4)}),
    }


def estimate_scope(records: list[TrialRecord], treatment: str, control: str,
                   families) -> dict:
    selected = [record for record in _scoped(records, families)
                if record.spec.method in (treatment, control)]
    # A layout random intercept fitted to paired blocks absorbs the pairing
    # itself as layout variance (measured: 1.2 recovered for a true 0.8 at
    # rho 0.5). So the layout model is fitted to crossed halves in which no two
    # observations of a layout share a replicate -- control from even
    # replicates with treatment from odd ones, then the mirror -- where
    # outcomes within a layout share only the layout shift. The two fits are
    # averaged; pairing is then read from every block.
    fits = []
    for control_parity in (0, 1):
        half = [record for record in selected
                if (record.spec.replicate % 2 == control_parity)
                == (record.spec.method == control)]
        fits.append(_fit(half, treatment, control, families, QUADRATURE_POINTS))
    family_names = fits[0]["families_in_order"]
    beta = np.mean([np.asarray(fit["coefficients"]) for fit in fits], axis=0)
    sd = fmean(float(fit["layout_random_intercept_sd"]) for fit in fits)
    fit = {"converged": all(value["converged"] for value in fits),
           "marginal_probability_difference": fmean(
               value["marginal_probability_difference"] for value in fits)}
    control_log_odds = _family_log_odds(beta, family_names, 0.0)
    treatment_log_odds = _family_log_odds(beta, family_names, 1.0)
    control_rate = fmean(_logistic(value) for value in control_log_odds)
    blocks_by_key: dict[tuple[str, int], dict[str, int]] = defaultdict(dict)
    for record in selected:
        blocks_by_key[(record.spec.layout_id, record.spec.replicate)][
            record.spec.method] = int(record.completed)
    blocks = [(value[control], value[treatment]) for value in blocks_by_key.values()
              if control in value and treatment in value]
    pairing = paired_noise_fraction(blocks, control_log_odds, treatment_log_odds, sd)
    control_records = [record for record in selected if record.spec.method == control]
    successes = sum(record.completed for record in control_records)
    return {
        "treatment": treatment, "control": control,
        "families": sorted(families) if families else "all",
        "fit_converged": fit["converged"],
        "control_probability_at_average_layout": control_rate,
        "control_marginal_rate": successes / len(control_records),
        "control_trials": len(control_records),
        "layout_logit_sd": sd,
        "paired_noise_fraction": pairing,
        "grid": adverse_values(control_rate, successes, len(control_records),
                               sd, pairing["estimate"]),
        # Runtime and measurement validation only; never an input to the grid.
        "pilot_effect_not_used_for_power": fit["marginal_probability_difference"],
    }


def runtime_summary(records: list[TrialRecord]) -> dict:
    walls = [float(record.wall_duration_s) for record in records]
    attempts = [len(record.infrastructure_attempts) for record in records]
    return {
        "trials": len(records),
        "mean_wall_s": fmean(walls) if walls else None,
        "max_wall_s": max(walls) if walls else None,
        "trials_with_infrastructure_retry": sum(value > 1 for value in attempts),
        "terminal_statuses": dict(sorted(_count(r.terminal_status for r in records).items())),
    }


def _count(values) -> dict:
    counts: dict = defaultdict(int)
    for value in values:
        counts[value] += 1
    return counts


def required_layouts(grid: dict, candidates: list[int], replicates: int,
                     simulations: int, randomization_draws: int, seed: int) -> dict:
    """Smallest candidate layout count whose every grid cell reaches 80% power
    on its Wilson lower bound, per the analysis plan's frozen procedure."""
    tried = []
    for layouts in candidates:
        result = estimate_power_grid(
            layouts, replicates, grid["control_rates"], SMALLEST_EFFECT_OF_INTEREST,
            grid["layout_logit_sds"], grid["paired_noise_fractions"],
            simulations=simulations, randomization_draws=randomization_draws, seed=seed)
        tried.append({"layouts": layouts,
                      "all_cells_pass": result["design_meets_target_in_every_cell"],
                      "minimum_power_ci95_lower_bound": result["minimum_power_ci95_lower_bound"],
                      "worst_case_assumptions": result["worst_case_assumptions"]})
        if result["design_meets_target_in_every_cell"]:
            return {"required_layouts": layouts, "tried": tried}
    return {"required_layouts": None, "tried": tried}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--design", type=Path, required=True)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--power", action="store_true",
                        help="also search the layout count with the frozen grid")
    parser.add_argument("--simulations", type=int, default=2000)
    parser.add_argument("--randomization-draws", type=int, default=1999)
    arguments = parser.parse_args()
    design = StudyDesign.read_frozen(arguments.design)
    store = TrialStore(arguments.raw)
    records = store.load_all()
    expected = {spec.trial_id for spec in design.trials}
    observed = {record.spec.trial_id for record in records}
    summary = {
        "schema_version": 1,
        "purpose": "gate3_pilot_nuisance_parameters",
        "design_hash": design.design_hash,
        "campaign_digest": store.campaign_digest(),
        "record_digests": store.digests(),
        "complete": observed == expected,
        "missing_trials": sorted(expected - observed),
        "smallest_effect_of_interest": SMALLEST_EFFECT_OF_INTEREST,
        "runtime": runtime_summary(records),
        "scopes": {name: estimate_scope(records, treatment, control, families)
                   for name, treatment, control, families in DECLARED_SCOPES},
    }
    if arguments.power:
        # All-family scope over three families; sensing scope over two. The
        # confirmatory design draws equal layouts per family, so a total of 3n
        # layouts gives the sensing scope 2n.
        for name, scope in summary["scopes"].items():
            totals = list(range(36, 121, 12))
            counts = totals if name == "all_families" else [2 * total // 3 for total in totals]
            scope["power"] = required_layouts(
                scope["grid"], counts, 3, arguments.simulations,
                arguments.randomization_draws, 20261030)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n",
                                encoding="utf-8")
    print(json.dumps({name: {key: scope[key] for key in (
        "control_probability_at_average_layout", "layout_logit_sd", "grid")}
        for name, scope in summary["scopes"].items()}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

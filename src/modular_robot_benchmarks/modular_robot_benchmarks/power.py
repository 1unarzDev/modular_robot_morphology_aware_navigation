from __future__ import annotations

from dataclasses import asdict, dataclass
from itertools import product
import json
from math import exp, log, sqrt
from pathlib import Path
from random import Random
from statistics import fmean, NormalDist


@dataclass(frozen=True)
class PowerAssumptions:
    layout_count: int
    replicates: int
    control_probability: float
    treatment_probability: float
    layout_logit_sd: float
    paired_noise_fraction: float
    alpha: float = 0.025

    def validate(self) -> None:
        if self.layout_count < 4 or self.replicates < 1:
            raise ValueError("power analysis requires at least four layouts and one replicate")
        if not 0 < self.control_probability < 1 or not 0 < self.treatment_probability < 1:
            raise ValueError("completion probabilities must be strictly between zero and one")
        if self.layout_logit_sd < 0 or not 0 <= self.paired_noise_fraction <= 1:
            raise ValueError("invalid variance or pairing assumption")
        if not 0 < self.alpha < 1:
            raise ValueError("alpha must be in (0, 1)")


def _logit(value: float) -> float:
    return log(value / (1 - value))


def _logistic(value: float) -> float:
    return 1 / (1 + exp(-value))


def _sign_flip_p(effects: list[float], draws: int, rng: Random) -> float:
    observed = abs(fmean(effects))
    exceed = 0
    for _ in range(draws):
        statistic = abs(fmean(value * (-1 if rng.getrandbits(1) else 1) for value in effects))
        exceed += statistic >= observed - 1e-12
    return (exceed + 1) / (draws + 1)


def _wilson_interval(successes: int, trials: int,
                     confidence: float = 0.95) -> tuple[float, float]:
    """Wilson interval for the Monte Carlo rejection probability."""
    if not 0 <= successes <= trials or trials <= 0 or not 0 < confidence < 1:
        raise ValueError("invalid binomial interval inputs")
    probability = successes / trials
    z = NormalDist().inv_cdf(0.5 + confidence / 2)
    denominator = 1 + z * z / trials
    center = (probability + z * z / (2 * trials)) / denominator
    radius = z / denominator * sqrt(
        probability * (1 - probability) / trials + z * z / (4 * trials * trials))
    return max(0.0, center - radius), min(1.0, center + radius)


def estimate_power(assumptions: PowerAssumptions, simulations: int = 2000,
                   randomization_draws: int = 1999, seed: int = 20260911) -> dict:
    """Simulation-based prospective power for the planned layout sign-flip test.

    Probabilities are conditional at an average-difficulty layout. A shared
    Gaussian layout effect induces within-layout dependence. The pairing
    fraction controls how often treatment and control use the same residual
    uniform draw, preserving their conditional marginal probabilities.
    """
    assumptions.validate()
    if simulations < 100 or randomization_draws < 199:
        raise ValueError("simulation counts are too small for a stable planning result")
    rng = Random(seed)
    rejected = 0
    observed_control, observed_treatment, observed_effects = [], [], []
    control_intercept = _logit(assumptions.control_probability)
    treatment_intercept = _logit(assumptions.treatment_probability)
    for _ in range(simulations):
        effects = []
        for _ in range(assumptions.layout_count):
            layout_shift = rng.gauss(0, assumptions.layout_logit_sd)
            p_control = _logistic(control_intercept + layout_shift)
            p_treatment = _logistic(treatment_intercept + layout_shift)
            differences = []
            for _ in range(assumptions.replicates):
                control_draw = rng.random()
                treatment_draw = control_draw if rng.random() < assumptions.paired_noise_fraction else rng.random()
                control = control_draw < p_control
                treatment = treatment_draw < p_treatment
                differences.append(float(treatment) - float(control))
                observed_control.append(float(control))
                observed_treatment.append(float(treatment))
            effects.append(fmean(differences))
        observed_effects.append(fmean(effects))
        rejected += _sign_flip_p(effects, randomization_draws, rng) <= assumptions.alpha
    probability = rejected / simulations
    power_interval = _wilson_interval(rejected, simulations)
    result = {
        "schema_version": 1,
        "purpose": "prospective_design_only",
        "assumptions": asdict(assumptions),
        "simulations": simulations,
        "randomization_draws_per_simulation": randomization_draws,
        "seed": seed,
        "estimated_power": probability,
        "monte_carlo_se": sqrt(probability * (1 - probability) / simulations),
        "monte_carlo_power_ci95": list(power_interval),
        "simulated_marginal_control_rate": fmean(observed_control),
        "simulated_marginal_treatment_rate": fmean(observed_treatment),
        "simulated_mean_rate_difference": fmean(observed_effects),
    }
    return result


def estimate_power_grid(
    layout_count: int, replicates: int, control_probabilities: list[float],
    absolute_effect: float, layout_logit_sds: list[float],
    paired_noise_fractions: list[float], alpha: float = 0.025,
    target_power: float = 0.8, simulations: int = 2000,
    randomization_draws: int = 1999, seed: int = 20260911,
) -> dict:
    """Evaluate a frozen smallest effect over a conservative nuisance grid."""
    if not 0 < absolute_effect < 1 or not 0 < target_power < 1:
        raise ValueError("effect and target power must be in (0, 1)")
    if not control_probabilities or not layout_logit_sds or not paired_noise_fractions:
        raise ValueError("power grid dimensions must be nonempty")
    cells = []
    for index, (control, layout_sd, pairing) in enumerate(product(
            control_probabilities, layout_logit_sds, paired_noise_fractions)):
        treatment = control + absolute_effect
        if treatment >= 1:
            raise ValueError("control probability plus effect must be below one")
        result = estimate_power(PowerAssumptions(
            layout_count, replicates, control, treatment, layout_sd, pairing,
            alpha), simulations, randomization_draws, seed + index * 100003)
        cells.append(result)
    worst = min(cells, key=lambda value: value["monte_carlo_power_ci95"][0])
    return {
        "schema_version": 1,
        "purpose": "prospective_nuisance_grid",
        "smallest_effect_of_interest_absolute": absolute_effect,
        "target_power": target_power,
        "base_seed": seed,
        "cell_count": len(cells),
        "cells": cells,
        "minimum_estimated_power": worst["estimated_power"],
        "minimum_power_ci95_lower_bound": worst["monte_carlo_power_ci95"][0],
        "worst_case_assumptions": worst["assumptions"],
        "decision_rule": "every cell's 95% Wilson lower bound meets target power",
        "design_meets_target_in_every_cell": all(
            cell["monte_carlo_power_ci95"][0] >= target_power for cell in cells),
    }


def write_power(result: dict, path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


# Every fourth layout of a design is a neutral control (`layout_index % 4 == 0`).
NEUTRAL_EVERY = 4
# Logits of exactly 0 or 1 are undefined; a stratum rate the pilot measured as
# 0/n is simulated just inside the open interval.
RATE_FLOOR = 0.005


@dataclass(frozen=True)
class StratifiedPowerAssumptions:
    """The design's own structure: neutral controls carry no contrast.

    Neutral layouts give both methods one completion rate, so their true
    difference is zero; the whole average effect must then come from the
    manipulated layouts, whose effect is `average_effect` scaled by the
    manipulated share. Layouts in each stratum share a Gaussian logit shift.
    """
    layout_count: int
    replicates: int
    neutral_rate: float
    manipulated_control_rate: float
    average_effect: float
    layout_logit_sd: float
    paired_noise_fraction: float
    alpha: float = 0.025

    @property
    def neutral_layouts(self) -> int:
        return (self.layout_count + NEUTRAL_EVERY - 1) // NEUTRAL_EVERY

    @property
    def manipulated_effect(self) -> float:
        manipulated = self.layout_count - self.neutral_layouts
        return self.average_effect * self.layout_count / manipulated

    def validate(self) -> None:
        if self.layout_count < 4 or self.replicates < 1:
            raise ValueError("power analysis requires at least four layouts and one replicate")
        if self.manipulated_control_rate + self.manipulated_effect >= 1:
            raise ValueError("the manipulated stratum cannot carry the average effect")
        if self.layout_logit_sd < 0 or not 0 <= self.paired_noise_fraction <= 1:
            raise ValueError("invalid variance or pairing assumption")


def estimate_stratified_power(assumptions: StratifiedPowerAssumptions,
                              simulations: int = 2000, randomization_draws: int = 1999,
                              seed: int = 20261030) -> dict:
    """Power of the planned layout sign-flip test under the stratified model."""
    assumptions.validate()
    rng = Random(seed)
    clip = lambda value: min(1 - RATE_FLOOR, max(RATE_FLOOR, value))
    neutral = _logit(clip(assumptions.neutral_rate))
    control = _logit(clip(assumptions.manipulated_control_rate))
    treatment = _logit(clip(assumptions.manipulated_control_rate
                            + assumptions.manipulated_effect))
    rejected, means = 0, []
    for _ in range(simulations):
        effects = []
        for layout in range(assumptions.layout_count):
            shift = rng.gauss(0, assumptions.layout_logit_sd)
            if layout % NEUTRAL_EVERY == 0:
                p_control = p_treatment = _logistic(neutral + shift)
            else:
                p_control = _logistic(control + shift)
                p_treatment = _logistic(treatment + shift)
            differences = []
            for _ in range(assumptions.replicates):
                control_draw = rng.random()
                treatment_draw = (control_draw if rng.random() < assumptions.paired_noise_fraction
                                  else rng.random())
                differences.append(float(treatment_draw < p_treatment)
                                   - float(control_draw < p_control))
            effects.append(fmean(differences))
        means.append(fmean(effects))
        rejected += _sign_flip_p(effects, randomization_draws, rng) <= assumptions.alpha
    probability = rejected / simulations
    return {
        "assumptions": {**asdict(assumptions),
                        "manipulated_effect": assumptions.manipulated_effect},
        "estimated_power": probability,
        "monte_carlo_power_ci95": list(_wilson_interval(rejected, simulations)),
        "simulated_mean_effect": fmean(means),
        "simulations": simulations,
        "seed": seed,
    }


def estimate_stratified_power_grid(
    layout_count: int, replicates: int, neutral_rates: list[float],
    manipulated_control_rates: list[float], average_effect: float,
    layout_logit_sds: list[float], paired_noise_fractions: list[float],
    alpha: float = 0.025, target_power: float = 0.8, simulations: int = 2000,
    randomization_draws: int = 1999, seed: int = 20261030,
) -> dict:
    cells = []
    for index, (neutral, control, layout_sd, pairing) in enumerate(product(
            neutral_rates, manipulated_control_rates, layout_logit_sds,
            paired_noise_fractions)):
        cells.append(estimate_stratified_power(StratifiedPowerAssumptions(
            layout_count, replicates, neutral, control, average_effect, layout_sd,
            pairing, alpha), simulations, randomization_draws, seed + index * 100003))
    worst = min(cells, key=lambda value: value["monte_carlo_power_ci95"][0])
    return {
        "purpose": "prospective_stratified_nuisance_grid",
        "smallest_effect_of_interest_absolute": average_effect,
        "target_power": target_power,
        "cells": cells,
        "minimum_power_ci95_lower_bound": worst["monte_carlo_power_ci95"][0],
        "worst_case_assumptions": worst["assumptions"],
        "design_meets_target_in_every_cell": all(
            cell["monte_carlo_power_ci95"][0] >= target_power for cell in cells),
    }

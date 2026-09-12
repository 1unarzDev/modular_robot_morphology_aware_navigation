from __future__ import annotations

from dataclasses import asdict, dataclass
from itertools import product
import json
from math import exp, log, sqrt
from pathlib import Path
from random import Random
from statistics import fmean


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
    result = {
        "schema_version": 1,
        "purpose": "prospective_design_only",
        "assumptions": asdict(assumptions),
        "simulations": simulations,
        "randomization_draws_per_simulation": randomization_draws,
        "seed": seed,
        "estimated_power": probability,
        "monte_carlo_se": sqrt(probability * (1 - probability) / simulations),
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
    worst = min(cells, key=lambda value: value["estimated_power"])
    return {
        "schema_version": 1,
        "purpose": "prospective_nuisance_grid",
        "smallest_effect_of_interest_absolute": absolute_effect,
        "target_power": target_power,
        "base_seed": seed,
        "cell_count": len(cells),
        "cells": cells,
        "minimum_estimated_power": worst["estimated_power"],
        "worst_case_assumptions": worst["assumptions"],
        "design_meets_target_in_every_cell": all(
            cell["estimated_power"] >= target_power for cell in cells),
    }


def write_power(result: dict, path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path

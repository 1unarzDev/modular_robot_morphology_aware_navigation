from __future__ import annotations

from collections import defaultdict
from math import exp, log, pi, sqrt
from random import Random
from statistics import fmean
from typing import Iterable

import numpy as np

from .records import TrialRecord


def _logistic(value: float) -> float:
    if value >= 0:
        inverse = exp(-value)
        return 1.0 / (1.0 + inverse)
    forward = exp(value)
    return forward / (1.0 + forward)


def _logsumexp(values: np.ndarray) -> float:
    maximum = float(np.max(values))
    return maximum + log(float(np.exp(values - maximum).sum()))


def _nelder_mead(objective, initial: np.ndarray, max_iterations: int = 250) -> tuple[np.ndarray, bool]:
    """Small bounded-free optimizer used to avoid a SciPy runtime dependency."""
    size = len(initial)
    simplex = [initial.copy()]
    for index in range(size):
        point = initial.copy()
        point[index] += 0.25
        simplex.append(point)
    values = [objective(point) for point in simplex]
    for _ in range(max_iterations):
        order = np.argsort(values)
        simplex = [simplex[index] for index in order]
        values = [values[index] for index in order]
        if np.max(np.abs(np.asarray(simplex[1:]) - simplex[0])) < 1e-4:
            return simplex[0], True
        centroid = np.mean(simplex[:-1], axis=0)
        reflected = centroid + (centroid - simplex[-1])
        reflected_value = objective(reflected)
        if values[0] <= reflected_value < values[-2]:
            simplex[-1], values[-1] = reflected, reflected_value
            continue
        if reflected_value < values[0]:
            expanded = centroid + 2.0 * (reflected - centroid)
            expanded_value = objective(expanded)
            if expanded_value < reflected_value:
                simplex[-1], values[-1] = expanded, expanded_value
            else:
                simplex[-1], values[-1] = reflected, reflected_value
            continue
        contracted = centroid + 0.5 * (simplex[-1] - centroid)
        contracted_value = objective(contracted)
        if contracted_value < values[-1]:
            simplex[-1], values[-1] = contracted, contracted_value
            continue
        best = simplex[0]
        simplex = [best] + [best + 0.5 * (point - best) for point in simplex[1:]]
        values = [values[0]] + [objective(point) for point in simplex[1:]]
    return simplex[int(np.argmin(values))], False


def _fit(records: list[TrialRecord], treatment: str, control: str,
         families: frozenset[str] | None, quadrature_points: int) -> dict:
    selected = [record for record in records
                if record.spec.method in (treatment, control)
                and (families is None or record.spec.family in families)]
    family_names = sorted({record.spec.family for record in selected})
    baseline_family = family_names[0]
    family_columns = family_names[1:]
    layouts: dict[tuple[str, str], list[tuple[int, np.ndarray]]] = defaultdict(list)
    for record in selected:
        covariates = np.asarray([
            1.0,
            float(record.spec.method == treatment),
            *(float(record.spec.family == family) for family in family_columns),
        ])
        layouts[(record.spec.family, record.spec.layout_id)].append(
            (int(record.completed), covariates))
    nodes, weights = np.polynomial.hermite.hermgauss(quadrature_points)
    log_weights = np.log(weights) - 0.5 * log(pi)

    def objective(parameters: np.ndarray) -> float:
        beta, log_sd = parameters[:-1], float(parameters[-1])
        if np.max(np.abs(beta)) > 12 or not -7 <= log_sd <= 3:
            return 1e12 + float(np.dot(parameters, parameters))
        shifts = sqrt(2.0) * exp(log_sd) * nodes
        likelihood = 0.0
        for observations in layouts.values():
            conditional = []
            for shift, log_weight in zip(shifts, log_weights):
                value = log_weight
                for outcome, covariates in observations:
                    probability = min(1 - 1e-12, max(1e-12, _logistic(float(beta @ covariates) + shift)))
                    value += log(probability if outcome else 1 - probability)
                conditional.append(value)
            likelihood += _logsumexp(np.asarray(conditional))
        return -likelihood

    initial = np.zeros(3 + len(family_columns))
    initial[0] = log((sum(r.completed for r in selected) + 0.5) /
                     (len(selected) - sum(r.completed for r in selected) + 0.5))
    initial[-1] = log(0.5)
    parameters, converged = _nelder_mead(objective, initial)
    beta, random_sd = parameters[:-1], exp(float(parameters[-1]))

    def standardized(method_value: float) -> float:
        probabilities = []
        for family, _ in sorted(layouts):
            covariates = np.asarray([
                1.0, method_value,
                *(float(family == candidate) for candidate in family_columns),
            ])
            probabilities.append(sum(
                weight / sqrt(pi) * _logistic(float(beta @ covariates) + sqrt(2) * random_sd * node)
                for node, weight in zip(nodes, weights)))
        return fmean(probabilities)

    control_probability = standardized(0.0)
    treatment_probability = standardized(1.0)
    return {
        "converged": converged,
        "log_likelihood": -objective(parameters),
        "method_log_odds_coefficient": float(beta[1]),
        "layout_random_intercept_sd": random_sd,
        "standardized_control_probability": control_probability,
        "standardized_treatment_probability": treatment_probability,
        "marginal_probability_difference": treatment_probability - control_probability,
        "family_reference": baseline_family,
        "quadrature_points": quadrature_points,
        "layout_count": len(layouts),
    }


def hierarchical_logistic_contrast(
    records: list[TrialRecord], treatment: str, control: str,
    families: frozenset[str] | None, bootstrap_draws: int = 400,
    quadrature_points: int = 15, seed: int = 12000,
) -> dict:
    """Fit the predeclared logistic random-intercept sensitivity analysis."""
    if bootstrap_draws < 40 or quadrature_points < 5:
        raise ValueError("hierarchical analysis settings are too small")
    fitted = _fit(records, treatment, control, families, quadrature_points)
    by_family: dict[str, dict[str, list[TrialRecord]]] = defaultdict(lambda: defaultdict(list))
    for record in records:
        if families is None or record.spec.family in families:
            by_family[record.spec.family][record.spec.layout_id].append(record)
    rng, estimates, failed, nonconverged = Random(seed), [], 0, 0
    for _ in range(bootstrap_draws):
        sampled = []
        for layouts in by_family.values():
            ids = sorted(layouts)
            for sample_index in range(len(ids)):
                chosen = rng.choice(ids)
                # Relabel duplicate clusters so each sampled copy gets its own intercept.
                for record in layouts[chosen]:
                    sampled.append((record, f"bootstrap_{sample_index}_{chosen}"))
        # _fit keys layouts from specs, so materialize lightweight proxy records with unique layout ids.
        proxies = []
        from dataclasses import replace
        for record, layout_id in sampled:
            proxies.append(replace(record, spec=replace(record.spec, layout_id=layout_id)))
        try:
            estimate = _fit(proxies, treatment, control, families, quadrature_points)
            estimates.append(estimate["marginal_probability_difference"])
            if not estimate["converged"]:
                nonconverged += 1
        except (ArithmeticError, ValueError):
            failed += 1
    if len(estimates) < max(20, int(0.8 * bootstrap_draws)):
        raise RuntimeError("too many hierarchical bootstrap fits failed")
    estimates.sort()
    fitted.update({
        "outcome": "completion",
        "treatment": treatment,
        "control": control,
        "families": sorted(families) if families else "all",
        "model": "binomial_logit_with_gaussian_layout_random_intercept",
        "ci95_layout_bootstrap": [
            estimates[int(0.025 * (len(estimates) - 1))],
            estimates[int(0.975 * (len(estimates) - 1))],
        ],
        "bootstrap_draws_requested": bootstrap_draws,
        "bootstrap_draws_completed": len(estimates),
        "bootstrap_draws_converged": len(estimates) - nonconverged,
        "bootstrap_draws_nonconverged": nonconverged,
        "bootstrap_draws_failed": failed,
        "inference_role": "predeclared_sensitivity_analysis",
    })
    return fitted

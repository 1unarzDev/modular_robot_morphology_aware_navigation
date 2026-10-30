from math import exp, log
from random import Random

import pytest

from modular_robot_benchmarks.design import TrialSpec
from modular_robot_benchmarks.pilot_nuisance import (
    adverse_values, estimate_scope, paired_noise_fraction)
from modular_robot_benchmarks.records import TrialManifest, TrialRecord


def _logistic(value):
    return 1.0 / (1.0 + exp(-value))


def _simulated(layouts, replicates, control_rate, treatment_rate, layout_sd, pairing, seed):
    """Records drawn from exactly the generative model `power.estimate_power` assumes."""
    rng = Random(seed)
    control_logit = log(control_rate / (1 - control_rate))
    treatment_logit = log(treatment_rate / (1 - treatment_rate))
    records = []
    for family in ("docking_observability", "combined_constraints"):
        for index in range(layouts):
            shift = rng.gauss(0, layout_sd)
            for replicate in range(replicates):
                control_draw = rng.random()
                treatment_draw = control_draw if rng.random() < pairing else rng.random()
                for method, draw, logit in (
                        ("feasibility_coupled", control_draw, control_logit),
                        ("sensing_feasibility_coupled", treatment_draw, treatment_logit)):
                    completed = draw < _logistic(logit + shift)
                    spec = TrialSpec(f"{family}-{index:02d}-r{replicate}-{method}", family,
                                     f"{family}-{index:02d}", replicate, method, 0, 1, 2, 3, 4)
                    records.append(TrialRecord(
                        schema_version=1, spec=spec,
                        manifest=TrialManifest("abc", "cfg", "design", {}, {}),
                        terminal_status="completed" if completed else "docking_failure",
                        completed=completed, simulated_duration_s=40.0, wall_duration_s=200.0,
                        final_execution_state="READY" if completed else "RECOVERY_REQUIRED",
                        unrecovered_fault=not completed,
                        infrastructure_attempts=[{"attempt": 1}]))
    return records


def test_the_estimator_recovers_the_parameters_the_power_model_simulates():
    records = _simulated(150, 2, 0.40, 0.65, 0.8, 0.5, seed=3)
    scope = estimate_scope(records, "sensing_feasibility_coupled", "feasibility_coupled",
                           frozenset({"docking_observability", "combined_constraints"}))
    assert scope["fit_converged"]
    assert scope["control_probability_at_average_layout"] == pytest.approx(0.40, abs=0.07)
    assert scope["layout_logit_sd"] == pytest.approx(0.8, abs=0.35)
    assert scope["paired_noise_fraction"]["estimate"] == pytest.approx(0.5, abs=0.2)


def test_pairing_is_zero_for_independent_draws_and_one_for_shared_ones():
    for pairing, expected in ((0.0, 0.0), (1.0, 1.0)):
        records = _simulated(150, 2, 0.5, 0.5 + 0.18, 0.0, pairing, seed=5)
        scope = estimate_scope(records, "sensing_feasibility_coupled", "feasibility_coupled",
                               frozenset({"docking_observability", "combined_constraints"}))
        assert scope["paired_noise_fraction"]["estimate"] == pytest.approx(expected, abs=0.15)


def test_moment_estimator_inverts_known_agreement():
    # p_c = p_t = 0.5 with no layout spread: shared draws always agree,
    # independent draws agree half the time, so 75% agreement means rho = 0.5.
    blocks = [(1, 1)] * 3 + [(0, 1)] + [(0, 0)] * 3 + [(1, 0)]
    result = paired_noise_fraction(blocks, [0.0], [0.0], 0.0)
    assert result["observed_agreement"] == pytest.approx(0.75)
    assert result["estimate"] == pytest.approx(0.5)


def test_adverse_values_move_every_nuisance_parameter_the_unfavourable_way():
    grid = adverse_values(0.30, 30, 100, 0.6, 0.5)
    assert grid["control_rates"][0] == pytest.approx(0.30)
    assert 0.30 < grid["control_rates"][1] < 0.5          # toward 0.5
    assert grid["layout_logit_sds"] == [0.6, 1.2]
    assert grid["paired_noise_fractions"] == [0.25, 0.5]
    capped = adverse_values(0.95, 95, 100, 0.1, 0.0)
    assert capped["control_rate_capped_for_effect"]
    assert max(capped["control_rates"]) <= 1 - 0.18 - 0.01 + 1e-9
    assert capped["layout_logit_sds"] == [0.1, 1.0]   # the declared floor
    assert capped["paired_noise_fractions"] == [0.0]

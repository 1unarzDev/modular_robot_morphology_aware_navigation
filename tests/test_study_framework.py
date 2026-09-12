from dataclasses import replace

import pytest

from modular_robot_benchmarks.analysis import analyze, validate_records, write_artifacts
from modular_robot_benchmarks.design import METHODS, StudyDesign, generate_design
from modular_robot_benchmarks.power import PowerAssumptions, estimate_power
from modular_robot_benchmarks.records import TrialManifest, TrialRecord, TrialStore


def _records(design, configuration_hash="frozen"):
    records = []
    for spec in design.trials:
        completed = spec.method == "sensing_feasibility_coupled" or spec.layout_id.endswith("-00")
        records.append(TrialRecord(
            schema_version=1, spec=spec,
            manifest=TrialManifest("abc123", configuration_hash, design.design_hash, {"ros": "jazzy"}, {}),
            terminal_status="completed" if completed else "docking_failure",
            completed=completed, simulated_duration_s=42.0 if completed else 30.0,
            wall_duration_s=2.0, mechanical_work_j=80.0,
            final_execution_state="READY" if completed else "RECOVERY_REQUIRED",
            unrecovered_fault=not completed, predicted_transition_probabilities=[0.8],
            observed_transition_outcomes=[int(completed)],
            planned_route_signature=f"route-{spec.method}",
            transition_edge_decisions=[{
                "transition_id": "compact_to_narrow",
                "feasible": spec.method == "sensing_feasibility_coupled",
                "reasons": [] if spec.method == "sensing_feasibility_coupled"
                else ["connector_not_visible"],
            }],
        ))
    return records


def test_confirmatory_design_is_balanced_paired_and_randomized():
    design = generate_design()
    assert design.expected_trials == 432
    for family in design.families:
        assert sum(spec.family == family for spec in design.trials) == 144
    block = [spec for spec in design.trials
             if spec.layout_id == "reconfiguration_workspace-00" and spec.replicate == 0]
    assert len(block) == 4 and {spec.method for spec in block} == set(METHODS)
    assert len({(s.world_seed, s.sensing_seed, s.friction_seed, s.fault_seed) for s in block}) == 1
    assert len({spec.method_order for spec in block}) == 4


def test_frozen_design_round_trip_is_hash_checked_and_immutable(tmp_path):
    design = generate_design(layouts_per_family=2, replicates=1)
    path = design.write_frozen(tmp_path / "design.json")
    assert StudyDesign.read_frozen(path) == design
    design.write_frozen(path)
    with pytest.raises(FileExistsError, match="overwrite"):
        generate_design(layouts_per_family=3, replicates=1).write_frozen(path)
    value = path.read_text().replace(design.design_hash, "0" * 64, 1)
    path.write_text(value)
    with pytest.raises(ValueError, match="hash"):
        StudyDesign.read_frozen(path)


def test_trial_store_is_append_only_and_resumable(tmp_path):
    design = generate_design(layouts_per_family=1, replicates=1)
    record = _records(design)[0]
    store = TrialStore(tmp_path / "raw")
    store.write_terminal(record); store.write_terminal(record)
    assert store.contains(record.spec.trial_id)
    assert len(store.pending(design.trials)) == design.expected_trials - 1
    with pytest.raises(FileExistsError):
        store.write_terminal(replace(record, terminal_status="timeout", completed=False,
                                     final_execution_state="RECOVERY_REQUIRED", unrecovered_fault=True))


def test_validation_rejects_incomplete_and_manifest_mismatched_trials():
    design = generate_design(layouts_per_family=1, replicates=1)
    records = _records(design)
    with pytest.raises(ValueError, match="incomplete design"):
        validate_records(records[:-1], design)
    records[-1] = replace(records[-1], manifest=replace(records[-1].manifest, configuration_hash="changed"))
    with pytest.raises(ValueError, match="manifest"):
        validate_records(records, design)


def test_analysis_retains_failures_and_computes_paired_contrasts():
    design = generate_design(layouts_per_family=2, replicates=2)
    result = analyze(_records(design), design, bootstrap_draws=200, permutation_draws=500)
    assert result["trial_count"] == 48
    assert result["failure_taxonomy"]["docking_failure"] > 0
    full = next(row for row in result["methods"] if row["method"] == "sensing_feasibility_coupled")
    assert full["completion_rate"] == 1.0
    assert full["deadline_penalized_time_s"] == 42.0
    assert len(result["primary_contrasts"]) == 2
    assert len(result["secondary_contrasts"]) == 2
    assert result["primary_contrasts"][0]["layout_count"] == 6
    assert result["primary_contrasts"][0]["permutation_method"] == "exact_sign_flip"
    assert result["primary_contrasts"][0]["assignments"] == 64
    assert result["primary_contrasts"][0]["monte_carlo_standard_error"] is None
    assert result["primary_contrasts"][0]["minimum_attainable_two_sided_p"] == 2 / 64
    assert result["transition_brier_score"] is not None
    assert result["transition_rejection_reasons_by_method"]["geometry_coupled"][
        "connector_not_visible"] > 0
    assert result["route_decision_disagreement_vs_full"]["geometry_coupled"][
        "different_route_or_transition_fraction"] == 1.0


def test_large_contrast_reports_randomization_monte_carlo_error():
    design = generate_design(layouts_per_family=8, replicates=1)
    result = analyze(_records(design), design, bootstrap_draws=100, permutation_draws=200)
    contrast = result["primary_contrasts"][0]
    assert contrast["permutation_method"] == "monte_carlo_sign_flip_200_draws"
    assert contrast["assignments"] == 200
    assert contrast["monte_carlo_standard_error"] >= 0.0
    assert contrast["minimum_attainable_two_sided_p"] is None


def test_analysis_writes_reviewable_tables_and_dependency_free_figures(tmp_path):
    design = generate_design(layouts_per_family=2, replicates=2)
    result = analyze(_records(design), design, bootstrap_draws=100, permutation_draws=200)
    write_artifacts(result, tmp_path)
    assert (tmp_path / "results.md").is_file()
    assert (tmp_path / "secondary_contrasts.csv").is_file()
    assert "<svg" in (tmp_path / "figures" / "completion.svg").read_text()
    assert (tmp_path / "figures" / "primary_effects.svg").is_file()


def test_prospective_power_simulation_is_deterministic_and_labeled():
    assumptions = PowerAssumptions(8, 2, 0.5, 0.8, 0.5, 0.7)
    first = estimate_power(assumptions, simulations=100, randomization_draws=199, seed=42)
    second = estimate_power(assumptions, simulations=100, randomization_draws=199, seed=42)
    assert first == second
    assert first["purpose"] == "prospective_design_only"
    assert 0 <= first["estimated_power"] <= 1


def test_success_requires_safe_terminal_topology():
    design = generate_design(layouts_per_family=1, replicates=1)
    record = _records(design)[0]
    with pytest.raises(ValueError, match="completion requires"):
        replace(record, final_execution_state="RECOVERY_REQUIRED").validate()


def test_transition_decision_records_require_auditable_fields():
    design = generate_design(layouts_per_family=1, replicates=1)
    record = _records(design)[0]
    with pytest.raises(ValueError, match="audit fields"):
        replace(record, transition_edge_decisions=[{"feasible": False}]).validate()

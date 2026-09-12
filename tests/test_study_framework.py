from dataclasses import replace

import pytest

from modular_robot_benchmarks.analysis import analyze, validate_records
from modular_robot_benchmarks.design import METHODS, generate_design
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
    assert result["transition_brier_score"] is not None


def test_success_requires_safe_terminal_topology():
    design = generate_design(layouts_per_family=1, replicates=1)
    record = _records(design)[0]
    with pytest.raises(ValueError, match="completion requires"):
        replace(record, final_execution_state="RECOVERY_REQUIRED").validate()

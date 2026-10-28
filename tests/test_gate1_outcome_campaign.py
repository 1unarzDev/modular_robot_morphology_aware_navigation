from pathlib import Path
from xml.etree import ElementTree

import pytest

from modular_robot_benchmarks.design import (
    GATE1_OUTCOME_CASES, GATE1_OUTCOMES_DESIGN_KIND, StudyDesign,
    generate_gate1_outcome_campaign, outcome_case_for_run,
)
from modular_robot_benchmarks.mission_batch import outcome_case_for, unperceived_obstacles
from modular_robot_benchmarks.sdf_export import export_confirmatory_sdf

ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "src/modular_robot_description/config/morphologies.yaml"


def test_every_declared_class_is_covered_with_independent_seeds():
    design = generate_gate1_outcome_campaign("combined_constraints", 0, "feasibility_coupled")
    assert design.design_kind == GATE1_OUTCOMES_DESIGN_KIND
    assert design.expected_trials == 2 * len(GATE1_OUTCOME_CASES)
    expected = {case.expected_status for case in GATE1_OUTCOME_CASES}
    assert expected == {"completed", "timeout", "localization_lost", "collision",
                        "unsafe_topology"}
    assert [outcome_case_for(design, spec).name for spec in design.trials[:5]] == [
        case.name for case in GATE1_OUTCOME_CASES]
    assert len({spec.friction_seed for spec in design.trials}) == design.expected_trials
    assert len({spec.world_seed for spec in design.trials}) == 1
    assert all(spec.trial_id.endswith(outcome_case_for_run(spec.replicate).name)
               for spec in design.trials)


def test_a_frozen_gate1_design_round_trips(tmp_path):
    design = generate_gate1_outcome_campaign("combined_constraints", 0, "feasibility_coupled")
    path = design.write_frozen(tmp_path / "gate1.json")
    assert StudyDesign.read_frozen(path).design_hash == design.design_hash


def test_other_designs_carry_no_outcome_stimulus():
    from modular_robot_benchmarks.design import generate_fault_matrix_design

    design = generate_fault_matrix_design("combined_constraints", 0, "feasibility_coupled")
    assert outcome_case_for(design, design.trials[0]) is None
    assert unperceived_obstacles(None, 1.0, 2.0) == ()


def test_unperceived_obstacle_sits_on_the_ground_ahead_of_the_start():
    case = next(case for case in GATE1_OUTCOME_CASES if case.name == "collision")
    ((name, center, size),) = unperceived_obstacles(case, 0.65, 1.75)
    assert center == pytest.approx((1.65, 1.75, 0.05))
    assert size == (0.30, 0.30, 0.10)
    # Below the lidar plane (core spawns at 0.18 m, lidar 0.12 m above it).
    assert center[2] + size[2] / 2.0 < 0.18


def test_unperceived_obstacle_is_in_the_world_but_not_the_autonomy_priors(tmp_path):
    from modular_robot_benchmarks.design import generate_gate1_outcome_campaign

    spec = generate_gate1_outcome_campaign(
        "combined_constraints", 0, "feasibility_coupled").trials[3]
    box = ("gate1_unperceived_obstacle", (1.65, 1.75, 0.05), (0.3, 0.3, 0.1))
    world, manifest = export_confirmatory_sdf(
        "combined_constraints", 0, spec.world_seed, tmp_path / "world.sdf",
        CATALOG, spec.friction_seed, (box,))
    models = {model.get("name") for model in ElementTree.parse(world).iter("model")}
    assert "gate1_unperceived_obstacle" in models
    assert "gate1_unperceived_obstacle" not in manifest.read_text(encoding="utf-8")
    plain, _ = export_confirmatory_sdf(
        "combined_constraints", 0, spec.world_seed, tmp_path / "plain" / "world.sdf",
        CATALOG, spec.friction_seed)
    assert "gate1_unperceived_obstacle" not in plain.read_text(encoding="utf-8")


def test_gate1_audit_reports_whether_each_stimulus_produced_its_class():
    batch = (ROOT / "src/modular_robot_benchmarks/modular_robot_benchmarks/"
             "gate1_audit.py").read_text(encoding="utf-8")
    assert "stimulus_produced_expected_class" in batch

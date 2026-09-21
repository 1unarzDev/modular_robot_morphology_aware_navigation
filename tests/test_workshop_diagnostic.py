import json
from pathlib import Path

from modular_robot_benchmarks.confirmatory_scenarios import make_confirmatory_scenario
from modular_robot_benchmarks.design import (
    WORKSHOP_METHODS, WORKSHOP_VARIANTS, StudyDesign, generate_workshop_design,
)
from morphology_planner.transition_validation import load_environment_boxes


ROOT = Path(__file__).resolve().parents[1]


def test_workshop_design_pairs_variants_methods_and_seeds(tmp_path):
    design = generate_workshop_design()
    assert design.expected_trials == 18
    assert set(design.methods) == set(WORKSHOP_METHODS)
    assert len({spec.world_seed for spec in design.trials}) == 1
    for replicate in (0, 1):
        seeds = {spec.friction_seed for spec in design.trials if spec.replicate == replicate}
        assert len(seeds) == 1
    assert StudyDesign.read_frozen(design.write_frozen(tmp_path / "design.json")) == design


def test_workshop_variants_share_geometry_and_differ_only_in_post():
    seed = generate_workshop_design().trials[0].world_seed
    scenarios = {variant: make_confirmatory_scenario(variant, 0, seed) for variant in WORKSHOP_VARIANTS}
    grids = {tuple(scenario.grid.data) for scenario in scenarios.values()}
    assert len(grids) == 1
    assert not scenarios["workshop_neutral"].transition_obstacles
    a = scenarios["workshop_blocked_a"].transition_obstacles[0]
    b = scenarios["workshop_blocked_b"].transition_obstacles[0]
    site_x, site_y = scenarios["workshop_neutral"].parameters["attractive_site_xy"]
    assert a.center[1] > site_y + 0.35 and b.center[1] < site_y - 0.35
    assert a.center[0] > site_x and b.center[0] < site_x


def test_planner_loads_static_transition_environment_from_manifest(tmp_path):
    seed = generate_workshop_design().trials[0].world_seed
    scenario = make_confirmatory_scenario("workshop_blocked_a", 0, seed)
    manifest = tmp_path / "world.manifest.json"
    manifest.write_text(json.dumps(scenario.manifest()), encoding="utf-8")
    boxes = load_environment_boxes(str(manifest))
    assert boxes == scenario.transition_obstacles
    assert load_environment_boxes("") == ()
    node = (ROOT / "src/morphology_planner/morphology_planner/ros_node.py").read_text()
    assert "environment=self._environment" in node
    launch = (ROOT / "src/modular_robot_bringup/launch/demo.launch.py").read_text()
    assert '"transition_environment": ParameterValue(transition_environment, value_type=str)' in launch
    batch = (ROOT / "src/modular_robot_benchmarks/modular_robot_benchmarks/mission_batch.py").read_text()
    assert "transition_environment:={scenario_manifest.resolve()}" in batch

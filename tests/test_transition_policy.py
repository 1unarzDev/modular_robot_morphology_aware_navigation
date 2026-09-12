from pathlib import Path

from morphology_planner import (
    CoupledTransitionPolicy,
    HybridState,
    OccupancyGrid,
    PodSensingState,
    build_transition_trajectories,
    load_catalog,
    make_method_planner,
    plan_signature,
)
from morphology_planner.transition_validation import Box3, Pose3


CATALOG = Path(__file__).parents[1] / "src/modular_robot_description/config/morphologies.yaml"


def _catalog_and_transition():
    catalog = load_catalog(CATALOG).supported_experiment_subset()
    transition = next(t for t in catalog.transitions if t.id == "compact_to_narrow")
    return catalog, transition


def test_catalog_transition_expands_to_sequential_world_trajectories():
    catalog, transition = _catalog_and_transition()
    trajectories = build_transition_trajectories(
        catalog, transition, Pose3(2.0, 3.0, 0.0, 0.0))
    assert [trajectory.module_id for trajectory in trajectories] == list(transition.moved_pods)
    assert all(trajectory.points[0].time_s == 0.0 for trajectory in trajectories)
    assert all(trajectory.points[-1].time_s == transition.time for trajectory in trajectories)
    assert trajectories[0].points[-1].pose.x == 2.71
    # pod_2 moves second and therefore holds its source pose through the first slot.
    assert trajectories[1].points[1].time_s == transition.time / len(transition.moved_pods)
    assert trajectories[1].points[0].pose == trajectories[1].points[1].pose


def test_world_trajectory_rotates_catalog_path_with_robot_heading():
    catalog, transition = _catalog_and_transition()
    trajectories = build_transition_trajectories(
        catalog, transition, Pose3(2.0, 3.0, 0.0, 1.5707963267948966))
    final = trajectories[0].points[-1].pose
    assert abs(final.x - 1.90) < 1e-9
    assert abs(final.y - 3.71) < 1e-9


def test_feasibility_policy_rejects_3d_overhang_that_geometry_policy_ignores():
    catalog, transition = _catalog_and_transition()
    grid = OccupancyGrid(50, 50, 0.1, origin_x=-2.5, origin_y=-2.5)
    state = HybridState(25, 25, 0, "compact_diff")
    # It intersects pod_0's relocation path but has no ground-plane occupancy.
    overhang = Box3("overhang", (0.50, 0.18, 0.13), (0.18, 0.25, 0.08))
    geometry = CoupledTransitionPolicy("geometry_coupled", catalog, grid, 4,
                                       environment=(overhang,))
    feasibility = CoupledTransitionPolicy("feasibility_coupled", catalog, grid, 4,
                                          environment=(overhang,))
    assert geometry(transition, state)
    assert not feasibility(transition, state)
    assert any("environment_collision" in reason for reason in feasibility.decisions[-1].reasons)
    assert feasibility.decisions[-1].checked_samples > 0


def test_sensing_ablation_changes_only_observability_checks():
    catalog, transition = _catalog_and_transition()
    grid = OccupancyGrid(50, 50, 0.1, origin_x=-2.5, origin_y=-2.5)
    state = HybridState(25, 25, 0, "compact_diff")
    poor = {pod: PodSensingState(False, 0.1) for pod in transition.moved_pods}
    feasibility = CoupledTransitionPolicy("feasibility_coupled", catalog, grid, 4,
                                          sensing=poor)
    sensing = CoupledTransitionPolicy("sensing_feasibility_coupled", catalog, grid, 4,
                                      sensing=poor)
    assert feasibility(transition, state)
    assert not sensing(transition, state)
    reasons = set(sensing.decisions[-1].reasons)
    assert "pod_0:connector_not_visible" in reasons
    assert "pod_0:covariance_too_large" in reasons


def test_all_frozen_methods_use_an_explicit_transition_policy():
    catalog, _ = _catalog_and_transition()
    grid = OccupancyGrid(50, 50, 0.1, origin_x=-2.5, origin_y=-2.5)
    methods = (
        "route_first_adaptation", "geometry_coupled", "feasibility_coupled",
        "sensing_feasibility_coupled",
    )
    instances = [make_method_planner(method, catalog, grid, heading_bins=4)
                 for method in methods]
    assert [instance.method for instance in instances] == list(methods)
    assert instances[0].transition_policy.method == "sensing_feasibility_coupled"
    assert instances[1].transition_policy.method == "geometry_coupled"


def test_plan_and_transition_audits_are_stable_record_inputs():
    catalog, transition = _catalog_and_transition()
    grid = OccupancyGrid(50, 50, 0.1, origin_x=-2.5, origin_y=-2.5)
    instance = make_method_planner("geometry_coupled", catalog, grid, heading_bins=4)
    state = HybridState(25, 25, 0, "compact_diff")
    assert instance.transition_policy(transition, state)
    record = instance.transition_decision_records()[0]
    assert record["transition_id"] == transition.id
    assert record["state"]["morphology"] == "compact_diff"
    plan = instance.plan(state, (26, 25))
    assert plan_signature(plan) == plan_signature(plan)

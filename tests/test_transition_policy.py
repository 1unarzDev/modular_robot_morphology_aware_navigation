from dataclasses import replace
from pathlib import Path

from morphology_planner import (
    CoupledTransitionPolicy,
    HybridState,
    OccupancyGrid,
    PodSensingState,
    build_transition_trajectories,
    build_verification_trajectories,
    load_catalog,
    make_method_planner,
    plan_signature,
)
from morphology_planner.catalog import PostTransitionVerification
from morphology_planner.transition_validation import (
    Box3, Pose3, TransitionTrajectoryValidator,
)


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
    assert abs(final.x - 1.80) < 1e-9
    assert abs(final.y - 3.71) < 1e-9


def test_feasibility_policy_rejects_3d_overhang_that_geometry_policy_ignores():
    catalog, transition = _catalog_and_transition()
    grid = OccupancyGrid(50, 50, 0.1, origin_x=-2.5, origin_y=-2.5)
    state = HybridState(25, 25, 0, "compact_diff")
    # It intersects pod_0's relocation path but has no ground-plane occupancy.
    overhang = Box3("overhang", (0.50, 0.20, 0.13), (0.18, 0.25, 0.08))
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


def test_verification_maneuver_expands_to_a_rigid_body_sweep():
    catalog, _ = _catalog_and_transition()
    verification = catalog.post_transition_verification
    trajectories = build_verification_trajectories(
        catalog, "narrow_tandem", Pose3(2.0, 3.0, 0.0, 0.0), verification)
    # The whole assembly moves together: the core travels with every pod.
    assert {t.module_id for t in trajectories} == {"core", *catalog.morphologies["narrow_tandem"].pod_poses}
    assert all(t.points[0].time_s == 0.0 for t in trajectories)
    assert all(abs(t.points[-1].time_s - verification.duration) < 1e-6 for t in trajectories)
    # Yaw is accumulated unwrapped so interpolation never crosses +/-pi.
    yaws = [point.pose.yaw for point in trajectories[0].points]
    assert max(yaws) > 0.37 and abs(yaws[-1]) < 1e-6
    # The declared maneuver returns the body to where it started.
    first, last = trajectories[0].points[0].pose, trajectories[0].points[-1].pose
    assert abs(first.x - last.x) < 1e-6 and abs(first.y - last.y) < 1e-6


def test_empty_verification_declaration_reduces_to_relocation_feasibility():
    catalog, transition = _catalog_and_transition()
    assert build_verification_trajectories(
        catalog, "narrow_tandem", Pose3(0.0, 0.0, 0.0, 0.0),
        PostTransitionVerification()) == ()


def test_feasibility_policy_rejects_a_site_only_the_verification_maneuver_reaches():
    """A site can host the transformation and still not host what follows.

    The post lies outside every pod's relocation sweep but inside the arc
    pod_0 traces when the committed narrow_tandem body yaws in place, which
    is the failure observed in the workshop Blocked-A mission.
    """
    catalog, transition = _catalog_and_transition()
    grid = OccupancyGrid(50, 50, 0.1, origin_x=-2.5, origin_y=-2.5)
    state = HybridState(25, 25, 0, "compact_diff")
    post = Box3("post", (0.60, 0.47, 0.06), (0.10, 0.10, 0.12))
    geometry = CoupledTransitionPolicy("geometry_coupled", catalog, grid, 4,
                                       environment=(post,))
    feasibility = CoupledTransitionPolicy("feasibility_coupled", catalog, grid, 4,
                                          environment=(post,))
    assert geometry(transition, state)
    assert not feasibility(transition, state)
    assert feasibility.decisions[-1].reasons == ("pod_0:post_transition_collision",)
    # Relocation alone clears the post; only the maneuver that follows does not.
    relocation_only = CoupledTransitionPolicy(
        "feasibility_coupled",
        replace(catalog, post_transition_verification=PostTransitionVerification()),
        grid, 4, environment=(post,))
    assert relocation_only(transition, state)


def test_verification_check_creates_no_rejection_without_an_obstruction():
    catalog, transition = _catalog_and_transition()
    grid = OccupancyGrid(50, 50, 0.1, origin_x=-2.5, origin_y=-2.5)
    state = HybridState(25, 25, 0, "compact_diff")
    feasibility = CoupledTransitionPolicy("feasibility_coupled", catalog, grid, 4)
    assert feasibility(transition, state)
    assert not any("post_transition" in reason
                   for decision in feasibility.decisions
                   for reason in decision.reasons)


def test_clearance_disk_already_covers_the_verification_maneuver():
    """The occupancy grid guards the maneuver through the planar disk check.

    `_verification_reasons` only sweeps declared 3D transition obstacles,
    which the 2D map cannot represent. Walls are covered instead by the
    planar clearance disk, but only while the disk is at least as large as
    the maneuver's reach -- so a catalog edit must not shrink it below that.
    """
    catalog = load_catalog(CATALOG).supported_experiment_subset()
    validator = TransitionTrajectoryValidator()
    verification = catalog.post_transition_verification
    for transition in catalog.transitions:
        swept = validator.swept_boxes(build_verification_trajectories(
            catalog, transition.target, Pose3(0.0, 0.0, 0.0, 0.0), verification))
        reach = max(max(abs(box.center[0]) + box.size[0] / 2.0,
                        abs(box.center[1]) + box.size[1] / 2.0) for box in swept)
        assert reach <= transition.swept_radius, transition.id

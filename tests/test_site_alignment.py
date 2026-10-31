from math import cos, pi, sin

import pytest

from modular_robot_bringup.qualification import (
    PlanarPose, RotationStallWatch, site_alignment_command)


def _align(start: PlanarPose, site: PlanarPose, budget_s: float = 30.0):
    """Integrate a unicycle under the navigator's site-alignment command."""
    pose, translating, dt = start, True, 0.05
    for _ in range(int(budget_s / dt)):
        linear, angular, translating, aligned = site_alignment_command(
            pose, site, 0.03, 0.05, translating)
        if aligned:
            return pose
        pose = PlanarPose(pose.x + linear * cos(pose.yaw) * dt,
                          pose.y + linear * sin(pose.yaw) * dt,
                          pose.yaw + angular * dt)
    return None


# Fault-matrix trial 03-r06: the follower stopped 0.10 m short facing the door,
# and the transformation ran there instead of at the planned -45 degree site,
# sweeping pod_4 into the shelf.
@pytest.mark.parametrize("start, site", [
    (PlanarPose(1.352, 1.756, 0.014), PlanarPose(1.45, 1.75, -pi / 4)),
    (PlanarPose(1.55, 1.75, 0.0), PlanarPose(1.45, 1.75, 0.0)),
    (PlanarPose(1.45, 1.75, 0.0), PlanarPose(1.45, 1.75, pi / 2)),
])
def test_site_alignment_reaches_planned_pose_within_tolerance(start, site):
    final = _align(start, site)
    assert final is not None
    assert ((final.x - site.x) ** 2 + (final.y - site.y) ** 2) ** 0.5 <= 0.06
    assert abs((final.yaw - site.yaw + pi) % (2 * pi) - pi) <= 0.05


def test_site_alignment_reverses_to_a_site_behind_instead_of_turning_around():
    linear, angular, _, aligned = site_alignment_command(
        PlanarPose(1.55, 1.75, 0.0), PlanarPose(1.45, 1.75, 0.0), 0.03, 0.05, True)
    assert not aligned and linear < 0.0 and abs(angular) < 1e-9


def test_bearing_branch_floors_its_turn_so_it_can_break_away():
    # 1.5 * 0.16 = 0.24 rad/s, below the 0.30 rad/s the executor uses for a pod
    # in-place turn. The branch commands a pure turn, so flooring it is safe.
    linear, angular, _, aligned = site_alignment_command(
        PlanarPose(0.0, 0.0, 0.0), PlanarPose(1.0, 0.16, 0.0), 0.03, 0.05, True)
    assert not aligned and linear == 0.0
    assert angular >= 0.30
    _, negative, _, _ = site_alignment_command(
        PlanarPose(0.0, 0.0, 0.0), PlanarPose(1.0, -0.16, 0.0), 0.03, 0.05, True)
    assert negative <= -0.30


def test_the_floor_does_not_steer_a_translating_body():
    # Inside the 0.15 rad gate the command translates, and the executor's own
    # comment warns against applying a breakaway floor while translating.
    linear, angular, _, aligned = site_alignment_command(
        PlanarPose(0.0, 0.0, 0.0), PlanarPose(1.0, 0.10, 0.0), 0.03, 0.05, True)
    assert not aligned and linear > 0.0
    assert 0.0 < angular < 0.30


def test_final_yaw_branch_keeps_its_lower_floor():
    # Already at the site position, trimming yaw inside the tolerance rather
    # than breaking away, so this branch is unchanged at 0.15 rad/s.
    linear, angular, _, aligned = site_alignment_command(
        PlanarPose(1.45, 1.75, 0.0), PlanarPose(1.45, 1.75, 0.06), 0.03, 0.05, True)
    assert not aligned and linear == 0.0
    assert abs(angular - 0.15) < 1e-9


def test_rotation_stall_watch_fires_only_when_the_body_is_not_turning():
    watch = RotationStallWatch(stall_s=5.0, progress_rad=0.02)
    # The recorded failure: a constant command, no yaw change at all.
    assert not watch.stalled(0.0, -1.571, True)
    assert not watch.stalled(4.9, -1.571, True)
    assert watch.stalled(5.1, -1.571, True)
    assert watch.elapsed_s >= 5.0


def test_rotation_stall_watch_rearms_on_progress_and_on_translation():
    watch = RotationStallWatch(stall_s=5.0, progress_rad=0.02)
    watch.stalled(0.0, 0.0, True)
    # A slow but real turn keeps re-arming, so a long turn is not a stall.
    for step in range(1, 40):
        assert not watch.stalled(step * 1.0, step * 0.03, True)
    # Translation clears the watch entirely.
    watch.stalled(100.0, 1.2, True)
    assert not watch.stalled(200.0, 1.2, False)
    assert watch.started_s is None
    assert not watch.stalled(201.0, 1.2, True)


def test_shortest_angle_wraps_to_the_nearer_direction():
    from math import pi

    from modular_robot_bringup.qualification import shortest_angle
    assert abs(shortest_angle(3 * pi / 2) + pi / 2) < 1e-9
    assert abs(shortest_angle(-3 * pi / 2) - pi / 2) < 1e-9
    assert abs(shortest_angle(0.02) - 0.02) < 1e-12


def test_a_path_splits_at_each_planned_vertex():
    """ADR 0014: the planner turns in place at vertices; pure pursuit cut them."""
    from modular_robot_bringup.qualification import path_pieces

    diagonal, east = 0.7853981633974483, 0.0
    poses = [PlanarPose(2.55, 1.35, diagonal), PlanarPose(2.65, 1.45, diagonal),
             PlanarPose(2.75, 1.55, diagonal), PlanarPose(2.85, 1.55, east),
             PlanarPose(2.95, 1.55, east)]
    assert path_pieces(poses) == [(0, 2, east), (2, 4, None)]


def test_straight_paths_are_one_piece_and_turns_in_place_merge():
    from modular_robot_bringup.qualification import path_pieces

    straight = [PlanarPose(x / 10, 0.0, 0.0) for x in range(5)]
    assert path_pieces(straight) == [(0, 4, None)]
    north, west = pi / 2, pi
    turning = [PlanarPose(0.0, 0.0, 0.0), PlanarPose(0.1, 0.0, 0.0),
               PlanarPose(0.1, 0.0, north), PlanarPose(0.1, 0.0, west),
               PlanarPose(0.0, 0.0, west)]
    # Two quarter turns at (0.1, 0) are one stop, turning to west.
    assert path_pieces(turning) == [(0, 1, west), (3, 4, None)]


def test_the_navigator_executes_narrow_vertices_in_place():
    """ADR 0014: binds the navigator to path_pieces for the narrow controller."""
    from pathlib import Path

    source = (Path(__file__).resolve().parents[1] / "src/modular_robot_bringup/"
              "modular_robot_bringup/hybrid_navigator.py").read_text(encoding="utf-8")
    assert 'self.declare_parameter("vertex_turn_controllers", ["FollowPathNarrow"])' in source
    assert "success = await self._traverse(segment)" in source
    assert "for start, end, turn_to in path_pieces(planar):" in source
    assert "await self._align_to_site(vertex)" in source

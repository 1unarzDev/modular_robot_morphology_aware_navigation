from math import cos, pi, sin

import pytest

from modular_robot_bringup.qualification import PlanarPose, site_alignment_command


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

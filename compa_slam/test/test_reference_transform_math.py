import math
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))

from reference_transform_math import (  # noqa: E402
    Pose2D,
    anchor_route_at_pose,
    compose,
    inverse,
    rotate_vector,
    wrap_angle,
)


class ReferenceTransformMathTest(unittest.TestCase):
    def assert_pose_close(self, actual, expected, tolerance=1.0e-9):
        self.assertTrue(math.isclose(actual.x, expected.x, abs_tol=tolerance))
        self.assertTrue(math.isclose(actual.y, expected.y, abs_tol=tolerance))
        self.assertTrue(
            math.isclose(
                wrap_angle(actual.yaw - expected.yaw),
                0.0,
                abs_tol=tolerance,
            )
        )

    def test_transform_and_inverse_cancel(self):
        transform = Pose2D(2.5, -1.2, 0.7)
        identity = compose(transform, inverse(transform))
        self.assert_pose_close(identity, Pose2D(0.0, 0.0, 0.0))

    def test_map_correction_moves_goal_without_moving_local_pose(self):
        # A fixed goal at x=10 in map initially appears at x=8 in odom.
        odom_from_map = Pose2D(-2.0, 0.0, 0.0)
        map_goal = Pose2D(10.0, 0.0, 0.0)
        self.assert_pose_close(
            compose(odom_from_map, map_goal), Pose2D(8.0, 0.0, 0.0)
        )

        # RTAB-Map later estimates one additional metre of local drift. The EKF
        # pose remains continuous; only the goal expressed in odom shifts.
        corrected_odom_from_map = Pose2D(-3.0, 0.0, 0.0)
        self.assert_pose_close(
            compose(corrected_odom_from_map, map_goal),
            Pose2D(7.0, 0.0, 0.0),
        )

    def test_start_relative_route_is_anchored_at_current_map_pose(self):
        map_base = Pose2D(4.0, -3.0, math.pi / 2.0)
        first_route_reference = Pose2D(0.0, 0.0, 0.0)
        map_from_route = anchor_route_at_pose(map_base, first_route_reference)

        self.assert_pose_close(
            compose(map_from_route, first_route_reference), map_base
        )
        self.assert_pose_close(
            compose(map_from_route, Pose2D(0.0, 2.0, 0.0)),
            Pose2D(2.0, -3.0, math.pi / 2.0),
        )

    def test_velocity_is_rotated_but_not_translated(self):
        vx, vy = rotate_vector(math.pi / 2.0, 1.0, 0.0)
        self.assertTrue(math.isclose(vx, 0.0, abs_tol=1.0e-9))
        self.assertTrue(math.isclose(vy, 1.0, abs_tol=1.0e-9))


if __name__ == "__main__":
    unittest.main()

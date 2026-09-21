"""Independent route compatibility and controller kinematics checks."""
import ast
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("waypoint_sim", PACKAGE/"scripts/run_waypoint_sim.py")
SIM = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SIM)
CFG = yaml.safe_load((PACKAGE/"config/simple_waypoint_sim.yaml").read_text())
SOURCE = PACKAGE.parent/"reference_trajectory/reference_trajectory/waypoint_traj_simple.py"


def test_exact_original_points_speed_and_schedule():
    """Execute only the original pure trajectory class, without importing ROS."""
    if not SOURCE.exists():
        pytest.skip("Original reference_trajectory source is not next to this standalone package")
    tree = ast.parse(SOURCE.read_text())
    original_class = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "WaypointTraj")
    namespace = {"np": np, "math": math}
    exec(compile(ast.Module(body=[original_class], type_ignores=[]), str(SOURCE), "exec"), namespace)
    trajectory_node = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "TrajectoryNode")
    initializer = next(node for node in trajectory_node.body if isinstance(node, ast.FunctionDef) and node.name == "__init__")
    assignments = [node for node in initializer.body if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "waypoints" for target in node.targets)]
    assert len(assignments) == 1
    original_points = ast.literal_eval(assignments[0].value.args[0])
    assert CFG["points_m"] == [point[:2] for point in original_points]
    assert all(point[2] == CFG["fixed_turret_yaw_rad"] for point in original_points)
    speed_calls = [node for node in ast.walk(trajectory_node) if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "declare_parameter" and node.args and isinstance(node.args[0], ast.Constant) and node.args[0].value == "v_lin"]
    assert len(speed_calls) == 1
    assert CFG["speed_m_s"] == ast.literal_eval(speed_calls[0].args[1]) == .25
    source = namespace["WaypointTraj"](original_points, v_lin=.25, w_yaw=.5)
    implementation = SIM.PolylineReference(CFG["points_m"], .25)
    assert implementation.duration == source.total_time == 52.
    assert implementation.length == 13.
    times = list(np.linspace(0, 60., 123))
    for boundary in source.t_start:
        times.extend([max(0., boundary-1e-6), boundary, boundary+1e-6])
    for time_s in sorted(times):
        x, y, yaw, vx, vy, yaw_rate = source.update(time_s)
        actual = implementation.sample(time_s)
        np.testing.assert_allclose(actual[:4], [x, y, vx, vy], atol=1e-12)
        assert yaw == yaw_rate == 0.
    catalog = yaml.safe_load((PACKAGE.parent/"reference_trajectory/config/trajectories/waypoint_traj_simple.yaml").read_text())
    assert catalog["points_m"] == CFG["points_m"]


@pytest.mark.parametrize("yaw", [-math.pi, -1.2, 0., .7, math.pi])
def test_offset_inverse_matches_forward_drive(yaw):
    """Wheel commands reproduce an XY base velocity at the measured axle offset."""
    desired = np.array([.23, -.18])
    radius, a, b = .1075, .33072, .27114
    left, right, omega = SIM.inverse_drive(*desired, yaw, radius, a, b)
    v = radius*(left+right)/2
    yaw_rate = radius*(right-left)/(2*a)
    c, s = math.cos(yaw), math.sin(yaw)
    reconstructed = np.array([c*v-s*b*yaw_rate, s*v+c*b*yaw_rate])
    np.testing.assert_allclose(reconstructed, desired, atol=1e-12)
    assert omega == pytest.approx(yaw_rate)


def test_original_route_cannot_pass_without_complete_run():
    route = SIM.PolylineReference(CFG["points_m"], .25)
    report = SIM.summarize([], route, False)
    assert not report["passed"]
    assert "Trajectory and final hold did not complete" in report["failures"]


@pytest.mark.parametrize("age", [0., .02, .04, .054, .10])
def test_continuous_prediction_matches_reference_clock_for_delayed_feedback(age):
    """A held constant-velocity pose must not invent an error after 40 ms.

    The 54 ms case reproduces the stale-state mechanism seen in Gazebo, where
    a 40 ms cap introduced about 2.2 mm of false forward error before recovery.
    """
    current_pose = np.array([1.2, -2.3, .7])
    velocity = np.array([.15856, -.03, .2])
    held_pose = current_pose-velocity*age
    predicted = SIM.predict_control_pose(*held_pose, velocity, age, True, continuous=True)
    np.testing.assert_allclose(predicted[:3], current_pose, atol=1e-14)
    assert predicted[3] == age
    if age == .054:
        old = SIM.predict_control_pose(*held_pose, velocity, age, True)
        assert current_pose[0]-old[0] == pytest.approx(.00221984)


@pytest.mark.parametrize("continuous", [False, True])
def test_prediction_disabled_or_future_stamp_preserves_measured_pose(continuous):
    pose, velocity = (1., -2., .3), (.25, .1, -.2)
    assert SIM.predict_control_pose(*pose, velocity, .054, False, continuous) == (*pose, 0.)
    assert SIM.predict_control_pose(*pose, velocity, -.005, True, continuous) == (*pose, 0.)


def test_existing_profiles_keep_their_prediction_horizon():
    assert SIM.predict_control_pose(0., 0., 0., (1., 2., 3.), .054, True) == (
        .04, .08, .12, .04)

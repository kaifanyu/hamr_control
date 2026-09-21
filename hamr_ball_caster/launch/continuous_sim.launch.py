"""Run the full vehicle and the shared 2 cm continuous planner once."""

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def _boolean(value, name):
    if value.lower() not in ("true", "false"):
        raise ValueError(f"{name} must be true or false")
    return value.lower() == "true"


def command_for_run(*, config, output, domain_id, gui, record):
    """One harness owns Gazebo, tracking, recording and child-process cleanup."""
    domain = int(domain_id)
    if not 0 <= domain <= 232:
        raise ValueError("domain_id must be in [0, 232]")
    configuration = Path(config).expanduser().resolve()
    if not configuration.is_file():
        raise ValueError(f"Missing configuration: {configuration}")
    destination = Path(output).expanduser().resolve()
    if destination.exists() and (not destination.is_dir() or any(destination.iterdir())):
        raise ValueError(f"output_dir must be new or empty: {destination}")
    executable = "record_waypoint_run.py" if record else "evaluate_waypoint_profile.py"
    command = ["ros2", "run", "hamr_ball_caster", executable,
               "--profile", "continuous", "--config", str(configuration),
               "--output-dir", str(destination), "--domain-id", str(domain)]
    if gui:
        command.append("--gui")
    return command


def _start(context):
    values = {name: LaunchConfiguration(name).perform(context) for name in
              ("config", "output_dir", "domain_id", "gui", "record")}
    values["gui"] = _boolean(values["gui"], "gui")
    values["record"] = _boolean(values["record"], "record")
    values["output"] = values.pop("output_dir")
    command = command_for_run(**values)
    # Recording cleanup may drain ffmpeg for 90 seconds, in addition to
    # stopping the trajectory and simulator. Let the owning harness finish.
    process = ExecuteProcess(cmd=command, output="screen",
                             sigterm_timeout="180" if values["record"] else "45",
                             sigkill_timeout="10")
    return [
        LogInfo(msg=f"Continuous full-vehicle run; results: {values['output']}"),
        RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=_finished,
        )),
        process,
    ]


def _finished(event, context):
    if event.returncode:
        raise RuntimeError(f"Trajectory harness failed ({event.returncode}); inspect output logs and run_summary.json")
    return [EmitEvent(event=Shutdown(reason="Trajectory harness passed; results saved"))]


def generate_launch_description():
    share = Path(get_package_share_directory("hamr_ball_caster"))
    output = Path.home() / "Videos" / "hamr_sim" / datetime.now().strftime("continuous_tight_%Y%m%d_%H%M%S_%f")
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Open Gazebo (WSLg supported)"),
        DeclareLaunchArgument("record", default_value="false", description="Save an MP4; runs physics at 0.15x wall time"),
        DeclareLaunchArgument("config", default_value=str(share / "config/continuous_tight_waypoint_sim.yaml")),
        DeclareLaunchArgument("output_dir", default_value=str(output), description="New or empty result directory"),
        DeclareLaunchArgument("domain_id", default_value="88", description="Isolated local simulation ROS domain"),
        OpaqueFunction(function=_start),
    ])

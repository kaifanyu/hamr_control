"""Run and optionally record one isolated caster trajectory, then clean up."""
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def _finished(event, context):
    if event.returncode:
        raise RuntimeError(f"Swivel caster run failed ({event.returncode}); inspect run_summary.json")
    return [EmitEvent(event=Shutdown(reason="Swivel caster run complete"))]


def _start(context):
    values = {name: LaunchConfiguration(name).perform(context) for name in
              ("gui", "record", "config", "output_dir", "domain_id", "profile", "initial_yaw_deg")}
    command = ["ros2", "run", "hamr_swivel_caster", "run_swivel_caster.py",
               "--output-dir", values["output_dir"], "--domain-id", values["domain_id"],
               "--profile", values["profile"], "--initial-yaw-deg", values["initial_yaw_deg"]]
    if values["config"]:
        command.extend(("--config", values["config"]))
    for option in ("gui", "record"):
        if values[option].lower() not in ("true", "false"):
            raise ValueError(f"{option} must be true or false")
        if values[option].lower() == "true":
            command.append("--" + option)
    process = ExecuteProcess(cmd=command, output="screen", sigterm_timeout="210", sigkill_timeout="10")
    return [RegisterEventHandler(OnProcessExit(target_action=process, on_exit=_finished)), process]


def generate_launch_description():
    share = Path(get_package_share_directory("hamr_swivel_caster"))
    output = Path.home() / "Videos/hamr_sim" / datetime.now().strftime("swivel_caster_%Y%m%d_%H%M%S_%f")
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("record", default_value="false", description="Save overview and caster closeup MP4s"),
        DeclareLaunchArgument("config", default_value="", description="Optional override for the selected profile"),
        DeclareLaunchArgument("profile", default_value="smooth", description="smooth or legacy (original simple trajectory)"),
        DeclareLaunchArgument("initial_yaw_deg", default_value="0", description="Initial chassis world heading, degrees"),
        DeclareLaunchArgument("output_dir", default_value=str(output)),
        DeclareLaunchArgument("domain_id", default_value="89"),
        OpaqueFunction(function=_start),
    ])

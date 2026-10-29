"""Check nested launch parameters without starting hardware."""

from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def test_hardware_include_keeps_serial_and_foxglove_configuration_separate(monkeypatch):
    from launch import LaunchContext
    from launch.launch_description_sources import AnyLaunchDescriptionSource
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare
    from launch_ros.utilities import evaluate_parameters

    package = SCRIPTS.parent
    monkeypatch.setattr(FindPackageShare, "find", lambda self, name: str(package.parent / name))
    context = LaunchContext()
    context.launch_configurations.update({
        "bridge_config": "/operator/serial_bridge.yaml", "run_foxglove": "true",
        "record_bag": "false", "record_camera": "false",
    })
    description = AnyLaunchDescriptionSource(
        str(package / "launch/hamr_HW.launch.xml")).get_launch_description(context)
    parameters = {}

    def visit(entity):
        if isinstance(entity, Node):
            if entity.condition is None or entity.condition.evaluate(context):
                executable = perform_substitutions(context, normalize_to_list_of_substitutions(entity.node_executable))
                parameters[executable] = evaluate_parameters(context, entity._Node__parameters)
            return  # Only evaluate launch substitutions; never start hardware.
        for child in entity.visit(context) or []:
            visit(child)

    visit(description)
    assert Path(parameters["relay_node"][0]) == Path("/operator/serial_bridge.yaml")
    assert Path(parameters["foxglove_bridge"][0]) == package / "config/foxglove_bridge.yaml"
    assert parameters["hamr_topic_monitor"][0]["config_file"] == str(package / "config/foxglove_monitor.yaml")
    assert "caster_encoder_bridge" not in parameters

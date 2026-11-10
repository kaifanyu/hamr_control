"""Keep encoder turret estimates distinct from measured Vicon ground truth."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import rclpy

from hamr_odometry.holonomic_odom_node import HolonomicOdomNode


@pytest.mark.parametrize('output_topic', [None, '/custom_onboard/turret_odom'])
def test_encoder_turret_publisher_has_a_distinct_configurable_topic(output_topic):
    args = []
    if output_topic is not None:
        args = ['--ros-args', '-p', f'turret_odom_topic:={output_topic}']
    rclpy.init(args=args)
    node = None
    try:
        node = HolonomicOdomNode()
        assert node.pub_turret_odom.topic_name == (
            output_topic or '/wheel_turret/odom')
        assert node.pub_turret_odom.topic_name != '/HAMR_turret/odom'
        assert node.pub_odom.topic_name == '/wheel_odom'
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_hardware_launch_selects_wheel_and_controller_turret_sources_separately():
    repository = Path(__file__).resolve().parents[2]
    root = ET.parse(repository / 'hamr_bringup/launch/hamr_HW.launch.xml').getroot()
    assert root.find('arg[@name="wheel_turret_odom_topic"]').get('default') == '/wheel_turret/odom'
    assert root.find('arg[@name="controller_turret_odom_topic"]').get('default') == '/HAMR_turret/odom'
    wheel = root.find('node[@exec="holonomic_odom_node"]')
    assert wheel.find('param[@name="turret_odom_topic"]').get('value') == '$(var wheel_turret_odom_topic)'
    controller = root.find('node[@exec="hamr_controller"]')
    assert controller.find('remap[@from="HAMR_turret/odom"]').get('to') == '$(var controller_turret_odom_topic)'

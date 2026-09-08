import rclpy
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)

from hamr_odometry.holonomic_odom_node import HolonomicOdomNode, VICON_ODOM_QOS


def test_vicon_odometry_subscription_is_best_effort_latest_only():
    assert VICON_ODOM_QOS.history == QoSHistoryPolicy.KEEP_LAST
    assert VICON_ODOM_QOS.depth == 1
    assert VICON_ODOM_QOS.reliability == QoSReliabilityPolicy.BEST_EFFORT
    assert VICON_ODOM_QOS.durability == QoSDurabilityPolicy.VOLATILE


def test_node_applies_vicon_qos_to_base_odometry_subscription():
    rclpy.init()
    node = None
    try:
        node = HolonomicOdomNode()
        qos = node.base_odom_sub.qos_profile
        assert node.base_odom_sub.topic_name == '/HAMR_base/odom'
        assert qos.history == QoSHistoryPolicy.KEEP_LAST
        assert qos.depth == 1
        assert qos.reliability == QoSReliabilityPolicy.BEST_EFFORT
        assert qos.durability == QoSDurabilityPolicy.VOLATILE
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

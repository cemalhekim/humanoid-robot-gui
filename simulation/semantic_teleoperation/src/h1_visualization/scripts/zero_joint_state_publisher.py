#!/usr/bin/env python3

import sys
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState


class ZeroJointStatePublisher(Node):
    def __init__(self, urdf_path):
        super().__init__("zero_joint_state_publisher")
        self.joint_names = self._load_joint_names(urdf_path)
        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_joint_states)
        self.get_logger().info(f"Publishing zero positions for {len(self.joint_names)} joints")

    @staticmethod
    def _load_joint_names(urdf_path):
        root = ET.parse(urdf_path).getroot()
        return [
            joint.get("name")
            for joint in root.findall("joint")
            if joint.get("type") != "fixed" and joint.get("name")
        ]

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [0.0] * len(self.joint_names)
        self.publisher.publish(msg)


def main():
    args = remove_ros_args(sys.argv)[1:]
    if len(args) != 1:
        print("Usage: zero_joint_state_publisher.py <urdf_path>", file=sys.stderr)
        raise SystemExit(2)

    rclpy.init()
    node = ZeroJointStatePublisher(args[0])
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

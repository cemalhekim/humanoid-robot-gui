#!/usr/bin/env python3

import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState


class FakeTrajectoryController(Node):
    def __init__(self, urdf_path):
        super().__init__("h1_fake_trajectory_controller")
        self.joint_names = self._load_joint_names(urdf_path)
        self.positions = {name: 0.0 for name in self.joint_names}
        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_joint_states)
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "h1_fake_controller/follow_joint_trajectory",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
        )
        self.get_logger().info(
            f"Fake FollowJointTrajectory controller ready for {len(self.joint_names)} joints"
        )

    @staticmethod
    def _load_joint_names(urdf_path):
        root = ET.parse(urdf_path).getroot()
        return [
            joint.get("name")
            for joint in root.findall("joint")
            if joint.get("type") != "fixed" and joint.get("name")
        ]

    def goal_callback(self, goal_request):
        unknown = [
            name
            for name in goal_request.trajectory.joint_names
            if name not in self.positions
        ]
        if unknown:
            self.get_logger().error(f"Rejecting trajectory with unknown joints: {unknown}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        joint_names = list(trajectory.joint_names)
        previous_time = 0.0

        for point in trajectory.points:
            target_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            sleep_time = max(0.0, target_time - previous_time)
            if sleep_time:
                time.sleep(sleep_time)
            previous_time = target_time

            for name, position in zip(joint_names, point.positions):
                self.positions[name] = position
            self.publish_joint_states()

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [self.positions[name] for name in self.joint_names]
        self.publisher.publish(msg)


def main():
    args = remove_ros_args(sys.argv)[1:]
    if len(args) != 1:
        print("Usage: fake_trajectory_controller.py <urdf_path>", file=sys.stderr)
        raise SystemExit(2)

    rclpy.init()
    node = FakeTrajectoryController(args[0])
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

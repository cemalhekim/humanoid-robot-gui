import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_h1_description_dir():
    moveit_share = Path(get_package_share_directory("h1_moveit_config"))
    workspace_root = moveit_share.parents[3]
    return str(workspace_root / "src" / "h1_description")


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "h1_description_dir",
                default_value=_default_h1_description_dir(),
                description="Path to src/h1_description.",
            ),
            DeclareLaunchArgument(
                "urdf_name",
                default_value="h1_with_hand.urdf",
                description="URDF file name inside h1_description/urdf.",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )


def _launch_nodes(context):
    h1_description_dir = LaunchConfiguration("h1_description_dir").perform(context)
    urdf_name = LaunchConfiguration("urdf_name").perform(context)
    package_share = get_package_share_directory("h1_moveit_config")

    urdf_path = os.path.join(h1_description_dir, "urdf", urdf_name)
    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description_xml = urdf_file.read()
    mesh_uri = Path(os.path.join(h1_description_dir, "meshes")).as_uri()
    robot_description_xml = robot_description_xml.replace(
        "package://h1_description/meshes",
        mesh_uri,
    )

    srdf_path = os.path.join(package_share, "config", "h1.srdf")
    with open(srdf_path, "r", encoding="utf-8") as srdf_file:
        robot_description_semantic_xml = srdf_file.read()

    robot_description = {"robot_description": robot_description_xml}
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_xml
    }
    robot_description_kinematics = {
        "robot_description_kinematics": _load_yaml(
            os.path.join(package_share, "config", "kinematics.yaml")
        )
    }
    robot_description_planning = {
        "robot_description_planning": _load_yaml(
            os.path.join(package_share, "config", "joint_limits.yaml")
        )
    }
    planning_pipelines = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": _load_yaml(os.path.join(package_share, "config", "ompl_planning.yaml")),
    }
    trajectory_execution = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": False,
    }
    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    moveit_params = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        planning_pipelines,
        _load_yaml(os.path.join(package_share, "config", "moveit_controllers.yaml")),
        trajectory_execution,
        planning_scene_monitor,
    ]

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", "pelvis"],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="h1_moveit_config",
            executable="fake_trajectory_controller.py",
            name="h1_fake_trajectory_controller",
            output="screen",
            arguments=[urdf_path],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=moveit_params,
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(package_share, "rviz", "moveit.rviz")],
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                planning_pipelines,
            ],
        ),
    ]

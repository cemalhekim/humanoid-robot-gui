import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_h1_description_dir():
    visualization_share = Path(get_package_share_directory("h1_visualization"))
    workspace_root = visualization_share.parents[3]
    return str(workspace_root / "src" / "h1_description")


def generate_launch_description():
    default_description_dir = _default_h1_description_dir()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "h1_description_dir",
                default_value=default_description_dir,
                description="Path to src/h1_description.",
            ),
            DeclareLaunchArgument(
                "urdf_name",
                default_value="h1_with_hand.urdf",
                description="URDF file name inside h1_description/urdf.",
            ),
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="Use joint_state_publisher_gui sliders.",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )


def _launch_nodes(context):
    h1_description_dir = LaunchConfiguration("h1_description_dir").perform(context)
    urdf_name = LaunchConfiguration("urdf_name").perform(context)
    use_gui = LaunchConfiguration("use_gui")
    package_share = get_package_share_directory("h1_visualization")
    rviz_config = os.path.join(package_share, "rviz", "h1.rviz")

    description_path = os.path.join(h1_description_dir, "urdf", urdf_name)
    with open(description_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    mesh_uri = Path(os.path.join(h1_description_dir, "meshes")).as_uri()
    robot_description = robot_description.replace(
        "package://h1_description/meshes",
        mesh_uri,
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            parameters=[{"robot_description": robot_description}],
            condition=IfCondition(use_gui),
        ),
        Node(
            package="h1_visualization",
            executable="zero_joint_state_publisher.py",
            name="zero_joint_state_publisher",
            output="screen",
            arguments=[description_path],
            condition=UnlessCondition(use_gui),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ]

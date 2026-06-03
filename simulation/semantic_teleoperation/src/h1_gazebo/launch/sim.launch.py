import os
import re
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_h1_description_dir():
    gazebo_share = Path(get_package_share_directory("h1_gazebo"))
    workspace_root = gazebo_share.parents[3]
    return str(workspace_root / "src" / "h1_description")


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
                default_value="h1.urdf",
                description="URDF file name inside h1_description/urdf.",
            ),
            DeclareLaunchArgument(
                "fixed_base",
                default_value="true",
                description="Attach pelvis to a fixed world link for stable visualization.",
            ),
            DeclareLaunchArgument(
                "paused",
                default_value="true",
                description="Start Gazebo paused.",
            ),
            DeclareLaunchArgument(
                "enable_walk",
                default_value="false",
                description="Start a scripted fixed-base walking animation.",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )


def _as_bool(value):
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _gazebo_robot_description(h1_description_dir, urdf_name, fixed_base):
    urdf_path = os.path.join(h1_description_dir, "urdf", urdf_name)
    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()
    robot_description = re.sub(r"^\s*<\?xml[^>]*\?>\s*", "", robot_description)

    mesh_source = os.path.join(h1_description_dir, "meshes")
    mesh_link = "/tmp/h1_gazebo_meshes"
    if os.path.lexists(mesh_link):
        if os.path.islink(mesh_link) and os.readlink(mesh_link) == mesh_source:
            pass
        else:
            os.unlink(mesh_link)
    if not os.path.exists(mesh_link):
        os.symlink(mesh_source, mesh_link)

    mesh_uri = Path(mesh_link).as_uri()
    robot_description = robot_description.replace(
        "package://h1_description/meshes",
        mesh_uri,
    )

    if fixed_base:
        fixed_world = """
  <link name="world"/>
  <joint name="world_to_pelvis" type="fixed">
    <parent link="world"/>
    <child link="pelvis"/>
    <origin xyz="0 0 1.05" rpy="0 0 0"/>
  </joint>
"""
        robot_description = robot_description.replace(">", ">" + fixed_world, 1)

    return robot_description


def _launch_nodes(context):
    h1_description_dir = LaunchConfiguration("h1_description_dir").perform(context)
    urdf_name = LaunchConfiguration("urdf_name").perform(context)
    fixed_base = _as_bool(LaunchConfiguration("fixed_base").perform(context))
    paused = LaunchConfiguration("paused").perform(context)
    enable_walk = _as_bool(LaunchConfiguration("enable_walk").perform(context))

    package_share = get_package_share_directory("h1_gazebo")
    world_path = os.path.join(package_share, "worlds", "empty.world")
    robot_description = _gazebo_robot_description(
        h1_description_dir,
        urdf_name,
        fixed_base,
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "world": world_path,
            "pause": paused,
        }.items(),
    )

    nodes = [
        gazebo_launch,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_h1",
            output="screen",
            arguments=[
                "-topic",
                "robot_description",
                "-entity",
                "h1",
            ],
        ),
    ]

    if enable_walk:
        nodes.append(
            Node(
                package="h1_gazebo",
                executable="scripted_walk.py",
                name="h1_scripted_walk",
                output="screen",
                arguments=["--model", "h1"],
            )
        )

    return nodes

import os
# from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    # ExecuteProcess,
    TimerAction,
)
# from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    # EnvironmentVariable,
    # PathJoinSubstitution,
)
from launch_ros.actions import Node

import xacro
# from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    controller_package = get_package_share_directory("panda_controller")
    # description_package = get_package_share_directory("panda_description")
    moveit_config_package = get_package_share_directory("panda_moveit_config")
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(controller_package, "launch"), "/robot.launch.py"]
        ),
    )

    _robot_description_xml = xacro.process_file(
        os.path.join(moveit_config_package, "config", "panda.urdf.xacro")
    ).toxml()
    robot_description = {"robot_description": _robot_description_xml}
    _robot_semantic_xml = xacro.process_file(
        os.path.join(moveit_config_package, "config", "panda.srdf")
    ).toxml()
    robot_semantic_description = {
        "robot_description_semantic": _robot_semantic_xml}
    nodes = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package="panda_controller",
                    executable="motion",
                    output="screen",
                    arguments=["--ros-args"],
                    parameters=[
                        robot_description,
                        robot_semantic_description,
                        {"use_sim_time": use_sim_time},
                    ],
                )
            ],
        ),
    ]

    return LaunchDescription(nodes + [gazebo_node])

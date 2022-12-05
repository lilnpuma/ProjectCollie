from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    
    declared_arguments = generate_declared_arguments()
    
    # Get substitution for all arguments
    description_package = LaunchConfiguration("description_package")
    description_filepath = LaunchConfiguration("description_filepath")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gazebo = LaunchConfiguration("use_gazebo")
    
    # Find the path to the panda_description package
    robot_description_package = FindPackageShare(description_package)
    # Generate the robot_description_config
    robot_description_config = Command(
        [
            "xacro",
            " ",
            PathJoinSubstitution([robot_description_package, description_filepath]),
            " ",
            'ros2_control:=',
            use_gazebo,
            
        ]
    )
    # Generate the robot_description parameter
    robot_description = {'robot_description': robot_description_config}
    # Generate the robot_state_publisher_node
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [robot_description, {"use_sim_time": use_sim_time}]
    )
    
    return LaunchDescription(declared_arguments + [robot_state_publisher_node])
    
def generate_declared_arguments() -> List:
    return [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value = 'false',
            description = 'Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'use_gazebo',
            default_value = 'false',
            description = 'Use Gazebo to simulate the robot'
        )
    ]
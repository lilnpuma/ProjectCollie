from os import path
from typing import List

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler
    )
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import (
    OnProcessStart)

def generate_launch_description() -> LaunchDescription:
    
    declared_arguments = generate_declared_arguments()
    
    # Get substitution for all arguments
    description_package = LaunchConfiguration("description_package")
    description_filepath = LaunchConfiguration("description_filepath")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gazebo = LaunchConfiguration("use_gazebo")
    # Find the path to the panda_description package
    panda_description_package = FindPackageShare(description_package)
    # Generate the robot_description_config
    # Call the robot_state_publisher launch file
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [panda_description_package, 'launch', 'robot_state_publisher.launch.py']
                ),
            ),
        launch_arguments = {
            'description_package': description_package,
            'description_filepath': description_filepath,
            'use_sim_time': use_sim_time,
            'use_gazebo': use_gazebo,
            }.items()
    )
    # Rviz node
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        output = 'screen',
        arguments = ['-d', PathJoinSubstitution([panda_description_package, 'rviz', 'view.rviz'])],
        parameters = [{"use_sim_time": use_sim_time}],
    )
    
    
    # Joint state publisher GUI node
    joint_state_publisher_node = Node(
        package = 'joint_state_publisher_gui',
        executable = 'joint_state_publisher_gui',
        output = 'screen',
        parameters = [{"use_sim_time": use_sim_time}],
        condition = UnlessCondition(use_gazebo)
    )
    rviz_node = RegisterEventHandler(
        OnProcessStart(
            target_action = joint_state_publisher_node,
            on_start = [rviz_node]
        ),
    )
    # Generate the launch description
    launch_description = [robot_state_publisher_launch]
    # Generate the nodes
    nodes = [ rviz_node, joint_state_publisher_node]
    # Return the launch description
    return LaunchDescription(declared_arguments + launch_description + nodes)
    
def generate_declared_arguments() -> List:
    return [
        DeclareLaunchArgument(
            'description_package',
            default_value = 'panda_description',
            description = 'Description package to load'
        ),
        DeclareLaunchArgument(
            'description_filepath',
            default_value = path.join('urdf', 'panda.urdf.xacro'),
            description = 'Path to the xacro or urdf to load'
        ),
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
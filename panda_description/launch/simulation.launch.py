from os import path
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess,LogInfo
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart

def generate_launch_description() -> LaunchDescription:
    
    declared_arguments = generate_declared_arguments()
    
    # Get substitution for all arguments
    description_package = LaunchConfiguration("description_package")
    description_filepath = LaunchConfiguration("description_filepath")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gazebo = LaunchConfiguration("use_gazebo")
    x_pos = LaunchConfiguration("x_pos")
    y_pos = LaunchConfiguration("y_pos")
    z_pos = LaunchConfiguration("z_pos")
    # Find the path to the panda_description package
    panda_description_package = FindPackageShare(description_package)
    # Generate the robot_description_config
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
        ),
    )
    
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [panda_description_package, 'launch', 'visualize.launch.py']
                ),
            ),
        launch_arguments = {
            'description_package': description_package,
            'description_filepath': description_filepath,
            'use_sim_time': use_sim_time,
            'use_gazebo': use_gazebo,
            }.items()
    )
    
    spawn_entity = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        output = 'screen',
        arguments = ['-entity', 'panda', '-topic', 'robot_description', '-x', x_pos, '-y', y_pos, '-z', z_pos],
        parameters = [{"use_sim_time": use_sim_time}],
    )
    
    load_joint_state_broadcaster = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['joint_state_broadcaster'],
    )
    
    load_joint_trajectory_controller = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['joint_trajectory_controller'],
    )
    
    load_gripper_trajectory_controller = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['gripper_trajectory_controller'],
    )
    
    launch_description = [
        gazebo, 
        robot_state_publisher_launch]
    nodes = [
        spawn_entity,
        RegisterEventHandler(
            OnProcessStart(
                target_action = spawn_entity,
                on_start = [
                    load_joint_state_broadcaster,
                    load_gripper_trajectory_controller,
                    load_joint_trajectory_controller,
                ]
            )
        )
        ]
    
    
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
            default_value = 'true',
            description = 'Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'use_gazebo',
            default_value = 'true',
            description = 'Use Gazebo to simulate the robot'
        ),
        DeclareLaunchArgument(
            'x_pos',
            default_value = '0.0',
            description = 'X position of the robot'
        ),
        DeclareLaunchArgument(
            'y_pos',
            default_value = '0.0',
            description = 'Y position of the robot'
        ),
        DeclareLaunchArgument(
            'z_pos',
            default_value = '0.0',
            description = 'Z position of the robot'
        )
    ]
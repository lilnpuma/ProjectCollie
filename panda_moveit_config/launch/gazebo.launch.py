import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable,LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node

import xacro
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    
    declared_arguments = generate_declared_arguments()
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")
    
    gz_resource_path_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            '/usr/share/gazebo-11/models/',
            ':',
            str(os.path.dirname(get_package_share_directory('panda_description'))),
        ]
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments = {'use_sim_time': use_sim_time}.items()
        )
    
    moveit_config = (
        MoveItConfigsBuilder("panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl"],
        )
        .to_moveit_configs()
    )
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )
    
    rviz_node = Node(
        package="rviz2",
            executable="rviz2",
            output="log",
            arguments=[
                "--display-config",
                rviz_config,
            ],
            parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
    )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )
    
    spawn_entity = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        arguments = ['-entity', 'panda', '-topic', 'robot_description'],
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
        arguments = ['panda_arm_controller'],
    )
    
    load_gripper_trajectory_controller = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['panda_gripper_controller'],
    )
    # spawn_entity = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=robot_state_publisher,
    #         on_start = [spawn_entity],
    #     )
    # )
    
    launch_description = [
        gz_resource_path_env_var,
        gazebo,
        move_group_node,
        rviz_node,
        robot_state_publisher]
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
    
    
    return LaunchDescription(declared_arguments+launch_description+nodes)
    
    
def generate_declared_arguments() -> List:
    
    return [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                get_package_share_directory("panda_moveit_config"),
                "config",
                "moveit.rviz",
            ),
            description="Path to configuration for RViz2.",
        )
    ]
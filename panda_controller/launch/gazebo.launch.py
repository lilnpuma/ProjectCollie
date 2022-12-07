import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch.event_handlers import OnExecutionComplete
from launch_ros.actions import Node
import xacro
import random

def generate_launch_description():
    
    panda_world_path = get_package_share_directory('panda_world')
    panda_description_path = get_package_share_directory('panda_description')    
    gazebo_ros = get_package_share_directory('gazebo_ros')    
    
    world_file_name = 'kitting.world'
    world_path = os.path.join(panda_world_path, 'worlds', world_file_name)
    
    declared_arguments = generate_declared_arguments(world_path)
    
    
    gz_resource_path_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            '/usr/share/gazebo-11/models/',
            ':',
            str(os.path.dirname(panda_description_path)),
            ':',
            str(os.path.dirname(panda_world_path)),
        ]
    )
  
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    block_gen = LaunchConfiguration('block_gen')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
    
    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items())
    
    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
    
    # Spawn camera
    camera_urdf = xacro.process_file(os.path.join(panda_world_path, 'urdf', 'camera.urdf.xacro'), mapping={'name':'TableCamera'}).toxml()
    camera_urdf = camera_urdf.replace('"', '\\"')
    camera_urdf = '{name: \"TableCamera\", xml: \"'+ camera_urdf +\
                            '\", initial_pose: {position: {x: '+str(0.0)+', y: '+str(-0.5)+', z: 2.75},\
                                orientation: {x: -0.5, y: 0.5, z: 0.5, w: 0.5}}}'
                                
    camera_node = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn_entity','gazebo_msgs/SpawnEntity', camera_urdf],output='log'
    )  
    
    spawner_node = block_spawner_nodes(5)
    spawner_node = TimerAction(period=5.0, actions=spawner_node+[camera_node], condition = IfCondition(block_gen))
    
    launch_node = [
        gz_resource_path_env_var,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        spawner_node
    ]
    
    return LaunchDescription(declared_arguments+launch_node)
    
def block_spawner_nodes(num_blocks):
    # Path to urdf.xacro file
    base_path = os.path.join(get_package_share_directory('panda_world'), 'urdf', 'unit_box.urdf.xacro')
    # Range of x and y coordinates for spawning blocks
    x_range = (-0.40, 0.40)
    y_range = (-0.2, -0.7)
    # List of colors for blocks
    blocks = ['red',
              'green',
              'blue',
            #   'yellow',
            #   'orange'
              ]
    spawn_nodes = []
    # Spawn blocks
    for block in blocks:
        # Process xacro file to urdf with color and block_id
        block_urdf = xacro.process_file(base_path, 
                                        mappings={'color': str(block), 'block_id':"block_"+block}
                                        ).toxml()
        block_urdf = block_urdf.replace('"', '\\"')
        # Spawn num_blocks of each color
        for i in range(0,num_blocks):
            spawn_args = '{name: \"block_'+block + str(i)+'\", xml: \"'+ block_urdf +\
                        '\", initial_pose: {position: {x: '+str(random.uniform(x_range[0], x_range[1]))+', y: '+str(random.uniform(y_range[0], y_range[1]))+', z: 1.25},\
                            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
            spawn_nodes.append(TimerAction(period = 1.5*i, actions = [ExecuteProcess(cmd=['ros2', 'service', 'call', '/spawn_entity','gazebo_msgs/SpawnEntity', spawn_args],output='log')
                    ]))
    return spawn_nodes
    
def generate_declared_arguments(world_path) -> List:
    
    return [
    DeclareLaunchArgument(
            name = "block_gen",
            default_value = "True",
            description = "Whether to generate blocks in the world"),
    DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient'),
    DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'),
    DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator'),
    DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load'),
    ]
    
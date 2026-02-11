import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, ExecuteProcess, 
                            RegisterEventHandler, LogInfo, DeclareLaunchArgument)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('robmob_bringup')
    pkg_cmd = get_package_share_directory('robmob_cmd_law')
    pkg_rrt = get_package_share_directory('rrt_connect_planner')

    sim_arg = DeclareLaunchArgument('simulated', default_value='True')

    acquisition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'acquisition.launch.py')
        ),
        launch_arguments={'simulated': LaunchConfiguration('simulated')}.items()
    )

    auto_explo_node = Node(
        package='robmob_auto_explo',
        executable='auto_explo_node',
        name='auto_explo_node'
    )

    map_output_path = os.path.join(pkg_bringup, 'config', 'map_generee')
    
    save_map_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_output_path],
        output='screen'
    )

    exploitation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'exploitation.launch.py')
        ),
        launch_arguments={
            'map_name': 'map_generee.yaml',
            'simulated': LaunchConfiguration('simulated')
        }.items()
    )

    planner_node = Node(
        package='rrt_connect_planner',
        executable='planner_node',
        parameters=[{'use_sim_time': LaunchConfiguration('simulated')}]
    )

    cmd_law_node = Node(
        package='robmob_cmd_law',
        executable='cmd_law_node',
        parameters=[os.path.join(pkg_cmd, 'config', 'params.yaml'),
                    {'use_sim_time': LaunchConfiguration('simulated')}]
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('simulated'))
    )

    start_acquisition = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gz_launch,
            on_start=[
                LogInfo(msg='Gazebo a démarré, lancement de l\'acquisition...'),
                acquisition_launch
            ]
        )
    )

    return LaunchDescription([
        sim_arg,
        gz_launch,
        planner_node,
        cmd_law_node,
        start_acquisition,
        
        RegisterEventHandler(
            OnProcessExit(
                target_action=auto_explo_node, 
                on_exit=[
                    LogInfo(msg='Exploration terminée détectée. Sauvegarde...'),
                    save_map_cmd,
                    exploitation_launch
                ]
            )
        )
    ])
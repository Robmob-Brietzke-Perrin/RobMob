import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    sim_arg = DeclareLaunchArgument('simulated', default_value='True', description='On Gazebo if True')
    auto_arg = DeclareLaunchArgument('autonomous', default_value='True', description='Either RRT + Cmd Law if True, or teleop if not')
    map_name_arg = DeclareLaunchArgument('map_name', default_value='my_map.yaml', description='name of map file in robmob_bringup/config')

    map_config_path = PathJoinSubstitution([
        get_package_share_directory('robmob_bringup'), 'config', LaunchConfiguration('map_name')
    ])

    pkg_bringup = get_package_share_directory('robmob_bringup')
    pkg_rrt = get_package_share_directory('rrt_connect_planner')
    pkg_cmd = get_package_share_directory('robmob_cmd_law')
    pkg_teleop = get_package_share_directory('robmob_teleop')

    # Gazebo simulation
    gz_launch = IncludeLaunchDescription(
        PathJoinSubstitution([get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_house.launch.py']),
        condition=IfCondition(LaunchConfiguration('simulated'))
    )

    # Initial localisation (algo monte-carlos localization)
    loc_launch = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_bringup, 'launch', 'map_server_amcl.launch.py']),
        launch_arguments={
            'map_file': map_config_path,
            'use_sim_time': LaunchConfiguration('simulated')
        }.items()
    )
    
    # Path Plannification & Trajectory tracking nodes
    planner_node = Node(
        package='rrt_connect_planner',
        executable='planner_node',
        name='planner_node',
        condition=IfCondition(LaunchConfiguration('autonomous')),
        parameters=[{'use_sim_time': LaunchConfiguration('simulated')}]
    )
    cmd_law_node = Node(
        package='robmob_cmd_law',
        executable='cmd_law_node',
        name='cmd_law_node',
        condition=IfCondition(LaunchConfiguration('autonomous')),
        parameters=[{'use_sim_time': LaunchConfiguration('simulated')}]
    )

    # Teleoperation
    joy_launch = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_teleop, 'launch', 'joystick.launch.py']),
        condition=UnlessCondition(LaunchConfiguration('autonomous'))
    )

    # Rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_rrt, 'rviz', 'test_rrt.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('simulated')}]
    )

    return LaunchDescription([
        sim_arg,
        auto_arg,
        map_name_arg,
        gz_launch,
        loc_launch,
        planner_node,
        cmd_law_node,
        joy_launch,
        rviz_node
    ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    sim_arg = DeclareLaunchArgument('simulated', default_value='True')
    pkg_cmd = get_package_share_directory('robmob_cmd_law')

    pkg_bringup = get_package_share_directory('robmob_bringup')
    
    rviz_config_path = PathJoinSubstitution([
      pkg_bringup, 'rviz', 'custom.rviz'
    ])

    carto_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py']),
        launch_arguments={'use_sim_time': LaunchConfiguration('simulated')}.items(),
        condition=IfCondition(LaunchConfiguration('simulated'))
    )

    carto_launch_bis = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('turtlebot4_navigation'), 'launch', 'slam.launch.py']),
        launch_arguments={'use_sim_time': LaunchConfiguration('simulated')}.items(),
        condition=UnlessCondition(LaunchConfiguration('simulated'))
    )

    explore_node = Node(
        package='robmob_auto_explo',
        executable='auto_explo_node',
        name='auto_explo_node',
        parameters=[{'use_sim_time': LaunchConfiguration('simulated')}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('simulated')}],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('simulated'))
    )

    cmd_law_node = Node(
        package='robmob_cmd_law',
        executable='cmd_law_node',
        parameters=[os.path.join(pkg_cmd, 'config', 'params.yaml'),
                    {'use_sim_time': LaunchConfiguration('simulated')}]
    )
    planner_node = Node(
        package='rrt_connect_planner',
        executable='planner_node',
        parameters=[{'use_sim_time': LaunchConfiguration('simulated')}]
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('simulated'))
    )

    return LaunchDescription([
        sim_arg,
        carto_launch,
        carto_launch_bis,
        explore_node,
        cmd_law_node,
        planner_node,
        rviz_node,
        gz_launch
    ])
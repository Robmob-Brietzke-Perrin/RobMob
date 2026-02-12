from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    sim_arg = DeclareLaunchArgument('simulated', default_value='True')
    launch_explo_arg = DeclareLaunchArgument('launch_explo', default_value='True') # pour pouvoir être lancer direct, ou via orchestrator qui instancie déjà ce node

    carto_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py']),
        launch_arguments={'use_sim_time': LaunchConfiguration('simulated')}.items()
    )

    explore_node = Node(
        package='robmob_auto_explo',
        executable='auto_explo_node',
        name='auto_explo_node',
        condition=IfCondition(LaunchConfiguration('launch_explo'))
    )

    return LaunchDescription([
        sim_arg,
        carto_launch,
        explore_node
    ])
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    sim_arg = DeclareLaunchArgument('simulated', default_value='True')

    carto_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py']),
        launch_arguments={'use_sim_time': LaunchConfiguration('simulated')}.items()
    )

    full_turn_node = Node(
        package='robmob_auto_explo',
        executable='full_turn_node',
        name='full_turn_node'
    )

    explore_node = Node(
        package='robmob_auto_explo',
        executable='auto_explo_node',
        name='auto_explo_node'
    )

    return LaunchDescription([
        sim_arg,
        carto_launch,
        explore_node
        # full_turn_node,
        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=full_turn_node,
        #         on_exit=[explore_node]
        #     )
        # )
    ])
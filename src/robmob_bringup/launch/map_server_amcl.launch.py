import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    map_file_arg = DeclareLaunchArgument('map_file')
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True')

    amcl_file = os.path.join(
        get_package_share_directory("robmob_bringup"), "config", "amcl.yaml"
    )

    return LaunchDescription(
        [
            map_file_arg,
            sim_time_arg,
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')},
                            {"yaml_filename": LaunchConfiguration('map_file')}],
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                parameters=[amcl_file],
                output='screen'
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration('use_sim_time')},
                    {"autostart": True},
                    {"node_names": ["amcl", "map_server"]},
                ],
            ),
        ]
    )

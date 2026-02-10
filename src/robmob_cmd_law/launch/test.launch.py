import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Chemins des packages
    pkg_rrt_planner = get_package_share_directory('rrt_connect_planner')
    pkg_bringup = get_package_share_directory('robmob_bringup')

    # Argument pour la map (on pointe vers robmob_bringup)
    default_map_path = os.path.join(pkg_bringup, 'config', 'my_map.yaml')
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Chemin complet vers le fichier .yaml de la map'
    )

    # 1. Map Server (Nav2)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map')},
                    {'use_sim_time': False}]
    )

    # 2. Lifecycle Manager (Indispensable pour activer le map_server)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    # 3. Static TF: map -> base_link
    # Permet au mock_robot_node de trouver la position du "robot" (ici à l'origine 0,0)
    fake_robot_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        arguments=['--x', '0',  '--y', '0', '--z', '0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'map', '--child-frame-id', 'base_link']
    )

    # 4. Planner Node (Serveur d'Action)
    planner_node = Node(
        package='rrt_connect_planner',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=[{
            'robot_radius': 0.2, # 0.14*√2 ≃ 0.2 (see urdf description)
            'verbose': True
        }]
    )

    # 5. Mock Robot Node (Client d'Action + Listener RViz)
    cmd_law_node = Node(
        package='robmob_cmd_law',
        executable='cmd_law_node',
        name='cmd_law_node',
        output='screen'
    )

    # 6. RViz2
    # On cherche d'abord le fichier de config spécifié dans ton tree: test_rrt.rviz
    rviz_config_path = os.path.join(pkg_rrt_planner, 'rviz', 'test_rrt.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        output='screen'
    )

    return LaunchDescription([
        map_arg,
        map_server_node,
        lifecycle_manager_node,
        fake_robot_tf_node,
        planner_node,
        cmd_law_node,
        rviz_node
    ])
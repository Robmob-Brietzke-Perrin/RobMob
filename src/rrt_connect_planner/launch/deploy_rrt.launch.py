import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rrt_connect_planner',
            executable='planner_node',
            name='planner'),
    ])
import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="joy_linux", executable="joy_linux_node", name="joystick"
            ),
            launch_ros.actions.Node(
                package="robmob_teleop",
                executable="teleop_joy_node",
                # remappings=[("/cmd_vel", "turtle1/cmd_vel")],
                name="teleop",
            ),
        ]
    )

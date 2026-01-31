from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    rviz_arg = DeclareLaunchArgument("use_rviz", default_value="True")
    sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="True")

    gz_launch_dir = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_gazebo"), "launch"]
    )
    gz_launch = IncludeLaunchDescription(
        PathJoinSubstitution([gz_launch_dir, "turtlebot3_house.launch.py"]),
    )

    carto_launch_dir = PathJoinSubstitution(
        [FindPackageShare("turtlebot3_cartographer"), "launch"]
    )
    carto_launch = IncludeLaunchDescription(
        PathJoinSubstitution([carto_launch_dir, "cartographer.launch.py"]),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    joy_launch_dir = PathJoinSubstitution([FindPackageShare("robmob_teleop"), "launch"])
    joy_launch = IncludeLaunchDescription(
        PathJoinSubstitution([joy_launch_dir, "joystick.launch.py"])
    )

    return LaunchDescription(
        [rviz_arg, sim_time_arg, gz_launch, carto_launch, joy_launch]
    )

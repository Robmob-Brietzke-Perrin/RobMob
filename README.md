# Robotique Mobile

![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/jammy)
![Ros_2](https://img.shields.io/ros/v/humble/rclcpp)

<test name="Starters">
Here are the bash script usefull to start with this project (you must be in /home/[user]/turtlebot4_env_docker):

-Install the docker
```bash
  . bash_scripts/install_docker.bash
```

-Start the docker
```bash
  . bash_scripts/start_docker.bash
```

-Join the docker
```bash
  . bash_scripts/join_docker.bash
```

-Unistall the docker
```bash
  . bash_scripts/uninstall_docker.bash
```

Then you can use the following command lines.

-To create the map using gazebo :
```bash 
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
-To launch the map construction with rvizz :
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True use_rviz:=True
```

Save rviz map
```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```
</test>


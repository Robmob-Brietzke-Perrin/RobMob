# Robotique Mobile

![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/jammy)
![Ros_2](https://img.shields.io/ros/v/humble/rclcpp)

### Summary

- [Setup the project](#quickstart)
- [Path finding algorithm](#rrt-connect)
- [Law of command](#law-of-command)
- [Autonomous exploration](#autonomous-exploration)
- [Obstacle avoidance](#obstacle-avoidance)

## QuickStart

<details>
  <summary>In a docker container</summary>

This section relies on the dockerfile and scripts provided by our coursework supervisors.

First, make sure you are in the project's root folder:`cd /home/<user>/turtlebot4_env_docker` then you can follow the steps below.

1. Installing docker
```bash
$ source bash_scripts/install_docker.bash
```

2. Building the docker image
```bash
$ source bash_script/install_turtlebot4_docker.bash
```

3. Starting a docker container
```bash
$ source bash_scripts/start_docker.bash
```

4. Joining a running container
```bash
$ source bash_scripts/join_docker.bash
```

5. Unistall the docker
```bash
  . bash_scripts/uninstall_docker.bash
```
</details>

---
<details>
  <summary>Directly on your machine</summary>
You have to make sure you have all dependancies on your machine:

1. Install ROS2, either [Humble][2] or [Jazzy][1]

2. Install the corresponding Gazebo: [Fortress][3] or [Harmonic][4]

3. Install the following dependancies

```bash
sudo apt update && sudo apt install \
ros-humble-control ros-humble-controllers \
ros2-humble-slam-toolbox \
ros2-humble-turtlebot3*
```

[1]:https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
[2]:https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
[3]:https:todofindgazebo
[4]:https:todofindgazebo
</details>

---
Whether you chose to build directly in your machine or inside a container, you should now be able to launch the project. Here are some usefull commands:

| cmd | effect |
| :--: | -- |
| `ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py` | start the house map in gazebo |
| `ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True use_rviz:=True` | Launch the SLAM algorithm (here with rviz enabled for visualization purposes) |
| `ros2 launch robmob_teleop joystick.launch.py` | Start the joystick teleoperation (you must have your controller connected, which you can verify by searching for `js0` in `/dev/input`) |
| `ros2 launch robmob_bringup gz_house.launch.py` | Simulteanously launch teh three above |
| `ros2 run nav2_map_server map_saver_cli -f <my_map>` | Save the current map in <my_map> file  |

## Path finding algorithm
<!-- TODO: complete this section -->
We chose to use RRT for it's beauty and efficiency.


![](images/image.png "Examples")

As one may already know, RRT (Rapidly-exploring Random Trees) works as follows :
+ Sets a starting point
+ Randomly choses a point
+ Tests if a step in the direction of the said point is reachable
+ Adds the point to the tree if reachable
+ Checks if the goal is reached

Then iterate, however RRT-connect has one slight change. There are 2 separate starting points and it alternates between the two trees.

The system we are going for is based of an inflated occupancygrid (to do some avoidance) that is provided by the SLAM.

## Law of command
<!-- TODO: complete this section -->

Here we have the path provided by RRT-connect, which will follow with a trajectory control. Meaning a spring-like system which will make the turtlebot follow the trail.

## Obstacle avoidance
<!-- TODO: complete this section -->


On the side of the law of command we will have an avoidance system that will act as an other repulsive force depending on how close the robot is from the obstacle. To do so we will use the lidar to detect and change the course of the robot.
## Autonomous exploration
<!-- TODO: complete this section -->
Now that the robot can freely move in a known environement, we can use a simple program to make the robot wander util it has found something worthy of attention. 

note pour le rendu :
ajouter la possibilité de cliquer sur rvizz pour set un goal
modifier pour que le départ soit toujours la position du robot
dans le rendu final il faut l'archi sous forme de diapo et tout autre graphe dispo
obstacle avoidance est optionnelle donc pas a prioriser
penser a soit passer par l'odom et la conversion dans le reel ou de passer par la map et son referentiel

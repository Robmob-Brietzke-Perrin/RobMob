#ifndef CMD_LAW_NODE_HPP
#define CMD_LAW_NODE_HPP
#ifdef ROS_DISTRO_JAZZY
#include "geometry_msgs/msg/twist_stamped.hpp"
#endif
#ifdef ROS_DISTRO_HUMBLE
#include "geometry_msgs/msg/twist_stamped.hpp"
#endif

class Obstacle_avoidance //recevies the cmdvel and check/modify the cmdvel if the robot will collide also recalculates the path by updating the slam and using RRT-Connect
{
    #ifdef ROS_DISTRO_JAZZY
        void Send_cmd(rclcpp::Publisher<TwistStamped>::SharedPtr cmd_vel_pub_)
    #endif
    #ifdef ROS_DISTRO_HUMBLE
        void Send_cmd(rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_)
    #endif

    //look template online to see how to sub to an action and sub to the interface 
};

class Traj_folowing // uses the RTT action callback to get the path and proceeds to send a contiuous stream of cmd_vels influenced by the dist to it's suposed position
{
    //Uses the obstacle avoidance send_cmd function to send cmd_vels

    //calculates the cmd_vel using the dist between it's supposed position and it's current position therefore it need to sub to the turtlebot odom
};

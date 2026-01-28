#ifndef ROBMOB_CMD_LAW__OBS_AVOID_NODE_HPP
#define ROBMOB_CMD_LAW__OBS_AVOID_NODE_HPP

#ifdef ROS_DISTRO_JAZZY
#include "geometry_msgs/msg/twist_stamped.hpp"
#endif
#ifdef ROS_DISTRO_HUMBLE
#include "geometry_msgs/msg/twist.hpp"
#endif

#include "rclcpp/rclcpp.hpp"

class ObstacleAvoidance :public rclcpp::Node//recevies the cmdvel and check/modify the cmdvel if the robot will collide also recalculates the path by updating the slam and using RRT-Connect
{
    public:
        void Send_cmd(rclcpp::Publisher<TwistCmd>::SharedPtr cmd_vel_pub_){};
    private:
        #ifdef ROS_DISTRO_JAZZY
            using TwistCmd = geometry_msgs::msg::TwistStamped;
        #endif
        #ifdef ROS_DISTRO_HUMBLE
            using TwistCmd = geometry_msgs::msg::Twist;
        #endif
        

    
    //look template online to see how to sub to an action and sub to the interface 
};

#endif //ROBMOB_CMD_LAW__OBS_AVOID_NODE_HPP
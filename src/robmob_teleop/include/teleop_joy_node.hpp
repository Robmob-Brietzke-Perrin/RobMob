#ifndef TURTLE_CUSTOM_HPP
#define TURTLE_CUSTOM_HPP

#include "rclcpp/rclcpp.hpp"
#ifdef ROS_DISTRO_JAZZY
#include "geometry_msgs/msg/twist_stamped.hpp"
#endif
#ifdef ROS_DISTRO_HUMBLE
#include "geometry_msgs/msg/twist_stamped.hpp"
#endif
#include "sensor_msgs/msg/joy.hpp"

using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
using namespace std::chrono_literals;

class TeleopJoy : public rclcpp::Node
{
public:
    TeleopJoy();

private:
    Joy joy_msg_;
#ifdef ROS_DISTRO_JAZZY
    TwistStamped cmd_vel_msg_;
#endif
#ifdef ROS_DISTRO_HUMBLE
    Twist cmd_vel_msg_;
#endif

    rclcpp::TimerBase::SharedPtr timer_;
#ifdef ROS_DISTRO_JAZZY
    rclcpp::Publisher<TwistStamped>::SharedPtr cmd_vel_pub_;
#endif
#ifdef ROS_DISTRO_HUMBLE
    rclcpp::Publisher<TwistStamped>::SharedPtr cmd_vel_pub_;
#endif
    rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
};

#endif
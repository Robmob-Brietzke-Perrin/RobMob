#ifndef TURTLE_CUSTOM_HPP
#define TURTLE_CUSTOM_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
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
    TwistStamped cmd_vel_msg_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
};

#endif
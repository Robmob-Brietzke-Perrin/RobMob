#include "teleop_joy_node.hpp"

TeleopJoy::TeleopJoy()
    : Node("teleop_joy")
{
  // Setup cmd_vel publisher
#ifdef ROS_DISTRO_JAZZY
    cmd_vel_pub_ = this->create_publisher<TwistStamped>("/cmd_vel", 10);
#endif
#ifdef ROS_DISTRO_HUMBLE
    cmd_vel_pub_ = this->create_publisher<Twist>("/cmd_vel", 10);
#endif

  auto timer_callback =
      [this]() -> void
  {
    this->cmd_vel_pub_->publish(cmd_vel_msg_);
  };
  timer_ = this->create_wall_timer(500ms, timer_callback);

  // Setup joy subscriber
  auto joy_callback =
      [this](Joy::UniquePtr msg) -> void
  {
    joy_msg_ = *msg;
#ifdef ROS_DISTRO_JAZZY
    cmd_vel_msg_.twist.linear.x = joy_msg_.axes[1];
    cmd_vel_msg_.twist.angular.z = joy_msg_.axes[2];
#endif
#ifdef ROS_DISTRO_HUMBLE
    cmd_vel_msg_.linear.x = joy_msg_.axes[1];
    cmd_vel_msg_.angular.z = joy_msg_.axes[2];
#endif
  };
  joy_sub_ = this->create_subscription<Joy>("/joy", 10, joy_callback);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopJoy>());
  rclcpp::shutdown();
  return 0;
}
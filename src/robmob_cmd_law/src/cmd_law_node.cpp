#include "robmob_cmd_law/cmd_law_node.hpp"

using namespace std::placeholders;

CmdLawNode::CmdLawNode(const rclcpp::NodeOptions & options) 
: Node("cmd_law_node", options) 
{
    client_ptr_ = rclcpp_action::create_client<ComputePath>(this, "compute_path");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  #ifdef ROS_DISTRO_JAZZY
      cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
  #else
      cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  #endif

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&CmdLawNode::goal_callback, this, _1));

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&CmdLawNode::control_loop, this));//20Hz
    control_timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "CmdLawNode prêt. En attente d'un goal sur /goal_pose...");
}

void CmdLawNode::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    this->last_goal_pose_ = *msg; 
    RCLCPP_INFO(this->get_logger(), "Cible reçue (x: %.2f, y: %.2f). Calcul du chemin...", 
                msg->pose.position.x, msg->pose.position.y);
    this->send_goal(); 
}

void CmdLawNode::send_goal() 
{
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Serveur d'action ComputePath indisponible !");
        return;
    }

    auto goal_msg = ComputePath::Goal();
    
    try {
        auto t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        goal_msg.header.frame_id = "map";
        goal_msg.header.stamp = this->now();
        goal_msg.start.position.x = t.transform.translation.x;
        goal_msg.start.position.y = t.transform.translation.y;
        goal_msg.start.orientation = t.transform.rotation;
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "TF Error: %s", ex.what());
        return;
    }

    goal_msg.goal = this->last_goal_pose_.pose;
    goal_msg.max_iterations = 32768;
    goal_msg.max_tree_size = 16384;

    auto send_goal_options = rclcpp_action::Client<ComputePath>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&CmdLawNode::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&CmdLawNode::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&CmdLawNode::result_callback, this, _1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void CmdLawNode::goal_response_callback(const GoalHandleComputePath::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal refusé par le serveur");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepté, calcul en cours...");
    }
}

void CmdLawNode::feedback_callback(GoalHandleComputePath::SharedPtr, const std::shared_ptr<const ComputePath::Feedback> feedback)
{
    (void)feedback;
}

void CmdLawNode::result_callback(const GoalHandleComputePath::WrappedResult & result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Chemin calculé ! Début du suivi.");
        this->current_path_ = result.result->fin_path;
        this->target_index_ = 0;
        this->path_following_active_ = true;
        this->control_timer_->reset(); 
    } else {
        RCLCPP_ERROR(this->get_logger(), "Échec du calcul de chemin");
    }
}

void CmdLawNode::control_loop()
{
    if (!path_following_active_ || current_path_.poses.empty()) return;

    if (target_index_ >= current_path_.poses.size()) {
        RCLCPP_INFO(this->get_logger(), "Destination atteinte !");
        stop_robot();
        return;
    }

    try {
        auto t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        double curr_x = t.transform.translation.x;
        double curr_y = t.transform.translation.y;
        double curr_yaw = tf2::getYaw(t.transform.rotation);

        auto target_pose = current_path_.poses[target_index_].pose;
        double dx = target_pose.position.x - curr_x;
        double dy = target_pose.position.y - curr_y;
        double dist = std::sqrt(dx*dx + dy*dy);

        if (dist < 0.2) { //Pour éviter de s'arreter si on atteint un point en avance
            target_index_++;
            return;
        }

        double target_yaw = std::atan2(dy, dx);
        double angle_err = target_yaw - curr_yaw;

        while (angle_err > M_PI) angle_err -= 2.0 * M_PI;
        while (angle_err < -M_PI) angle_err += 2.0 * M_PI;

        TwistCmd cmd;
#ifdef ROS_DISTRO_JAZZY
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";
        cmd.twist.linear.x = std::min(0.4, 0.5 * dist);   
        cmd.twist.angular.z = 1.2 * angle_err;           
#else
        cmd.linear.x = std::min(0.4, 0.5 * dist);
        cmd.angular.z = 1.2 * angle_err;
#endif
        cmd_vel_pub_->publish(cmd);

    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "TF Wait: %s", ex.what());
    }
}

void CmdLawNode::stop_robot()
{
    path_following_active_ = false;
    control_timer_->cancel();
    
    TwistCmd stop_msg;

#ifdef ROS_DISTRO_JAZZY
    stop_msg.header.stamp = this->now();
    stop_msg.header.frame_id = "base_link";
    stop_msg.twist.linear.x = 0.0;
    stop_msg.twist.angular.z = 0.0;
#else
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
#endif

    cmd_vel_pub_->publish(stop_msg);
    RCLCPP_INFO(this->get_logger(), "Robot stoppé.");
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CmdLawNode)
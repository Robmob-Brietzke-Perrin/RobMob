#include "robmob_cmd_law/cmd_law_node.hpp"

using namespace std::placeholders;

CmdLawNode::CmdLawNode(const rclcpp::NodeOptions & options) 
: Node("cmd_law_node", options) 
{
    this->declare_parameter("l1", 0.1);    // Dist to virtual control point P
    this->declare_parameter("k1", 0.8);    // Gain in X
    this->declare_parameter("k2", 0.8);    // Gain in Y
    this->declare_parameter("v_max", 0.4); // Max Linear velocity
    this->declare_parameter("w_max", 1.0); // Max Angular velocity

    client_ptr_ = rclcpp_action::create_client<ComputePath>(this, "compute_path");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_vel_pub_ = this->create_publisher<TwistCmd>("/cmd_vel", 10);
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
    if (!path_following_active_ || target_index_ >= current_path_.poses.size()) {
        stop_robot();
        return;
    }

    try {
        // Get current posture
        auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        double curr_x = tf.transform.translation.x;
        double curr_y = tf.transform.translation.y;
        double curr_yaw = tf2::getYaw(tf.transform.rotation);

        // dynamically get control parameters
        double l1 = this->get_parameter("l1").as_double();
        double k1 = this->get_parameter("k1").as_double();
        double k2 = this->get_parameter("k2").as_double();
        double V_MAX = this->get_parameter("v_max").as_double();
        double W_MAX = this->get_parameter("w_max").as_double();

        //`Get current target
        auto target_pose = current_path_.poses[target_index_].pose;
        double xr = target_pose.position.x;
        double yr = target_pose.position.y;

        // compute virtual control point position
        double xp = curr_x + l1 * std::cos(curr_yaw);
        double yp = curr_y + l1 * std::sin(curr_yaw);

        // Compute distance to waypoint
        double dx = xr - curr_x;
        double dy = yr - curr_y;
        double dist = std::sqrt(dx*dx + dy*dy);

        if (dist < 0.15) { // Threshold
            target_index_++;
            return;
        }

        // Intermediate cmd law (on v)
        double v1 = k1 * (xr - xp);
        double v2 = k2 * (yr - yp);

        // Jacobian inversion to get back to u
        double u1 = std::cos(curr_yaw) * v1 + std::sin(curr_yaw) * v2;
        double u2 = (-std::sin(curr_yaw) / l1) * v1 + (std::cos(curr_yaw) / l1) * v2;

        // Clamp the cmd and publish
        TwistCmd cmd;
        #ifdef ROS_DISTRO_JAZZY
            cmd.header.stamp = this->now();
            cmd.header.frame_id = "base_link";
            cmd.twist.linear.x = std::clamp(u1, -V_MAX, V_MAX);
            cmd.twist.angular.z = std::clamp(u2, -W_MAX, W_MAX);
        #else
            cmd.linear.x = std::clamp(u1, -V_MAX, V_MAX);
            cmd.angular.z = std::clamp(u2, -W_MAX, W_MAX);
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
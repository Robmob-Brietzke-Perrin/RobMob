#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "robmob_interfaces/action/compute_path.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class MockRobotNode : public rclcpp::Node
{
public:
    using ComputePath = robmob_interfaces::action::ComputePath;
    using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePath>;

    MockRobotNode() : Node("mock_robot_node")
    {
        // 1. Client d'Action
        client_ptr_ = rclcpp_action::create_client<ComputePath>(
            this,
            "compute_path"
        );

        // 2. Setup TF2 (Pour savoir où est le robot)
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 3. Subscription au click RViz
        // Le topic par défaut de l'outil "2D Goal Pose" de RViz est /goal_pose
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&MockRobotNode::goal_callback, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "Waiting for 2D Goal Pose on /goal_pose ...");
    }

private:
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Action server 'compute_path' not available after waiting");
            return;
        }

        // 1. Récupérer la position du robot (Start)
        geometry_msgs::msg::Pose start_pose;
        try {
            // On cherche la transform de base_link vers map
            geometry_msgs::msg::TransformStamped t;
            t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            
            start_pose.position.x = t.transform.translation.x;
            start_pose.position.y = t.transform.translation.y;
            start_pose.orientation = t.transform.rotation;
        }
        catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not get robot position: %s", ex.what());
            // Pour le test, si pas de TF, on met un point arbitraire (ex: 0,0) ou on return
            RCLCPP_WARN(this->get_logger(), "Using (0,0) as start for testing due to missing TF");
            start_pose.position.x = 0.0;
            start_pose.position.y = 0.0;
            start_pose.orientation.w = 1.0;
        }

        // 2. Préparer le Goal de l'Action
        auto goal_msg = ComputePath::Goal();
        goal_msg.header.frame_id = "map";
        goal_msg.header.stamp = this->now();
        
        goal_msg.start = start_pose;
        goal_msg.goal = msg->pose; // Le click RViz
        goal_msg.max_iterations = 32768;

        RCLCPP_INFO(this->get_logger(), "Sending goal: Start(%.2f, %.2f) -> End(%.2f, %.2f)",
            start_pose.position.x, start_pose.position.y,
            msg->pose.position.x, msg->pose.position.y);

        // 3. Envoyer
        auto send_goal_options = rclcpp_action::Client<ComputePath>::SendGoalOptions();
        
        // Callback pour voir l'optimisation en temps réel (Feedback)
        send_goal_options.feedback_callback = 
            [this](GoalHandleComputePath::SharedPtr, const std::shared_ptr<const ComputePath::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "Optimization update received (Path size: %zu)", feedback->inter_path.poses.size());
            };

        send_goal_options.result_callback = 
            [this](const GoalHandleComputePath::WrappedResult & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "Path computation SUCCESS! Final size: %zu", result.result->fin_path.poses.size());
                    // TODO: teleport to goal point
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Path computation FAILED");
                }
            };

        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    rclcpp_action::Client<ComputePath>::SharedPtr client_ptr_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockRobotNode>());
    rclcpp::shutdown();
    return 0;
}
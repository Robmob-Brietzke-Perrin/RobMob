#ifndef RRT_CONNECT_PLANNER__PLANNER_NODE_HPP
#define RRT_CONNECT_PLANNER__PLANNER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rrt_connect_planner/rrt_connect.hpp"
#include "rrt_connect_planner/map_utils.hpp"
#include "robmob_interfaces/action/compute_path.hpp"


constexpr int ALGORITHM_RATE = 10;
constexpr int MAP_QUEUE_SIZE = 10;

using namespace robmob_interfaces::action;
using namespace std::placeholders;
using namespace nav_msgs::msg;
using GoalHandleComputePath = rclcpp_action::ServerGoalHandle<ComputePath>;

class PlannerNode : public rclcpp::Node
{
public:

    PlannerNode()
    : Node("planner_node")
    {
        /* ===== Map Subscription =====*/

        auto map_callback = [this](OccupancyGrid::UniquePtr msg)
        {
            // Use mutex so that this does not interfere with the action (in another thread)
            std::lock_guard<std::mutex> lock(map_mutex_);
            // TODO: inflate map?
            latest_map_ = std::move(msg);
        };
        map_sub_ = this->create_subscription<OccupancyGrid>("/map"/*TODO: find where to get the map*/, MAP_QUEUE_SIZE, map_callback);

        /* ===== Path publisher for Rviz ===== */

        path_pub_ = this->create_publisher<Path>("/path_verbose", 10);

        /* ===== Compute Path Action ===== */

        // Lambda function on goal recieved
        auto handle_goal = [this](
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const ComputePath::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Recieved path computing request.");
            
            if (!is_goal_reachable(goal)) // TODO: implement goal point verification?
            {
                RCLCPP_WARN(this->get_logger(), "Goal position unreachable, action rejected");
                return rclcpp_action::GoalResponse::REJECT;
            }
            if (latest_map_ == nullptr)
            {
                RCLCPP_WARN(this->get_logger(), "No map is loaded yet, action rejected");
                return rclcpp_action::GoalResponse::REJECT;
            }

            (void)uuid; // (does effectively nothing exept getting rid of unused param warn)
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };

        // Lambda function on goal cancel (TODO: call this from client in case too long?)
        auto handle_cancel = [this](const std::shared_ptr<GoalHandleComputePath> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            if (goal_handle->is_canceling())
            {
                // TODO: correctly cleanup interrupted computation?
            }
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        // Lambda function on goal accepted (must finish quickly so it delegates to a thread)
        auto handle_accepted = [this](const std::shared_ptr<GoalHandleComputePath> goal_handle)
        {
            auto execute_in_thread = [this, goal_handle]()
            { return this->execute(goal_handle); };
            std::thread{execute_in_thread}.detach();
        };

        // Create the action server
        this->compute_path_server_ = rclcpp_action::create_server<ComputePath>(
            this,
            "compute_path",
            handle_goal,
            handle_cancel,
            handle_accepted);
    }
    void execute(const std::shared_ptr<GoalHandleComputePath> goal_handle);
private:
    rclcpp_action::Server<ComputePath>::SharedPtr compute_path_server_;
    rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<Path>::SharedPtr path_pub_;
    OccupancyGrid::SharedPtr latest_map_ = nullptr;
    std::mutex map_mutex_;
};

#endif // RRT_CONNECT_PLANNER__PLANNER_NODE_HPP
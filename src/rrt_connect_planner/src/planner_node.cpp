#include "rrt_connect_planner/planner_node.hpp"
#include "rrt_connect_planner/rrt_connect.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::placeholders;

PlannerNode::PlannerNode() : Node("rrt_planner_node")
{
    // Parameters
    this->declare_parameter("robot_radius", 0.25);
    this->declare_parameter("verbose", true);

    robot_radius_ = this->get_parameter("robot_radius").as_double();
    verbose_ = this->get_parameter("verbose").as_bool();

    // Map subscribtion (transient_local)
    rclcpp::QoS map_qos(1);
    map_qos.reliable();
    map_qos.durability_volatile();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", map_qos, std::bind(&PlannerNode::map_callback, this, _1));

    // Path publication for verbosity / debug
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);

    // Action server
    compute_path_server_ = rclcpp_action::create_server<ComputePath>(
        this, "compute_path",
        std::bind(&PlannerNode::handle_goal, this, _1, _2),
        std::bind(&PlannerNode::handle_cancel, this, _1),
        std::bind(&PlannerNode::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "RRT Connect Planner Initialized");
}

void PlannerNode::map_callback(const OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_helper_.initialize(*msg);
    // map_helper_.inflate_obstacles(robot_radius_);
    RCLCPP_INFO(this->get_logger(), "Map updated and inflated");
}

rclcpp_action::GoalResponse PlannerNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputePath::Goal> goal)
{
  // check that map is ready
    if (map_helper_.get_state() == MapHelper::INSTANCIATED) {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal: Map not ready");
        return rclcpp_action::GoalResponse::REJECT;
    }
    // check that goal request is accessible
    // if(!map_helper_.is_free(goal->goal)){
    //     RCLCPP_WARN(this->get_logger(), "Rejecting goal: Goal is not accessible !");
    //     return rclcpp_action::GoalResponse::REJECT;
    // }
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlannerNode::handle_cancel(
    const std::shared_ptr<GoalHandleComputePath> goal_handle)
{
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PlannerNode::handle_accepted(const std::shared_ptr<GoalHandleComputePath> goal_handle)
{
    std::thread{std::bind(&PlannerNode::execute, this, _1), goal_handle}.detach();
}

nav_msgs::msg::Path PlannerNode::to_ros_path(const std::vector<Point>& points) 
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = this->now();

    for (size_t i = 0; i < points.size(); ++i) {
        geometry_msgs::msg::PoseStamped ps;
        ps.pose.position.x = points[i].x;
        ps.pose.position.y = points[i].y;

        double yaw = 0.0;
        if (i < points.size() - 1) {
            // Orientation is set to point the next point in path
            yaw = std::atan2(points[i+1].y - points[i].y, points[i+1].x - points[i].x);
        } else if (i > 0) {
            // last point keep previous orientation
            yaw = std::atan2(points[i].y - points[i-1].y, points[i].x - points[i-1].x);
        }

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        ps.pose.orientation = tf2::toMsg(q);
        
        path.poses.push_back(ps);
    }
    return path;
}

void PlannerNode::execute(const std::shared_ptr<GoalHandleComputePath> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ComputePath::Result>();
    rclcpp::Rate debug_rate(1.2); // 0.5s entre chaque étape pour bien voir dans RViz

    RCLCPP_INFO(this->get_logger(), "New Path Request: Start(%.2f, %.2f) -> Goal(%.2f, %.2f)", 
                goal->start.position.x, goal->start.position.y, goal->goal.position.x, goal->goal.position.y);

    // Initialisation & instanciate Check Collision
    auto collision_checker = [this](double x, double y) -> bool {
        return this->map_helper_.is_free(x, y);
    };

    std::lock_guard<std::mutex> lock(map_mutex_);
    if (map_helper_.get_state() == MapHelper::INITIALIZED) {
        RCLCPP_INFO(this->get_logger(), "Inflating map for new planning request...");
        map_helper_.inflate_obstacles(robot_radius_);
    }
    // if (!map_helper_.is_free(goal->start.position.x, goal->start.position.y) || 
    //     !map_helper_.is_free(goal->goal.position.x, goal->goal.position.y)) {
    //     RCLCPP_ERROR(this->get_logger(), "Action Failed: Start or Goal is in obstacle!");
    //     goal_handle->abort(result);
    //     return;
    // }

    RRTConnect rrt(collision_checker, 
                   map_helper_.get_min_x(), map_helper_.get_max_x(),
                   map_helper_.get_min_y(), map_helper_.get_max_y());

    // Start RRT connect algorithm
    Tree tree_start, tree_goal;
    tree_start.add_node({goal->start.position.x, goal->start.position.y}, nullptr);
    tree_goal.add_node({goal->goal.position.x, goal->goal.position.y}, nullptr);
    
    std::vector<Point> current_path;
    bool success = false;
    Tree *tree_a = &tree_start, *tree_b = &tree_goal;

    for (int i = 0; i < (int)goal->max_iterations && rclcpp::ok(); ++i) {
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            return;
        }

        size_t total_nodes = tree_start.get_nodes().size() + tree_goal.get_nodes().size();
        if (total_nodes >= goal->max_tree_size) {
            RCLCPP_WARN(this->get_logger(), "Planning arrêté : Taille max d'arbre atteinte (%zu/%u)", 
                        total_nodes, goal->max_tree_size);
            break;
        }

        Point q_rand = rrt.random_config();
        std::shared_ptr<TreeNode> q_new, q_conn;

        if (rrt.extend(*tree_a, q_rand, q_new) != RRTConnect::TRAPPED) {
            RCLCPP_INFO(this->get_logger(), "extenting...");
            if (rrt.connect(*tree_b, q_new->get_point(), q_conn) == RRTConnect::REACHED) {
                current_path = (tree_a == &tree_start) ? rrt.get_path(q_new, q_conn) : rrt.get_path(q_conn, q_new);
                success = true;
                break;
            }
        }
        std::swap(tree_a, tree_b);
    }

    if (!success) {
        RCLCPP_WARN(this->get_logger(), "RRT Failed to find path");
        goal_handle->abort(result);
        return;
    }
    // RRT is done, now we try to improve the path

    auto publish_stage = [&](const std::string& label, const std::vector<Point>& path_pts) {
        RCLCPP_INFO(this->get_logger(), "Stage: %s (Points: %zu)", label.c_str(), path_pts.size());
        auto ros_path = to_ros_path(path_pts);
        feedback_msg_.inter_path = ros_path;
        goal_handle->publish_feedback(std::make_shared<ComputePath::Feedback>(feedback_msg_));
        if (verbose_) {
            path_pub_->publish(ros_path);
            // debug_rate.sleep();
        }
    };

    // First feedback : raw path
    publish_stage("Raw RRT Path", current_path);

    current_path = rrt.prune_path(current_path);
    // Second feedback : pruned path (eliminate a lot of points & no more jittering)
    publish_stage("Pruned Path", current_path);

    // to be considered : add distance to obstacles (~gradient descent) if want to optimize safety

    current_path = rrt.smooth_corners(current_path, 0.5); // max_cut_dist = 0.3m
    current_path = rrt.smooth_corners(current_path, 0.5); // max_cut_dist = 0.3m
    // Last feedback : smooth out the path (after pruned -> sharp turns, this add some points to help that)
    publish_stage("Corner Smoothing", current_path);

    // Send result with optimized path
    result->fin_path = to_ros_path(current_path);
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Path computation successful");
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
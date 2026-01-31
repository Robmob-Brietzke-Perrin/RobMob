#include "rrt_connect_planner/planner_node.hpp"
#include "rrt_connect_planner/rrt_connect.hpp"

using namespace std::placeholders;

PlannerNode::PlannerNode() : Node("rrt_planner_node")
{
    this->declare_parameter("robot_radius", 0.3f);
    this->declare_parameter("verbose", true);

    robot_radius_ = this->get_parameter("robot_radius").as_double();
    verbose_ = this->get_parameter("verbose").as_bool();

    rclcpp::QoS map_qos(1);
    map_qos.reliable();
    map_qos.durability_volatile();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", map_qos, std::bind(&PlannerNode::map_callback, this, _1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);

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
    map_helper_.inflate_obstacles(robot_radius_);
    RCLCPP_INFO(this->get_logger(), "Map updated and inflated");
}

rclcpp_action::GoalResponse PlannerNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputePath::Goal> goal)
{
    if (map_helper_.get_state() != MapHelper::INFLATED) {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal: Map not ready");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if(!map_helper_.is_free(goal->goal)){
        RCLCPP_WARN(this->get_logger(), "Rejecting goal: Goal is not accessible !");
        return rclcpp_action::GoalResponse::REJECT;
    }
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

    for (const auto& p : points) {
        geometry_msgs::msg::PoseStamped ps;
        // ps.header.frame_id = "map";
        ps.pose.position.x = p.x;
        ps.pose.position.y = p.y;
        ps.pose.orientation.w = 1.0; 
        // FIXME: ajouter l'orientation (peut-être vers le point n+1?)
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

    // 1. Initialisation & Check Collision
    auto collision_checker = [this](double x, double y) -> bool {
        return this->map_helper_.is_free(x, y);
    };

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!map_helper_.is_free(goal->start.position.x, goal->start.position.y) || 
            !map_helper_.is_free(goal->goal.position.x, goal->goal.position.y)) {
            RCLCPP_ERROR(this->get_logger(), "Action Failed: Start or Goal is in obstacle!");
            goal_handle->abort(result);
            return;
        }
    }

    RRTConnect rrt(collision_checker, 
                   map_helper_.get_min_x(), map_helper_.get_max_x(),
                   map_helper_.get_min_y(), map_helper_.get_max_y());

    // 2. Phase de Recherche RRT-Connect
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

        Point q_rand = rrt.random_config();
        std::shared_ptr<TreeNode> q_new, q_conn;

        if (rrt.extend(*tree_a, q_rand, q_new) != RRTConnect::TRAPPED) {
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

    // --- SÉQUENCE D'OPTIMISATION VISUELLE ---
    auto publish_stage = [&](const std::string& label, const std::vector<Point>& path_pts) {
        RCLCPP_INFO(this->get_logger(), "Stage: %s (Points: %zu)", label.c_str(), path_pts.size());
        auto ros_path = to_ros_path(path_pts);
        feedback_msg_.inter_path = ros_path;
        goal_handle->publish_feedback(std::make_shared<ComputePath::Feedback>(feedback_msg_));
        if (verbose_) {
            path_pub_->publish(ros_path);
            debug_rate.sleep();
        }
    };

    // Étape A : Chemin Brut
    publish_stage("Raw RRT Path", current_path);

    // Étape B : Élagage (Pruning)
    current_path = rrt.prune_path(current_path);
    publish_stage("Pruned Path", current_path);

    // Étape C : Corner Smoothing (Injection de points)
    current_path = rrt.smooth_corners(current_path, 0.3); // max_cut_dist = 0.3m
    publish_stage("Corner Smoothing", current_path);
    current_path = rrt.smooth_corners(current_path, 0.3); // max_cut_dist = 0.3m
    publish_stage("Corner Smoothing", current_path);

    // Étape D : Gradient Smoothing (Lissage final léger)
    // current_path = rrt.smooth_path(current_path, 15);
    // publish_stage("Final Gradient Smoothing", current_path);

    // 3. Résultat Final
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

/*

ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/joachimperrin/turtlebot4_env_docker/shared/robmob_ws/src/robmob_bringup/config/test_map.yaml

ros2 lifecycle set /map_server configure

ros2 run rrt_connect_planner planner_node

ros2 action send_goal /compute_path robmob_interfaces/action/ComputePath "{start: {header: {frame_id: 'map'}, pose: {position: {x: -9.0, y: -9.0, z: 0.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}, goal: {header: {frame_id: 'map'}, pose: {position: {x: 40.0, y: 40.0, z: 0.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}}"



*/
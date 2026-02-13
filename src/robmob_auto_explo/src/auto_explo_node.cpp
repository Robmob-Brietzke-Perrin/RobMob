#include "robmob_auto_explo/auto_explo_node.hpp"
#include "robmob_auto_explo/explo_helper.hpp"

using namespace std::chrono_literals;

AutoExploNode::AutoExploNode() : Node("auto_explo_node") {

    this->declare_parameter("goal_threshold", 0.15);
    this->declare_parameter("obstacle_threshold", 0.35);
    this->declare_parameter("wait_duration", 3.0);
    this->declare_parameter("scan_fov_deg", 45.0);
    this->declare_parameter("search_dist", 1.0);
    this->declare_parameter("timer_period_ms", 100);

    // Initialisation de la structure
    params_.goal_threshold = this->get_parameter("goal_threshold").as_double();
    params_.obstacle_threshold = this->get_parameter("obstacle_threshold").as_double();
    params_.wait_duration = this->get_parameter("wait_duration").as_double();
    params_.scan_fov_deg = this->get_parameter("scan_fov_deg").as_double();
    params_.search_dist = this->get_parameter("search_dist").as_double();
    params_.timer_period_ms = this->get_parameter("timer_period_ms").as_int();

    // Subscriptions
    map_sub_ = this->create_subscription<OccupancyGrid>(
        "map", 10, std::bind(&AutoExploNode::map_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<LaserScan>(
        "scan", rclcpp::SensorDataQoS(), std::bind(&AutoExploNode::scan_callback, this, std::placeholders::_1));

    // Publisher vers le planner (délègue à cmd_law_node)
    goal_pub_ = this->create_publisher<PoseStamped>("goal_pose", 10);

    // TF Buffer et Listener pour la pose du robot
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Boucle de décision (pas trop pressé, 2Hz par défault, à tune eventuellement)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(params_.timer_period_ms), 
        std::bind(&AutoExploNode::decision_loop, this));

    stop_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "stop_exploration", 10, std::bind(&AutoExploNode::stop_cb, this, std::placeholders::_1));

    // map_save_client_ = this->create_client<SaveMap>("/slam_toolbox/save_map");
    // this->declare_parameter("map_save_path", "~/robmob_ws/src/robmob_bringup/config/map_generee");

    RCLCPP_INFO(this->get_logger(), "Noeud d'exploration auto prêt.");
}

void AutoExploNode::stop_cb(const std_msgs::msg::Empty::SharedPtr msg) {
    (void)msg;
    RCLCPP_INFO(this->get_logger(), "Fin de l'exploration...");

    timer_->cancel();
    // Renvoie le robot à la base
    if (initial_pose_saved_) {
        initial_pose_.header.stamp = this->now();
        goal_pub_->publish(initial_pose_);
        RCLCPP_INFO(this->get_logger(), "Ordre de retour au départ envoyé.");
    }

    // Save la map via le service de Slam Toolbox
    // if (!map_save_client_->wait_for_service(std::chrono::seconds(5))) {
    //     RCLCPP_ERROR(this->get_logger(), "Service Slam Toolbox non disponible !");
    // } else {
    //     auto request = std::make_shared<SaveMap::Request>();
    //     request->name.data = this->get_parameter("map_save_path").as_string();; 
        
    //     RCLCPP_INFO(this->get_logger(), "Sauvegarde de la carte via Slam Toolbox...");
    //     map_save_client_->async_send_request(request);
    // }

    // // Délai pour laisser le temps au robot de bouger et au fichier d'être écrit
    // rclcpp::sleep_for(std::chrono::seconds(3));
    // rclcpp::shutdown();
}

void AutoExploNode::decision_loop() {
    // 1. Vérifications de base : on a besoin des données pour travailler
    if (!latest_map_ || !latest_scan_) return;

    double x, y, yaw;
    if (!get_robot_pose(x, y, yaw)) return;

    // Initialisation de la position de départ (une seule fois)
    if (!initial_pose_saved_) {
        initial_pose_.header.frame_id = "map";
        initial_pose_.pose.position.x = x;
        initial_pose_.pose.position.y = y;
        initial_pose_.pose.orientation.w = 1.0;
        initial_pose_saved_ = true;
        RCLCPP_INFO(this->get_logger(), "Position de départ sauvegardée.");
    }

    // --- PHASE 1 : SURVEILLANCE DES OBSTACLES ---
    float min_scan = 100.0f;
    // Conversion Deg -> Rad depuis le paramètre
    float fov_rad = (params_.scan_fov_deg * M_PI / 180.0f);
    
    int center_index = (-latest_scan_->angle_min) / latest_scan_->angle_increment;
    int index_range = (fov_rad / 2.0) / latest_scan_->angle_increment;

    // // On vérifie le scan à chaque itération du timer (ex: 10Hz)
    // float min_scan = 100.0f;
    // float fov_rad = (45.0f * M_PI / 180.0f);
    // int center_index = (-latest_scan_->angle_min) / latest_scan_->angle_increment;
    // int index_range = (fov_rad / 2.0) / latest_scan_->angle_increment;

    for (int i = std::max(0, center_index - index_range); 
         i <= std::min((int)latest_scan_->ranges.size() - 1, center_index + index_range); ++i) {
        float r = latest_scan_->ranges[i];
        if (r > latest_scan_->range_min && r < min_scan) min_scan = r;
    }

    // Si un obstacle est trop proche alors qu'on avance
    if (goal_active_ && min_scan <= params_.obstacle_threshold) {
        RCLCPP_WARN(this->get_logger(), "Obstacle détecté à %.2fm ! Arrêt.", min_scan);
        publish_goal(x, y, yaw, 0.0, 0.0);
        goal_active_ = false;
        is_waiting_ = true;
        last_stop_time_ = this->now();
        return; 
    }

    // attente pour ne pas recalculer un goal avant de se sortir de dvt l'obstacle
    if (is_waiting_) {
        auto elapsed = (this->now() - last_stop_time_).seconds();
        if (elapsed < params_.wait_duration) return; 
        is_waiting_ = false;
        RCLCPP_INFO(this->get_logger(), "Fin du délai d'attente, recherche d'une nouvelle direction...");
    }

    if (goal_active_) {
        double dist_to_goal = std::hypot(current_goal_.pose.position.x - x, current_goal_.pose.position.y - y);
        if (dist_to_goal <= params_.goal_threshold) {
            goal_active_ = false;
        }
    } else {
        auto best_angle = ExploHelper::getBestDirection(
            latest_scan_, latest_map_, x, y, yaw, params_.search_dist, last_angle_);

        if (best_angle.has_value()) {
            publish_goal(x, y, yaw, best_angle.value(), params_.search_dist);
            last_angle_ = best_angle.value();
            goal_active_ = true;
        }
    }
}

bool AutoExploNode::get_robot_pose(double &x, double &y, double &yaw) {
    try {
        auto t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        x = t.transform.translation.x;
        y = t.transform.translation.y;

        double qz = t.transform.rotation.z;
        double qw = t.transform.rotation.w;
        yaw = 2.0 * atan2(qz, qw); 
        return true;
    } catch (const tf2::TransformException & ex) {
        return false;
    }
}

void AutoExploNode::publish_goal(double x, double y, double yaw, double sector_angle, double dist) {
    current_goal_.header.stamp = this->now();
    current_goal_.header.frame_id = "map";
    current_goal_.pose.position.x = x + dist * std::cos(yaw + sector_angle);
    current_goal_.pose.position.y = y + dist * std::sin(yaw + sector_angle);
    current_goal_.pose.orientation.w = 1.0;

    goal_pub_->publish(current_goal_);
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoExploNode>());
    rclcpp::shutdown();
    return 0;
}
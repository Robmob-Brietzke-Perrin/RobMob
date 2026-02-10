#ifndef RRT_CONNECT_PLANNER__RRT_CONNECT_HPP
#define RRT_CONNECT_PLANNER__RRT_CONNECT_HPP

#include <functional>
#include <vector>
#include <random>
#include <ctime>
#include "rrt_connect_planner/rrt_types.hpp"

class RRTConnect {
public:
    using CollisionChecker = std::function<bool(double, double)>;
    
    enum State { TRAPPED, REACHED, ADVANCED };

    RRTConnect(CollisionChecker checker, 
               double min_x, double max_x, 
               double min_y, double max_y,
               float step_size = 0.07f);

    // Core RRT
    Point random_config();
    State extend(Tree &tree, const Point &q_target, std::shared_ptr<TreeNode> &new_node_out);
    State connect(Tree &tree, const Point &q_target, std::shared_ptr<TreeNode> &last_node_out);
    std::vector<Point> get_path(std::shared_ptr<TreeNode> node_start, std::shared_ptr<TreeNode> node_goal);

    // Optimisation Algorithms
    std::vector<Point> prune_path(const std::vector<Point>& path);
    // std::vector<Point> smooth_path(const std::vector<Point>& path, int iterations = 1);
    std::vector<Point> smooth_corners(const std::vector<Point>& path, double max_cut_dist);

private:
    std::shared_ptr<TreeNode> nearest_neighbor(const Point &point, const Tree &tree);
    bool new_state(const Point &q_target, const Point &q_near, Point &q_new_out);
    bool is_segment_free(const Point &p1, const Point &p2);

    CollisionChecker checker_;
    double min_x_, max_x_, min_y_, max_y_;
    float step_size_;
};

#endif // RRT_CONNECT_PLANNER__RRT_CONNECT_HPP
#include "rrt_connect_planner/rrt_connect.hpp"
#include <algorithm>
#include <limits>
#include <cmath>

RRTConnect::RRTConnect(CollisionChecker checker, 
                       double min_x, double max_x, 
                       double min_y, double max_y, 
                       float step_size)
    : checker_(checker), 
      min_x_(min_x), max_x_(max_x), 
      min_y_(min_y), max_y_(max_y), 
      step_size_(step_size)
{
    std::srand(std::time(nullptr));
}

Point RRTConnect::random_config() {
    float rx = static_cast<float>(rand()) / RAND_MAX;
    float ry = static_cast<float>(rand()) / RAND_MAX;
    return { min_x_ + (max_x_ - min_x_) * rx, min_y_ + (max_y_ - min_y_) * ry };
}

std::shared_ptr<TreeNode> RRTConnect::nearest_neighbor(const Point &point, const Tree &tree) {
    std::shared_ptr<TreeNode> nearest = nullptr;
    double min_d = std::numeric_limits<double>::max();

    for (const auto &node : tree.get_nodes()) {
        // Comparing squared dist
        double d = dist_sq(node->get_point(), point);
        if (d < min_d) {
            min_d = d;
            nearest = node;
        }
    }
    return nearest;
}

bool RRTConnect::new_state(const Point &q_target, const Point &q_near, Point &q_new_out) {
    double d = dist(q_target, q_near);
    if (d < 0.001) return false;

    if (d <= step_size_) {
        q_new_out = q_target;
    } else {
        double ratio = step_size_ / d;
        q_new_out.x = q_near.x + (q_target.x - q_near.x) * ratio;
        q_new_out.y = q_near.y + (q_target.y - q_near.y) * ratio;
    }
    return checker_(q_new_out.x, q_new_out.y);
}

RRTConnect::State RRTConnect::extend(Tree &tree, const Point &q_target, std::shared_ptr<TreeNode> &new_node_out) {
    auto q_near = nearest_neighbor(q_target, tree);
    Point q_new;

    if (new_state(q_target, q_near->get_point(), q_new)) {
        new_node_out = tree.add_node(q_new, q_near);
        if (dist(q_new, q_target) < 0.1) return REACHED;
        return ADVANCED;
    }
    return TRAPPED;
}

RRTConnect::State RRTConnect::connect(Tree &tree, const Point &q_target, std::shared_ptr<TreeNode> &last_node_out) {
    State s = ADVANCED;
    std::shared_ptr<TreeNode> tmp;
    while (s == ADVANCED) {
        s = extend(tree, q_target, tmp);
        if (s != TRAPPED) last_node_out = tmp;
    }
    return s;
}

std::vector<Point> RRTConnect::get_path(std::shared_ptr<TreeNode> node_start_side, std::shared_ptr<TreeNode> node_goal_side) {
    std::vector<Point> path;
    
    // joint node to start half
    auto curr = node_start_side;
    while (curr) {
        path.push_back(curr->get_point());
        curr = curr->get_parent();
    }
    std::reverse(path.begin(), path.end());

    // joint node to goal half
    curr = node_goal_side;
    while (curr) {
        path.push_back(curr->get_point());
        curr = curr->get_parent();
    }
    return path;
}

bool RRTConnect::is_segment_free(const Point &p1, const Point &p2) {
    double d = dist(p1, p2);
    if (d < 0.01) return true;
    
    double step = 0.05; 
    int steps = std::floor(d / step);

    for (int i = 1; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        double x = p1.x + (p2.x - p1.x) * t;
        double y = p1.y + (p2.y - p1.y) * t;
        if (!checker_(x, y)) return false;
    }
    return true;
}

std::vector<Point> RRTConnect::prune_path(const std::vector<Point>& path) {
    if (path.size() < 3) return path;
    
    std::vector<Point> pruned;
    pruned.push_back(path.front());
    
    size_t current = 0;
    while (current < path.size() - 1) {
        size_t next = current + 1;
        // Look for the next reachable point
        for (size_t i = path.size() - 1; i > current + 1; --i) {
            if (is_segment_free(path[current], path[i])) {
                next = i;
                break;
            }
        }
        pruned.push_back(path[next]);
        current = next;
    }
    return pruned;
}

std::vector<Point> RRTConnect::smooth_corners(const std::vector<Point>& path, double max_cut_dist) {
    if (path.size() < 3) return path;

    std::vector<Point> smooth_path;
    smooth_path.reserve(path.size() * 2);
    smooth_path.push_back(path[0]);

    for (size_t i = 1; i < path.size() - 1; ++i) {
        const Point& A = smooth_path.back();
        const Point& B = path[i];
        const Point& C = path[i+1];

        double dist_BA = dist(B, A);
        double dist_BC = dist(B, C);
        
        // If the points are already close, continue
        if (dist_BA < 0.05 || dist_BC < 0.05) {
            smooth_path.push_back(B);
            continue;
        }

        // Adapt d to available distance (max 40% of the neighboring segments)
        double d = std::min({max_cut_dist, dist_BA * 0.4, dist_BC * 0.4});

        Point B1 = { B.x + (A.x - B.x) * (d / dist_BA), B.y + (A.y - B.y) * (d / dist_BA) };
        Point B2 = { B.x + (C.x - B.x) * (d / dist_BC), B.y + (C.y - B.y) * (d / dist_BC) };

        if (is_segment_free(B1, B2)) {
            smooth_path.push_back(B1);
            smooth_path.push_back(B2);
        } else {
            smooth_path.push_back(B);
        }
    }

    smooth_path.push_back(path.back());
    return smooth_path;
}

// std::vector<Point> RRTConnect::smooth_path(const std::vector<Point>& path, int iterations) {
//     if (path.size() < 3) return path;

//     std::vector<Point> current_path = path;
//     double weight_data = 0.2;
//     double weight_smooth = 0.2;
    
//     for (int iter = 0; iter < iterations; ++iter) {
//         for (size_t i = 1; i < current_path.size() - 1; ++i) {
//             Point p_orig = current_path[i];

//             double x_new = p_orig.x + weight_data * (path[i].x - p_orig.x) +
//                            weight_smooth * (current_path[i-1].x + current_path[i+1].x - 2.0 * p_orig.x);
//             double y_new = p_orig.y + weight_data * (path[i].y - p_orig.y) +
//                            weight_smooth * (current_path[i-1].y + current_path[i+1].y - 2.0 * p_orig.y);
            
//             Point p_new = {x_new, y_new};

//             if (is_segment_free(current_path[i-1], p_new) && is_segment_free(p_new, current_path[i+1])) {
//                 current_path[i] = p_new;
//             }
//         }
//     }
//     return current_path;
// }
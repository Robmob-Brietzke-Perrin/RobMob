#ifndef RRT_CONNECT_PLANNER__RRT_TYPES_HPP
#define RRT_CONNECT_PLANNER__RRT_TYPES_HPP

#include <vector>
#include <memory>
#include <cmath>

struct Point {
    double x;
    double y;
};

inline double dist_sq(const Point& p1, const Point& p2) {
    return std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2);
}

inline double dist(const Point& p1, const Point& p2) {
    return std::sqrt(dist_sq(p1, p2));
}

class TreeNode {
public:
    TreeNode(Point p, std::shared_ptr<TreeNode> parent = nullptr)
        : point_(p), parent_(parent) {}

    const Point& get_point() const { return point_; }
    std::shared_ptr<TreeNode> get_parent() const { return parent_; }

private:
    Point point_;
    std::shared_ptr<TreeNode> parent_;
};

class Tree {
public:
    std::shared_ptr<TreeNode> add_node(Point p, std::shared_ptr<TreeNode> parent) {
        auto node = std::make_shared<TreeNode>(p, parent);
        nodes_.push_back(node);
        return node;
    }
    const std::vector<std::shared_ptr<TreeNode>>& get_nodes() const { return nodes_; }
    void clear() { nodes_.clear(); }

private:
    std::vector<std::shared_ptr<TreeNode>> nodes_;
};

#endif // RRT_CONNECT_PLANNER__RRT_TYPES_HPP
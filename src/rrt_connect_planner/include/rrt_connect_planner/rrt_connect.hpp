#ifndef ROBMOB_RRT_CONNECT_HPP
#define ROBMOB_RRT_CONNECT_HPP

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rrt_connect_planner/map_helper.hpp"

using namespace geometry_msgs::msg;

constexpr float STEP_SIZE = 0.5f;

inline float cart_dist(
    const Pose &p1,
    const Pose &p2)  // Gives the distance in between two given Pose
{
  return std::sqrt(std::pow(p1.position.x - p2.position.x, 2) +
                   std::pow(p1.position.y - p2.position.y, 2));
}

class TreeNode {
 public:
  TreeNode(Pose pose, std::shared_ptr<TreeNode> parent =
                          nullptr)  // By default defined without parent
      : pose_(pose), parent_(parent) {}

  const Pose &get_pose() const { return pose_; }  // returns the position
  std::shared_ptr<TreeNode> get_parent() const {
    return parent_;
  }  // returns the parent of the current point/Treenode

 private:
  Pose pose_;
  std::shared_ptr<TreeNode> parent_;
};

class Tree {
 public:
  std::shared_ptr<TreeNode> add_node(
      Pose pose, std::shared_ptr<TreeNode>
                     parent);  // add an element(TreeNode) to the Tree
  const std::vector<std::shared_ptr<TreeNode>> &get_nodes() const {
    return nodes_;
  }  // gives a vector of the current nodes in the tree

 private:
  std::vector<std::shared_ptr<TreeNode>> nodes_;
};

class RRTConnect {
  using Path = nav_msgs::msg::Path;

 public:
  enum State {
    TRAPPED,
    REACHED,
    ADVANCED
  };  // used to define the state of which a given point (Trapped = out of free
      // map or inaccessible / Reached = got to the goal / Advanced = the point
      // is accessible)

  RRTConnect(MapHelper &map_helper) : map_helper_(map_helper) {
    std::srand(std::time(nullptr));
  }

  Pose random_config();
  State extend(
      Tree &tree, const Pose &q_target,
      std::shared_ptr<TreeNode> &new_node_out);  // tries to reach a new point
  State connect(
      Tree &tree, const Pose &q_target,
      std::shared_ptr<TreeNode>
          &last_node_out);  // loops on extend while it returns ADVANCED
  Path get_path(std::shared_ptr<TreeNode> node_start,
                std::shared_ptr<TreeNode>
                    node_goal);  // loops back from the goal and the start from
                                 // the joint point (where both trees link) and
                                 // returns the path found

 private:
  std::shared_ptr<TreeNode> nearest_neighbor(const Pose &point,
                                             const Tree &tree);
  bool new_state(const Pose &q_target, const Pose &q_near, Pose &q_new_out);

  bool is_valid(const Pose &p) { return map_helper_.is_free(p); }
  MapHelper &map_helper_;
};

#endif
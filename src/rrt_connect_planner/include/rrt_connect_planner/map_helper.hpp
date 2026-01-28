#ifndef RRT_CONNECT_PLANNER__MAP_HELPER_HPP
#define RRT_CONNECT_PLANNER__MAP_HELPER_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class MapHelper
{
public:
  enum State { INSTANCIATED, INITIALIZED, INFLATED };

  MapHelper() = default;

  void initialize(const nav_msgs::msg::OccupancyGrid &map_msg);
  void inflate_obstacles(float bot_radius);

  State get_state() const { return state_; }
  const nav_msgs::msg::OccupancyGrid &get_map() const { return map_; }

  
  // Helpers
  bool is_in_map(const geometry_msgs::msg::Pose &pose);
  bool is_free(const geometry_msgs::msg::Pose &pose);
  bool is_free(double x, double y);

  // Conversions Workspace <-> Map
  bool ws_to_map(double wx, double wy, int &mx, int &my);
  void map_to_ws(int mx, int my, double &wx, double &wy);

  // Getters (map limits)
  double get_min_x() const { return origin_x_; }
  double get_max_x() const { return origin_x_ + (width_ * resolution_); }

  double get_min_y() const { return origin_y_; }
  double get_max_y() const { return origin_y_ + (height_ * resolution_); }

private:
  int get_index(int mx, int my) const;

  State state_ = INSTANCIATED;
  nav_msgs::msg::OccupancyGrid map_;

  uint32_t width_ = 0;
  uint32_t height_ = 0;
  float resolution_ = 0.05f;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
};

#endif // RRT_CONNECT_PLANNER__MAP_HELPER_HPP
#ifndef RRT_CONNECT_PLANNER__MAP_HELPER_HPP
#define RRT_CONNECT_PLANNER__MAP_HELPER_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class MapHelper
{
  using Pose = geometry_msgs::msg::Pose;
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

public:
  enum State { INSTANCIATED, INITIALIZED, INFLATED };//gives the various states of the map

  void initialize(const OccupancyGrid &map_msg);//sets the map based on the SLAM
  void inflate_obstacles(float bot_radius);//makes the obstacles bigger to make sure we don't hit any
  
  State get_state() const { return state_; }//gives the current state of the map
  const OccupancyGrid &get_map() const { return map_; }//returns the current map

  bool is_in_map(const Pose &pose);//tests if the given point is bounds
  bool is_free(const Pose &pose);//test if the given point isn't in an obstacle

  // Workspace <-> Map conversions
  bool ws_to_map(double wx, double wy, int &mx, int &my);//converts from workspace to map
  void map_to_ws(int mx, int my, double &wx, double &wy);//converts from map to workspace

  //Next for are to get the bounderies 
  double get_min_x() const { return origin_x_; }
  double get_max_x() const { return origin_x_ + (width_ * resolution_); }
  
  double get_min_y() const { return origin_y_; }
  double get_max_y() const { return origin_y_ + (height_ * resolution_); }

private:
  int get_index(int mx, int my) const;//returns the index of the point in the occupancygrid

  State state_ = INSTANCIATED;
  OccupancyGrid map_;
  
  uint32_t width_;
  uint32_t height_;
  float resolution_;
  double origin_x_;
  double origin_y_;
};

#endif // RRT_CONNECT_PLANNER__MAP_HELPER_HPP
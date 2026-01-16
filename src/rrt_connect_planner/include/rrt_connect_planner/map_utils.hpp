#ifndef ROBMOB_MAP_UTILS_HPP
#define ROBMOB_MAP_UTILS_HPP

#include "robmob_interfaces/action/compute_path.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace geometry_msgs::msg;

namespace map
{
  float resolution;
  Point origin;
  bool initialized = false;
  bool is_goal_reachable(std::shared_ptr<const robmob_interfaces::action::ComputePath::Goal>){return true;}
  void init(float resol, Point orig){resolution = resol;origin=orig;initialized=true;}
  void workspace_to_cspace(Point &p){if(!initialized)return;p.x = (p.x - origin.x) / resolution;p.y = (p.y - origin.y) / resolution;}
  void cspace_to_workspace(Point &p){if(!initialized)return;p.x = p.x*resolution+origin.x;p.y = p.y*resolution+origin.y;}
  [[nodiscard]] OccupancyGrid inflate_map(const OccupancyGrid::UniquePtr msg, float radius){(void)radius;return *msg;}
}

#endif
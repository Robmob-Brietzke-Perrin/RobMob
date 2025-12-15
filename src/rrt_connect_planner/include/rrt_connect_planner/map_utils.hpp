#ifndef ROBMOB_MAP_UTILS_HPP
#define ROBMOB_MAP_UTILS_HPP

#include "robmob_interfaces/action/compute_path.hpp"

bool is_goal_reachable(std::shared_ptr<const robmob_interfaces::action::ComputePath::Goal>){return true;}
void inflate_map(float radius){(void)radius;}
void coord_to_pose(){}
void pose_to_coord(){}

#endif
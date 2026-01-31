#include "rrt_connect_planner/map_helper.hpp"

#include <iostream>

void MapHelper::initialize(
    const OccupancyGrid &map_msg)  // initialize the map with an occupancygrid
{
  map_ = map_msg;

  width_ = map_.info.width;
  height_ = map_.info.height;
  resolution_ = map_.info.resolution;
  origin_x_ = map_.info.origin.position.x;
  origin_y_ = map_.info.origin.position.y;

  state_ = INITIALIZED;
}

bool MapHelper::ws_to_map(double wx, double wy, int &mx,
                          int &my)  // converts from workspace to map
{
  if (state_ == INSTANCIATED) return false;

  mx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  my = static_cast<int>(std::floor((wy - origin_y_) / resolution_));

  if (mx < 0 || mx >= static_cast<int>(width_) || my < 0 ||
      my >= static_cast<int>(height_)) {
    return false;
  }
  return true;
}

void MapHelper::map_to_ws(int mx, int my, double &wx,
                          double &wy)  // converts from mapt to workspace
{
  // Return corresponding cell's center
  wx = (mx + 0.5) * resolution_ + origin_x_;
  wy = (my + 0.5) * resolution_ + origin_y_;
}

int MapHelper::get_index(int mx,
                         int my) const  // simple conversion to get the index
{
  return my * width_ + mx;
}

bool MapHelper::is_in_map(
    const Pose &pose)  // test if the point is in bounderies
{
  int mx, my;
  return ws_to_map(pose.position.x, pose.position.y, mx,
                   my);  // uses the conversion to test it
}

bool MapHelper::is_free(
    const Pose &pose)  // test if the point isn't in an obstacle
{
  int mx, my;
  if (!ws_to_map(pose.position.x, pose.position.y, mx,
                 my)) {  // uses conversion to test it
    return false;
  }

  int index = get_index(mx, my);
  int8_t value = map_.data[index];

  if (value != 0)  // FIXME: shoudl we accept unknown pose as valid? (value ==
                   // -1) / it shouldn't be possible (considered out of map)
  {
    return false;
  }
  return true;
}

void MapHelper::inflate_obstacles(
    float bot_radius)  // inflating the obstacle from the size of the robot
{
  if (state_ != INITIALIZED)
    return;  // checks if the map is ready to be inflated

  int cell_radius = std::ceil(bot_radius / resolution_);  // sets the radius

  std::vector<int8_t> original_data = map_.data;  // retrive map data
  for (int y = 0; y < static_cast<int>(height_);
       ++y)  // iterates on x and y to modify the right pixels at right range
  {
    for (int x = 0; x < static_cast<int>(width_); ++x) {
      int idx = get_index(x, y);
      if (original_data[idx] > 0)  // checks if it used to be an obstacle
      {
        for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
          for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
            if (dx * dx + dy * dy > cell_radius * cell_radius) continue;

            int nx = x + dx;
            int ny = y + dy;

            if (nx >= 0 && nx < static_cast<int>(width_) && ny >= 0 &&
                ny < static_cast<int>(height_)) {
              map_.data[get_index(nx, ny)] = 100;
            }
          }
        }
      }
    }
  }

  state_ = INFLATED;
}
#include "rrt_connect_planner/map_helper.hpp"
#include <iostream>

void MapHelper::initialize(const nav_msgs::msg::OccupancyGrid &map_msg)
{
  map_ = map_msg;
  
  width_ = map_.info.width;
  height_ = map_.info.height;
  resolution_ = map_.info.resolution;
  origin_x_ = map_.info.origin.position.x;
  origin_y_ = map_.info.origin.position.y;

  state_ = INITIALIZED;
}

bool MapHelper::ws_to_map(double wx, double wy, int &mx, int &my)
{
  if (state_ == INSTANCIATED) return false;

  mx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  my = static_cast<int>(std::floor((wy - origin_y_) / resolution_));

  // if (mx < 0 || mx >= static_cast<int>(width_) || 
  //     my < 0 || my >= static_cast<int>(height_)) {
  //     RCLCPP_INFO(this->get_logger(), "out of bounds");
  //   return false;
  // }
  return true;
}

void MapHelper::map_to_ws(int mx, int my, double &wx, double &wy)
{
  wx = (mx + 0.5) * resolution_ + origin_x_;
  wy = (my + 0.5) * resolution_ + origin_y_;
}

int MapHelper::get_index(int mx, int my) const
{
  return my * width_ + mx;
}

bool MapHelper::is_free(double x, double y)
{
  int mx, my;
  if (!ws_to_map(x, y, mx, my)) {
    return false;
  }

  int index = get_index(mx, my);
  int8_t value = map_.data[index];

  if (value == 0 || value == -1) { // Accept unknown
      return true; 
  }
  return false;
}

bool MapHelper::is_free(const geometry_msgs::msg::Pose &pose)
{
    return is_free(pose.position.x, pose.position.y);
}

bool MapHelper::is_in_map(const geometry_msgs::msg::Pose &pose)
{
  int mx, my;
  return ws_to_map(pose.position.x, pose.position.y, mx, my);
}

void MapHelper::inflate_obstacles(float bot_radius)
{
  if(state_ != INITIALIZED) return; // Prevent multiple inflations

  int cell_radius = std::ceil(bot_radius / resolution_);

  // Copy to always check on the original map
  std::vector<int8_t> original_data = map_.data;
  
  int w = static_cast<int>(width_);
  int h = static_cast<int>(height_);

  for (int y = 0; y < h; ++y)
  {
    for (int x = 0; x < w; ++x)
    {
      int idx = get_index(x, y);
      
      // If the cell is originally an obstacle
      if (original_data[idx] > 0)
      {
        // Then we fill out a circle around it in the new map
        for (int dy = -cell_radius; dy <= cell_radius; ++dy)
        {
          for (int dx = -cell_radius; dx <= cell_radius; ++dx)
          {
            if (dx*dx + dy*dy > cell_radius*cell_radius) continue; // Square's corners (not in circle)

            int nx = x + dx;
            int ny = y + dy;

            if (nx >= 0 && nx < w && ny >= 0 && ny < h) // Still check if in bounds
            {
              map_.data[get_index(nx, ny)] = 1; // Update to obstacle
            }
          }
        }
      }
    }
  }
  // Update map state
  state_ = INFLATED;
}
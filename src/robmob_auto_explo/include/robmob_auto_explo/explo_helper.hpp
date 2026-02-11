#ifndef ROBMOB_AUTO_EXPLO__EXPLO_HELPER_HPP
#define ROBMOB_AUTO_EXPLO__EXPLO_HELPER_HPP

#include <vector>
#include <cmath>
#include <optional>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


// FIXME: mauvais scope
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using LaserScan = sensor_msgs::msg::LaserScan;

class ExploHelper {
public:
    struct Sector {
        double angle;
        double avg_distance;
        bool is_unknown;
    };

    /**
     * Calcule la meilleure direction basée sur le LiDAR et la carte.
     */
    static std::optional<double> getBestDirection(
        const LaserScan::SharedPtr scan,
        const OccupancyGrid::SharedPtr map,
        double robot_x, double robot_y, double robot_yaw,
        double projection_dist,
        double last_angle);
        
private:

    static double getAvgDistInSector(const LaserScan::SharedPtr scan, int start_idx, int end_idx);
    
    static bool isPointUnknown(const OccupancyGrid::SharedPtr map, double x, double y);
};

#endif // ROBMOB_AUTO_EXPLO__EXPLO_HELPER_HPP
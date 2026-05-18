#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    // (Re)configure the costmap geometry and inflation behaviour.
    void initCostmap(double resolution,
                     int width,
                     int height,
                     double origin_x,
                     double origin_y,
                     double inflation_radius,
                     int max_cost);

    // Reset the underlying grid to all-free (0). Called once per scan.
    void resetGrid();

    // Convert a single laser-scan return into a marked obstacle cell.
    void markObstacleFromRange(double angle, double range);

    // Inflate every obstacle cell outward using a linear cost decay.
    void inflateObstacles();

    // Package the working grid into a ROS OccupancyGrid for publishing.
    void toOccupancyGrid(nav_msgs::msg::OccupancyGrid& msg) const;

    double resolution() const { return resolution_; }
    int width() const { return width_; }
    int height() const { return height_; }
    double originX() const { return origin_x_; }
    double originY() const { return origin_y_; }

  private:
    bool worldToGrid(double x, double y, int& gx, int& gy) const;

    rclcpp::Logger logger_;

    double resolution_ = 0.1;
    int width_ = 100;
    int height_ = 100;
    double origin_x_ = -5.0;
    double origin_y_ = -5.0;
    double inflation_radius_ = 1.0;
    int max_cost_ = 100;

    // Row-major grid of size width_ * height_ holding cost values in [0, 100].
    std::vector<int8_t> grid_;
};

}  

#endif  

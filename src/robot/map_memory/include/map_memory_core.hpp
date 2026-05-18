#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Initialize the global map. The grid is centred around (0, 0) in the
    // global frame and pre-filled with -1 (unknown).
    void initMap(double resolution,
                 int width,
                 int height,
                 const std::string& frame_id);

    // Merge the local costmap into the global map using the robot's current
    // pose (x, y, yaw). Cells outside the global map are ignored. Cells in
    // the costmap that are 0 (unknown to costmap) leave the global map cell
    // unchanged; otherwise, the higher cost wins.
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                          double robot_x,
                          double robot_y,
                          double robot_yaw);

    const nav_msgs::msg::OccupancyGrid& map() const { return map_; }
    nav_msgs::msg::OccupancyGrid& mutableMap() { return map_; }

  private:
    bool worldToGrid(double x, double y, int& gx, int& gy) const;

    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid map_;
};

}  

#endif  

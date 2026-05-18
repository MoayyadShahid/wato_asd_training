#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <functional>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot
{

// 2D grid index used by A*.
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex& other) const {
    return (x == other.x && y == other.y);
  }
  bool operator!=(const CellIndex& other) const {
    return (x != other.x || y != other.y);
  }
};

struct CellIndexHash
{
  std::size_t operator()(const CellIndex& idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // Run A* on `map` between world coordinates (start_*) and (goal_*).
    // Returns the path (sequence of world (x, y) waypoints) on success.
    // Returns an empty vector if no path exists.
    std::vector<std::pair<double, double>> planPath(
        const nav_msgs::msg::OccupancyGrid& map,
        double start_x, double start_y,
        double goal_x, double goal_y,
        int obstacle_threshold = 50) const;

  private:
    rclcpp::Logger logger_;
};

}  

#endif  

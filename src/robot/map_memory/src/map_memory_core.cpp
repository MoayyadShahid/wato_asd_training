#include "map_memory_core.hpp"

#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void MapMemoryCore::initMap(double resolution,
                            int width,
                            int height,
                            const std::string& frame_id) {
  map_.header.frame_id = frame_id;
  map_.info.resolution = static_cast<float>(resolution);
  map_.info.width = static_cast<unsigned int>(width);
  map_.info.height = static_cast<unsigned int>(height);
  map_.info.origin.position.x = -(width * resolution) / 2.0;
  map_.info.origin.position.y = -(height * resolution) / 2.0;
  map_.info.origin.position.z = 0.0;
  map_.info.origin.orientation.w = 1.0;
  // -1 == unknown for nav_msgs/OccupancyGrid.
  map_.data.assign(static_cast<size_t>(width * height), -1);
}

bool MapMemoryCore::worldToGrid(double x, double y, int& gx, int& gy) const {
  const double res = map_.info.resolution;
  gx = static_cast<int>(std::floor((x - map_.info.origin.position.x) / res));
  gy = static_cast<int>(std::floor((y - map_.info.origin.position.y) / res));
  return (gx >= 0 && gx < static_cast<int>(map_.info.width) &&
          gy >= 0 && gy < static_cast<int>(map_.info.height));
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                                     double robot_x,
                                     double robot_y,
                                     double robot_yaw) {
  if (map_.data.empty() || costmap.data.empty()) {
    return;
  }

  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);

  const double cm_res = costmap.info.resolution;
  const int cm_w = static_cast<int>(costmap.info.width);
  const int cm_h = static_cast<int>(costmap.info.height);
  const double cm_origin_x = costmap.info.origin.position.x;
  const double cm_origin_y = costmap.info.origin.position.y;

  for (int cy = 0; cy < cm_h; ++cy) {
    for (int cx = 0; cx < cm_w; ++cx) {
      const int8_t cost = costmap.data[cy * cm_w + cx];
      if (cost < 0) {
        continue;  // costmap cell is unknown; keep whatever the map had.
      }

      // Centre of this costmap cell in the costmap (robot) frame.
      const double lx = cm_origin_x + (cx + 0.5) * cm_res;
      const double ly = cm_origin_y + (cy + 0.5) * cm_res;

      // Transform into the global frame using the robot's pose.
      const double gx_world = robot_x + cos_yaw * lx - sin_yaw * ly;
      const double gy_world = robot_y + sin_yaw * lx + cos_yaw * ly;

      int gx, gy;
      if (!worldToGrid(gx_world, gy_world, gx, gy)) {
        continue;
      }

      int8_t& cell = map_.data[gy * map_.info.width + gx];
      if (cell < 0) {
        // First time we observe this cell -> just write what we saw.
        cell = cost;
      } else if (cost > cell) {
        // Prefer the higher cost so previously detected obstacles stay
        // present even if the latest scan didn't see them.
        cell = cost;
      }
    }
  }
}

}  // namespace robot

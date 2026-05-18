#include "costmap_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::initCostmap(double resolution,
                              int width,
                              int height,
                              double origin_x,
                              double origin_y,
                              double inflation_radius,
                              int max_cost) {
  resolution_ = resolution;
  width_ = width;
  height_ = height;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  inflation_radius_ = inflation_radius;
  max_cost_ = max_cost;
  grid_.assign(static_cast<size_t>(width_ * height_), 0);
}

void CostmapCore::resetGrid() {
  std::fill(grid_.begin(), grid_.end(), 0);
}

bool CostmapCore::worldToGrid(double x, double y, int& gx, int& gy) const {
  gx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  gy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return (gx >= 0 && gx < width_ && gy >= 0 && gy < height_);
}

void CostmapCore::markObstacleFromRange(double angle, double range) {
  const double x = range * std::cos(angle);
  const double y = range * std::sin(angle);
  int gx, gy;
  if (!worldToGrid(x, y, gx, gy)) {
    return;
  }
  grid_[gy * width_ + gx] = static_cast<int8_t>(max_cost_);
}

void CostmapCore::inflateObstacles() {
  // Snapshot the obstacle cells so newly inflated cells don't re-inflate.
  std::vector<std::pair<int, int>> obstacles;
  obstacles.reserve(grid_.size() / 8);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (grid_[y * width_ + x] >= static_cast<int8_t>(max_cost_)) {
        obstacles.emplace_back(x, y);
      }
    }
  }

  const int radius_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
  for (const auto& [ox, oy] : obstacles) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const int nx = ox + dx;
        const int ny = oy + dy;
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
          continue;
        }
        const double dist = std::hypot(dx, dy) * resolution_;
        if (dist > inflation_radius_) {
          continue;
        }
        const int cost = static_cast<int>(
            max_cost_ * (1.0 - dist / inflation_radius_));
        const int idx = ny * width_ + nx;
        if (cost > grid_[idx]) {
          grid_[idx] = static_cast<int8_t>(cost);
        }
      }
    }
  }
}

void CostmapCore::toOccupancyGrid(nav_msgs::msg::OccupancyGrid& msg) const {
  msg.info.resolution = static_cast<float>(resolution_);
  msg.info.width = static_cast<unsigned int>(width_);
  msg.info.height = static_cast<unsigned int>(height_);
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(grid_.begin(), grid_.end());
}

}  // namespace robot

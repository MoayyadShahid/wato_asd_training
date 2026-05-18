#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace robot
{

namespace {

struct AStarNode {
  CellIndex index;
  double f_score;
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) const {
    return a.f_score > b.f_score;
  }
};

inline bool worldToGrid(const nav_msgs::msg::OccupancyGrid& map,
                        double x, double y, int& gx, int& gy) {
  const double res = map.info.resolution;
  gx = static_cast<int>(std::floor((x - map.info.origin.position.x) / res));
  gy = static_cast<int>(std::floor((y - map.info.origin.position.y) / res));
  return (gx >= 0 && gx < static_cast<int>(map.info.width) &&
          gy >= 0 && gy < static_cast<int>(map.info.height));
}

inline std::pair<double, double> gridToWorld(
    const nav_msgs::msg::OccupancyGrid& map, int gx, int gy) {
  return {
      map.info.origin.position.x + (gx + 0.5) * map.info.resolution,
      map.info.origin.position.y + (gy + 0.5) * map.info.resolution};
}

inline double heuristic(const CellIndex& a, const CellIndex& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

}  // namespace

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

std::vector<std::pair<double, double>> PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid& map,
    double start_x, double start_y,
    double goal_x, double goal_y,
    int obstacle_threshold) const {
  std::vector<std::pair<double, double>> result;

  if (map.data.empty() || map.info.width == 0 || map.info.height == 0) {
    return result;
  }

  int sx, sy, gx, gy;
  if (!worldToGrid(map, start_x, start_y, sx, sy) ||
      !worldToGrid(map, goal_x, goal_y, gx, gy)) {
    RCLCPP_WARN(logger_, "Planner: start or goal outside the map bounds.");
    return result;
  }

  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);

  auto cellCost = [&](int x, int y) -> int {
    return static_cast<int>(map.data[y * width + x]);
  };

  // Only treat fully occupied cells (cost >= 100) as hard blockers; lower
  // costs (including inflation) are walked over with a soft penalty so the
  // robot can plan out of inflated zones if it has to.
  auto isHardBlocked = [&](int x, int y) {
    return cellCost(x, y) >= 100;
  };

  if (isHardBlocked(gx, gy)) {
    RCLCPP_WARN(logger_, "Planner: goal cell is occupied; aborting.");
    return result;
  }

  CellIndex start(sx, sy);
  CellIndex goal(gx, gy);

  if (start == goal) {
    result.emplace_back(start_x, start_y);
    return result;
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_set<CellIndex, CellIndexHash> closed;

  g_score[start] = 0.0;
  open.emplace(start, heuristic(start, goal));

  static const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  static const double step8[8] = {
      1.0, 1.0, 1.0, 1.0,
      M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2};

  bool found = false;
  while (!open.empty()) {
    AStarNode current = open.top();
    open.pop();

    if (closed.count(current.index)) {
      continue;
    }
    closed.insert(current.index);

    if (current.index == goal) {
      found = true;
      break;
    }

    for (int i = 0; i < 8; ++i) {
      const int nx = current.index.x + dx8[i];
      const int ny = current.index.y + dy8[i];
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }
      if (isHardBlocked(nx, ny)) {
        continue;
      }
      const CellIndex neighbour(nx, ny);
      if (closed.count(neighbour)) {
        continue;
      }

      // Step cost: base distance plus a soft cost from the underlying
      // grid value, so the planner prefers low-cost lanes but can still
      // walk through inflated regions if it has to.
      const int raw = cellCost(nx, ny);
      double soft_cost = 0.0;
      if (raw >= obstacle_threshold) {
        soft_cost = 20.0;  // strongly discourage but don't forbid
      } else if (raw > 0) {
        soft_cost = raw / 25.0;
      }
      const double tentative_g = g_score[current.index] + step8[i] + soft_cost;

      auto it = g_score.find(neighbour);
      if (it == g_score.end() || tentative_g < it->second) {
        g_score[neighbour] = tentative_g;
        came_from[neighbour] = current.index;
        const double f = tentative_g + heuristic(neighbour, goal);
        open.emplace(neighbour, f);
      }
    }
  }

  if (!found) {
    return result;
  }

  // Reconstruct the path.
  std::vector<CellIndex> cells;
  CellIndex cur = goal;
  cells.push_back(cur);
  while (cur != start) {
    auto it = came_from.find(cur);
    if (it == came_from.end()) {
      return {};
    }
    cur = it->second;
    cells.push_back(cur);
  }
  std::reverse(cells.begin(), cells.end());

  result.reserve(cells.size());
  for (const auto& c : cells) {
    result.push_back(gridToWorld(map, c.x, c.y));
  }
  // Ensure the very last waypoint matches the requested goal exactly.
  if (!result.empty()) {
    result.back() = {goal_x, goal_y};
  }
  return result;
}

}  // namespace robot

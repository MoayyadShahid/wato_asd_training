#include "control_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
    const nav_msgs::msg::Path& path,
    double robot_x, double robot_y,
    double lookahead_distance) const {
  if (path.poses.empty()) {
    return std::nullopt;
  }

  // Find the path waypoint closest to the robot; this is our starting
  // search index for the lookahead point.
  size_t nearest_idx = 0;
  double nearest_d = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - robot_x;
    const double dy = path.poses[i].pose.position.y - robot_y;
    const double d = std::hypot(dx, dy);
    if (d < nearest_d) {
      nearest_d = d;
      nearest_idx = i;
    }
  }

  for (size_t i = nearest_idx; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - robot_x;
    const double dy = path.poses[i].pose.position.y - robot_y;
    if (std::hypot(dx, dy) >= lookahead_distance) {
      return path.poses[i];
    }
  }

  // Robot has run out of "ahead" waypoints; return the final pose so the
  // controller drives the last short leg to the goal.
  return path.poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
    const geometry_msgs::msg::PoseStamped& target,
    double robot_x, double robot_y, double robot_yaw,
    double linear_speed,
    double max_angular_speed) const {
  geometry_msgs::msg::Twist cmd_vel;

  const double dx = target.pose.position.x - robot_x;
  const double dy = target.pose.position.y - robot_y;

  // Bearing to target relative to the robot's heading.
  const double target_bearing = std::atan2(dy, dx);
  double heading_error = target_bearing - robot_yaw;
  // Normalise into [-pi, pi].
  while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
  while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

  const double distance = std::hypot(dx, dy);

  // Pure-pursuit curvature: kappa = 2*sin(alpha)/L, omega = v*kappa.
  // Using the actual distance to the target as the lookahead L works well
  // when the target was selected via findLookaheadPoint above.
  double omega = 0.0;
  if (distance > 1e-3) {
    const double curvature = 2.0 * std::sin(heading_error) / distance;
    omega = linear_speed * curvature;
  }
  omega = std::clamp(omega, -max_angular_speed, max_angular_speed);

  // If the target is sharply behind us, prefer to rotate in place first
  // so we don't drive away from the path.
  double v = linear_speed;
  if (std::fabs(heading_error) > M_PI / 2.0) {
    v = 0.0;
    omega = std::copysign(max_angular_speed, heading_error);
  }

  cmd_vel.linear.x = v;
  cmd_vel.angular.z = omega;
  return cmd_vel;
}

}  // namespace robot

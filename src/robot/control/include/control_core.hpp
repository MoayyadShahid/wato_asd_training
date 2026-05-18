#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

class ControlCore {
  public:
    ControlCore(const rclcpp::Logger& logger);

    // Pick a lookahead waypoint on `path` that is at least `lookahead_distance`
    // meters away from the robot. If the robot is already past the end of the
    // path, returns std::nullopt.
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
        const nav_msgs::msg::Path& path,
        double robot_x, double robot_y,
        double lookahead_distance) const;

    // Compute a Twist command that steers the robot toward `target` from its
    // current pose using a pure-pursuit law.
    geometry_msgs::msg::Twist computeVelocity(
        const geometry_msgs::msg::PoseStamped& target,
        double robot_x, double robot_y, double robot_yaw,
        double linear_speed,
        double max_angular_speed) const;

    static double extractYaw(const geometry_msgs::msg::Quaternion& q);

  private:
    rclcpp::Logger logger_;
};

} 

#endif 

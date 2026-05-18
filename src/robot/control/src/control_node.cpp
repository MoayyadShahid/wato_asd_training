#include "control_node.hpp"

#include <chrono>
#include <cmath>

ControlNode::ControlNode()
: Node("control"),
  control_(robot::ControlCore(this->get_logger())) {
  this->declare_parameter("lookahead_distance", 1.0);
  this->declare_parameter("goal_tolerance", 0.3);
  this->declare_parameter("linear_speed", 0.5);
  this->declare_parameter("max_angular_speed", 1.5);
  this->declare_parameter("control_period_ms", 100);
  this->declare_parameter("path_topic", std::string("/path"));
  this->declare_parameter("odom_topic", std::string("/odom/filtered"));
  this->declare_parameter("cmd_vel_topic", std::string("/cmd_vel"));

  lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  linear_speed_ = this->get_parameter("linear_speed").as_double();
  max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
  const int period_ms = this->get_parameter("control_period_ms").as_int();
  const std::string path_topic = this->get_parameter("path_topic").as_string();
  const std::string odom_topic = this->get_parameter("odom_topic").as_string();
  const std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      path_topic, 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::controlLoop() {
  if (!robot_odom_) {
    return;
  }

  // No path yet, or planner explicitly cleared the path -> stop.
  if (!current_path_ || current_path_->poses.empty()) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  const double robot_x = robot_odom_->pose.pose.position.x;
  const double robot_y = robot_odom_->pose.pose.position.y;
  const double robot_yaw = robot::ControlCore::extractYaw(
      robot_odom_->pose.pose.orientation);

  // If we are within tolerance of the path's final pose, stop.
  const auto& goal_pose = current_path_->poses.back().pose.position;
  const double dist_to_goal =
      std::hypot(goal_pose.x - robot_x, goal_pose.y - robot_y);
  if (dist_to_goal < goal_tolerance_) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  auto lookahead = control_.findLookaheadPoint(
      *current_path_, robot_x, robot_y, lookahead_distance_);
  if (!lookahead) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  auto cmd_vel = control_.computeVelocity(
      *lookahead, robot_x, robot_y, robot_yaw,
      linear_speed_, max_angular_speed_);
  cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

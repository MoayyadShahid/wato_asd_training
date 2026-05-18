#include "planner_node.hpp"

#include <chrono>
#include <cmath>

#include "geometry_msgs/msg/pose_stamped.hpp"

PlannerNode::PlannerNode()
: Node("planner"),
  planner_(robot::PlannerCore(this->get_logger())) {
  this->declare_parameter("map_topic", std::string("/map"));
  this->declare_parameter("goal_topic", std::string("/goal_point"));
  this->declare_parameter("odom_topic", std::string("/odom/filtered"));
  this->declare_parameter("path_topic", std::string("/path"));
  this->declare_parameter("map_frame", std::string("sim_world"));
  this->declare_parameter("goal_tolerance", 0.5);
  this->declare_parameter("goal_timeout_seconds", 30.0);
  this->declare_parameter("obstacle_threshold", 50);
  this->declare_parameter("replan_period_ms", 500);

  const std::string map_topic = this->get_parameter("map_topic").as_string();
  const std::string goal_topic = this->get_parameter("goal_topic").as_string();
  const std::string odom_topic = this->get_parameter("odom_topic").as_string();
  const std::string path_topic = this->get_parameter("path_topic").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  goal_timeout_seconds_ = this->get_parameter("goal_timeout_seconds").as_double();
  obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_int();
  const int replan_period_ms = this->get_parameter("replan_period_ms").as_int();

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, 10,
      std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      goal_topic, 10,
      std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(replan_period_ms),
      std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  has_map_ = true;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planAndPublish();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  has_goal_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  goal_start_time_ = this->now();
  RCLCPP_INFO(this->get_logger(),
              "Planner: received new goal (%.2f, %.2f).",
              goal_.point.x, goal_.point.y);
  planAndPublish();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  has_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }
  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Planner: goal reached.");
    state_ = State::WAITING_FOR_GOAL;
    // Publish an empty path so the controller stops.
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->now();
    empty_path.header.frame_id = map_frame_;
    path_pub_->publish(empty_path);
    return;
  }

  const double elapsed = (this->now() - goal_start_time_).seconds();
  if (elapsed > goal_timeout_seconds_) {
    RCLCPP_WARN(this->get_logger(),
                "Planner: goal timed out after %.1f s; abandoning.",
                elapsed);
    state_ = State::WAITING_FOR_GOAL;
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->now();
    empty_path.header.frame_id = map_frame_;
    path_pub_->publish(empty_path);
    return;
  }

  planAndPublish();
}

bool PlannerNode::goalReached() const {
  if (!has_goal_ || !has_odom_) {
    return false;
  }
  const double dx = goal_.point.x - robot_x_;
  const double dy = goal_.point.y - robot_y_;
  return std::hypot(dx, dy) < goal_tolerance_;
}

void PlannerNode::planAndPublish() {
  if (!has_map_ || !has_goal_ || !has_odom_) {
    return;
  }

  auto waypoints = planner_.planPath(
      current_map_, robot_x_, robot_y_,
      goal_.point.x, goal_.point.y, obstacle_threshold_);

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = map_frame_;

  if (waypoints.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Planner: no path found to goal.");
    path_pub_->publish(path);
    return;
  }

  path.poses.reserve(waypoints.size());
  for (const auto& [x, y] : waypoints) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

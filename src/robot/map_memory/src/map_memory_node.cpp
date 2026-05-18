#include "map_memory_node.hpp"

#include <chrono>
#include <cmath>

namespace {

double yawFromQuaternion(double x, double y, double z, double w) {
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

MapMemoryNode::MapMemoryNode()
: Node("map_memory"),
  map_memory_(robot::MapMemoryCore(this->get_logger())) {
  this->declare_parameter("resolution", 0.3);
  this->declare_parameter("width", 200);
  this->declare_parameter("height", 200);
  this->declare_parameter("frame_id", std::string("sim_world"));
  this->declare_parameter("update_distance", 1.5);
  this->declare_parameter("publish_period_ms", 1000);
  this->declare_parameter("costmap_topic", std::string("/costmap"));
  this->declare_parameter("odom_topic", std::string("/odom/filtered"));
  this->declare_parameter("map_topic", std::string("/map"));

  const double resolution = this->get_parameter("resolution").as_double();
  const int width = this->get_parameter("width").as_int();
  const int height = this->get_parameter("height").as_int();
  const std::string frame_id = this->get_parameter("frame_id").as_string();
  update_distance_ = this->get_parameter("update_distance").as_double();
  const int publish_period_ms = this->get_parameter("publish_period_ms").as_int();
  const std::string costmap_topic = this->get_parameter("costmap_topic").as_string();
  const std::string odom_topic = this->get_parameter("odom_topic").as_string();
  const std::string map_topic = this->get_parameter("map_topic").as_string();

  map_memory_.initMap(resolution, width, height, frame_id);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic, 10,
      std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_period_ms),
      std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  has_costmap_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_yaw_ = yawFromQuaternion(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
  has_odom_ = true;

  if (!has_last_pose_) {
    last_update_x_ = robot_x_;
    last_update_y_ = robot_y_;
    has_last_pose_ = true;
    should_update_map_ = true;
    return;
  }

  const double dx = robot_x_ - last_update_x_;
  const double dy = robot_y_ - last_update_y_;
  if (std::hypot(dx, dy) >= update_distance_) {
    last_update_x_ = robot_x_;
    last_update_y_ = robot_y_;
    should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap() {
  // Integrate the latest local costmap into the global map whenever we
  // have one. The 1.5 m distance gate from the assignment is enforced
  // implicitly by `should_update_map_`, but we keep the map fresh on the
  // very first costmap so downstream nodes have something to plan with.
  if (has_costmap_ && has_odom_ && (should_update_map_ || !map_published_once_)) {
    map_memory_.integrateCostmap(latest_costmap_, robot_x_, robot_y_, robot_yaw_);
    should_update_map_ = false;
    map_published_once_ = true;
  }

  auto& map = map_memory_.mutableMap();
  map.header.stamp = this->get_clock()->now();
  map_pub_->publish(map);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}

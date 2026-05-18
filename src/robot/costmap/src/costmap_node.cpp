#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
: Node("costmap"),
  costmap_(robot::CostmapCore(this->get_logger())) {
  this->declare_parameter("resolution", 0.1);
  this->declare_parameter("width", 200);
  this->declare_parameter("height", 200);
  this->declare_parameter("inflation_radius", 1.0);
  this->declare_parameter("max_cost", 100);
  this->declare_parameter("lidar_topic", std::string("/lidar"));
  this->declare_parameter("costmap_topic", std::string("/costmap"));

  const double resolution = this->get_parameter("resolution").as_double();
  const int width = this->get_parameter("width").as_int();
  const int height = this->get_parameter("height").as_int();
  const double inflation_radius =
      this->get_parameter("inflation_radius").as_double();
  const int max_cost = this->get_parameter("max_cost").as_int();
  const std::string lidar_topic =
      this->get_parameter("lidar_topic").as_string();
  const std::string costmap_topic =
      this->get_parameter("costmap_topic").as_string();

  // Centre the local costmap on the robot, since /lidar arrives in the
  // robot frame.
  const double origin_x = -(width * resolution) / 2.0;
  const double origin_y = -(height * resolution) / 2.0;

  costmap_.initCostmap(resolution,
                       width,
                       height,
                       origin_x,
                       origin_y,
                       inflation_radius,
                       max_cost);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      lidar_topic, 10,
      std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic, 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  costmap_.resetGrid();

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    const double range = scan->ranges[i];
    if (!std::isfinite(range) || range < scan->range_min ||
        range > scan->range_max) {
      continue;
    }
    const double angle = scan->angle_min + i * scan->angle_increment;
    costmap_.markObstacleFromRange(angle, range);
  }

  costmap_.inflateObstacles();

  nav_msgs::msg::OccupancyGrid msg;
  costmap_.toOccupancyGrid(msg);
  msg.header.stamp = scan->header.stamp;
  // Costmap lives in the lidar's local frame; map_memory transforms it
  // into the global sim_world frame.
  msg.header.frame_id = scan->header.frame_id;
  costmap_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}

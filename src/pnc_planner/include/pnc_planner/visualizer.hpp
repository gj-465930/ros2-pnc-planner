#pragma once

#include "pnc_planner/common.hpp"
#include "pnc_planner/msg/obstacle_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <vector>

namespace pnc_planner
{

class Visualizer
{
public:
  explicit Visualizer(rclcpp::Node & node);

  // 用mark消息画参考线
  void publishReferenceLineMarker(const std::vector<geometry_msgs::msg::Point> & points) const;

  // 用path消息画参考线
  void publishReferenceLine(const nav_msgs::msg::Path & path) const;

  void publishTrajectory(const nav_msgs::msg::Path & path) const;

  void publishStaticObstacles(const pnc_planner::msg::ObstacleArray & obstacles_array) const;

private:
  rclcpp::Node & node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_marker_pub_;
};

}  // namespace pnc_planner

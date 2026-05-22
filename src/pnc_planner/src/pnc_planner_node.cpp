#include <cmath>

#include "nav_msgs/msg/path.hpp"
#include "pnc_planner/pnc_planner_node.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace pnc_planner {
PncPlannerNode::PncPlannerNode(const std::string &node_name) : Node(node_name) {

  // 初始化自车
  ego_vehicle_ = std::make_shared<EgoVehicle>(this);
  ego_vehicle_->setCommand(0.2, 0.2);
  ego_vehicle_->setPose(0.0, 0.0, 0.0);
  // 初始化参考线对象
  ref_line_ = std::make_shared<ReferenceLine>();

  this->declare_parameter<bool>("use_mock_routing", false);
  bool use_mock;
  this->get_parameter("use_mock_routing", use_mock);

  if (use_mock) {
    RCLCPP_WARN(this->get_logger(), "正在使用mock数据测试");
    std::vector<geometry_msgs::msg::Point> test_points;
    for (int i = 0; i < 5; i++) {
      geometry_msgs::msg::Point p;
      p.x = i * 5.0;
      p.y = i * i * 1.0; // 抛物线
      p.z = 0.0;
      test_points.push_back(p);
    }
    this->updateReferenceLine(test_points);
  }

  visualizer_ = std::make_shared<Visualizer>(this);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                   [this]() { this->timerCallback(); });
}

void PncPlannerNode::timerCallback() {
  // this->testSinPathVisual();
  this->testEgoVehicle();
  this->publishReferenceLine();
}

void PncPlannerNode::testSinPathVisual() {
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = this->now();

  for (double x = 0.0; x <= 10.0; x += 0.1) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();

    pose.pose.position.x = x;
    pose.pose.position.y = std::sin(x);
    pose.pose.position.z = 0.0;

    double yaw = std::atan2(std::cos(x), 1.0);

    tf2::Quaternion qtn;
    qtn.setRPY(0.0, 0.0, yaw);

    pose.pose.orientation.x = qtn.getX();
    pose.pose.orientation.y = qtn.getY();
    pose.pose.orientation.z = qtn.getZ();
    pose.pose.orientation.w = qtn.getW();

    path.poses.push_back(pose);
  }

  if (visualizer_ != nullptr) {
    visualizer_->publishReferenceLine(path);
  }
}

void PncPlannerNode::testEgoVehicle() {
  ego_vehicle_->updateState(0.1);

  VehicleInfo state = ego_vehicle_->getVehicleState();

  RCLCPP_INFO(this->get_logger(),
              "x = %6.2f, y = %6.2f, yaw = %6.2f, v = %6.2f,", state.pose.x,
              state.pose.y, state.pose.yaw, state.v);
}

void PncPlannerNode::updateReferenceLine(
    const std::vector<geometry_msgs::msg::Point> &points) {
  std::vector<double> x, y;
  for (auto point : points) {
    x.push_back(point.x);
    y.push_back(point.y);
  }

  if (!ref_line_->init(x, y)) {
    RCLCPP_WARN(this->get_logger(), "参考线初始化失败!");
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "初始化参考线成功，总长度为 %.2f",
                ref_line_->getTotalLength());
  }
}

void PncPlannerNode::publishReferenceLine() {
  if (ref_line_ == nullptr || ref_line_->getTotalLength() < 0.0) {
    return;
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = this->now();

  double max_s = ref_line_->getTotalLength();

  for (double s = 0.0; s <= max_s; s += 0.1) {
    WayPoint wp = ref_line_->getWayPoint(s);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();

    pose.pose.position.x = wp.x;
    pose.pose.position.y = wp.y;
    pose.pose.position.z = 0.0;

    tf2::Quaternion qtn;
    qtn.setRPY(0.0, 0.0, wp.heading);
    pose.pose.orientation.x = qtn.x();
    pose.pose.orientation.y = qtn.y();
    pose.pose.orientation.z = qtn.z();
    pose.pose.orientation.w = qtn.w();

    path.poses.push_back(pose);
  }
  visualizer_->publishReferenceLine(path);
}

} // namespace pnc_planner
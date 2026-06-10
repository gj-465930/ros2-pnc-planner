#include "pnc_planner/pnc_planner_node.hpp"

#include "tf2/LinearMath/Quaternion.hpp"

#include "nav_msgs/msg/path.hpp"

#include <cmath>

namespace pnc_planner
{
PncPlannerNode::PncPlannerNode(const std::string & node_name) : Node(node_name)
{
  // 声明参数
  // limits
  declare_parameter("lattice_planner.limits.max_v", 35.0);
  declare_parameter("lattice_planner.limits.min_v", -0.1);
  declare_parameter("lattice_planner.limits.max_acc", 3.0);
  declare_parameter("lattice_planner.limits.min_acc", -5.0);
  declare_parameter("lattice_planner.limits.max_jerk", 4.0);
  declare_parameter("lattice_planner.limits.max_lat_offset", 3.5);
  declare_parameter("lattice_planner.limits.target_speed", 15.0);
  declare_parameter("lattice_planner.limits.planning_time", 5.0);
  // weights
  declare_parameter("lattice_planner.weights.w_lat", 1.0);
  declare_parameter("lattice_planner.weights.w_lon", 10.0);
  declare_parameter("lattice_planner.weights.w_offset", 0.3);
  declare_parameter("lattice_planner.weights.w_speed", 1.0);

  // 读取参数
  LatticePlannerConfig config;
  config.max_v = get_parameter("lattice_planner.limits.max_v").as_double();
  config.min_v = get_parameter("lattice_planner.limits.min_v").as_double();
  config.max_acc = get_parameter("lattice_planner.limits.max_acc").as_double();
  config.min_acc = get_parameter("lattice_planner.limits.min_acc").as_double();
  config.max_jerk = get_parameter("lattice_planner.limits.max_jerk").as_double();
  config.max_lat_offset = get_parameter("lattice_planner.limits.max_lat_offset").as_double();
  config.target_speed = get_parameter("lattice_planner.limits.target_speed").as_double();
  config.planning_time = get_parameter("lattice_planner.limits.planning_time").as_double();

  config.w_lat = get_parameter("lattice_planner.weights.w_lat").as_double();
  config.w_lon = get_parameter("lattice_planner.weights.w_lon").as_double();
  config.w_offset = get_parameter("lattice_planner.weights.w_offset").as_double();
  config.w_speed = get_parameter("lattice_planner.weights.w_speed").as_double();

  lattice_planner_ = std::make_shared<LatticePlanner>(config);

  // 初始化自车
  ego_vehicle_ = std::make_shared<EgoVehicle>(this);
  ego_vehicle_->setCommand(0.2, 0.2, 0.0);
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
      p.y = i * i * 1.0;  // 抛物线
      p.z = 0.0;
      test_points.push_back(p);
    }
    this->updateReferenceLine(test_points);
  }

  visualizer_ = std::make_shared<Visualizer>(this);

  global_route_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/routing_path", 10,
    [this](nav_msgs::msg::Path::ConstSharedPtr msg) { this->globalRouteCallback(msg); });

  timer_ =
    this->create_wall_timer(std::chrono::milliseconds(100), [this]() { this->timerCallback(); });
}

void PncPlannerNode::timerCallback()
{
  // this->testSinPathVisual();
  this->testEgoVehicle();
  this->publishReferenceLine();

  VehicleInfo ego = ego_vehicle_->getVehicleState();
  Trajectory out_trajectory;
  if (lattice_planner_->plan(ego, *ref_line_, out_trajectory)) {
    this->publishTrajectory(out_trajectory);
  }
}

void PncPlannerNode::testSinPathVisual() const
{
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

void PncPlannerNode::testEgoVehicle() const
{
  if (ref_line_ && ref_line_->getTotalLength() > 0.0) {
    double curr_s = 0.0;
    double curr_l = 0.0;
    double dt = 0.1;

    ego_vehicle_->updateState(dt);
    auto egoState = ego_vehicle_->getVehicleState();
    if (ref_line_->getFrenetPoint(egoState.pose.x, egoState.pose.y, curr_s, curr_l)) {
      RCLCPP_INFO(this->get_logger(), "[frenet]: s = %.2f, l = %.2f", curr_s, curr_l);
    }
  }
}

void PncPlannerNode::updateReferenceLine(const std::vector<geometry_msgs::msg::Point> & points)
{
  std::vector<double> x, y;
  for (auto point : points) {
    x.push_back(point.x);
    y.push_back(point.y);
  }

  if (!ref_line_->init(x, y)) {
    RCLCPP_WARN(this->get_logger(), "参考线初始化失败!");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "初始化参考线成功，总长度为 %.2f", ref_line_->getTotalLength());
}

void PncPlannerNode::publishReferenceLine()
{
  if (ref_line_ == nullptr || ref_line_->getTotalLength() <= 0.0) {
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

void PncPlannerNode::globalRouteCallback(const nav_msgs::msg::Path::ConstSharedPtr & msg)
{
  if (msg->poses.size() < 2) return;

  std::vector<geometry_msgs::msg::Point> raw_points;

  for (const auto & pose_stamp : msg->poses) {
    geometry_msgs::msg::Point p;
    p.x = pose_stamp.pose.position.x;
    p.y = pose_stamp.pose.position.y;
    p.z = pose_stamp.pose.position.z;

    raw_points.push_back(p);
  }

  this->updateReferenceLine(raw_points);
}

void PncPlannerNode::publishTrajectory(const Trajectory & traj)
{
  nav_msgs::msg::Path path;

  path.header.frame_id = "map";
  path.header.stamp = this->now();

  for (const auto & i : traj) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();

    pose.pose.position.x = i.x;
    pose.pose.position.y = i.y;
    pose.pose.position.z = 0.0;

    tf2::Quaternion qtn;
    qtn.setRPY(0.0, 0.0, i.heading);
    pose.pose.orientation.x = qtn.getX();
    pose.pose.orientation.y = qtn.getY();
    pose.pose.orientation.z = qtn.getZ();
    pose.pose.orientation.w = qtn.getW();

    path.poses.push_back(pose);
  }
  visualizer_->publishTrajectory(path);
}

}  // namespace pnc_planner
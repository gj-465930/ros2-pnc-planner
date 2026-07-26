#include "pnc_planner/pnc_planner_node.hpp"

#include "pnc_planner/controller/pid_controller.hpp"
#include "pnc_planner/controller/pure_pursuit_controller.hpp"
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
  declare_parameter("planning_failure_fallback_decel", -3.0);
  declare_parameter("lattice_planner.weights.w_lat", 1.0);
  declare_parameter("lattice_planner.weights.w_lon", 10.0);
  declare_parameter("lattice_planner.weights.w_offset", 0.3);
  declare_parameter("lattice_planner.weights.w_speed", 1.0);

  // mock_ego
  declare_parameter("mock_ego.x", 0.0);
  declare_parameter("mock_ego.y", 0.0);
  declare_parameter("mock_ego.yaw", 0.0);
  declare_parameter("mock_ego.v", 5.0);
  declare_parameter("mock_ego.a", 0.0);

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

  planning_failure_fallback_decel_ = get_parameter("planning_failure_fallback_decel").as_double();
  if (!std::isfinite(planning_failure_fallback_decel_) || planning_failure_fallback_decel_ > 0.0) {
    RCLCPP_WARN(
      this->get_logger(), "Invalid planning_failure_fallback_decel %.2f, using default -3.0",
      planning_failure_fallback_decel_);
    planning_failure_fallback_decel_ = -3.0;
  }

  lattice_planner_ = std::make_shared<LatticePlanner>(config);

  // 初始化自车
  ego_vehicle_ = std::make_shared<EgoVehicle>(this);

  // 初始化参考线对象
  ref_line_ = std::make_shared<ReferenceLine>();

  this->declare_parameter("use_mock_routing", false);
  use_mock_routing_ = this->get_parameter("use_mock_routing").as_bool();

  if (use_mock_routing_) {
    RCLCPP_WARN(this->get_logger(), "Using internal mock routing and ego state");
    std::vector<geometry_msgs::msg::Point> test_points;
    for (int i = 0; i < 20; i++) {
      geometry_msgs::msg::Point p;
      p.x = i * 5.0;
      p.y = i * i * 1.0;  // 抛物线
      p.z = 0.0;
      test_points.push_back(p);
    }
    const bool mock_route_ready = this->updateReferenceLine(test_points);

    const double x = this->get_parameter("mock_ego.x").as_double();
    const double y = this->get_parameter("mock_ego.y").as_double();
    const double yaw = this->get_parameter("mock_ego.yaw").as_double();
    const double v = this->get_parameter("mock_ego.v").as_double();
    const double a = this->get_parameter("mock_ego.a").as_double();

    const bool mock_state_valid = std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw) &&
                                  std::isfinite(v) && std::isfinite(a);

    if (!mock_route_ready || !mock_state_valid) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to initialize mock routing mode: route_ready=%s, state_valid=%s",
        mock_route_ready ? "true" : "false", mock_state_valid ? "true" : "false");
    } else {
      ego_vehicle_->setPose(x, y, yaw);
      ego_vehicle_->setVelocity(v);
      ego_vehicle_->setCommand(a, 0.0);
      ego_vehicle_->updateState(0.0);

      route_ready_ = true;
      initial_state_ready_ = true;

      RCLCPP_INFO(
        this->get_logger(), "Initialized mock inputs: x=%.2f, y=%.2f, yaw=%.2f, v=%.2f, a=%.2f", x,
        y, yaw, v, a);
    }
  }

  // 初始化控制器
  lateral_ctrl_ = std::make_unique<controller::PurePursuitController>(3.0, 0.8, 2.8);
  longitudinal_controller_ = std::make_unique<controller::PidController>(
    2.0, 0.1, 0.05, 0.1, 3.0, config.max_acc, config.min_acc);

  visualizer_ = std::make_shared<Visualizer>(this);

  rclcpp::QoS scenario_qos(rclcpp::KeepLast(1));
  scenario_qos.reliable();
  scenario_qos.transient_local();

  global_route_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/routing_path", scenario_qos,
    [this](const nav_msgs::msg::Path::ConstSharedPtr & msg) { this->globalRouteCallback(msg); });

  initial_state_sub_ = this->create_subscription<pnc_planner::msg::ScenarioInitialState>(
    "/scenario/initial_state", scenario_qos,
    [this](const pnc_planner::msg::ScenarioInitialState::ConstSharedPtr & msg) {
      this->initialStateCallback(msg);
    });

  timer_ =
    this->create_wall_timer(std::chrono::milliseconds(100), [this]() { this->timerCallback(); });
}

void PncPlannerNode::timerCallback()
{
  if (!route_ready_ || !initial_state_ready_) {
    return;
  }

  double dt = 0.1;

  // 可视化参考线
  this->publishReferenceLine();

  if (ref_line_ == nullptr || ref_line_->getTotalLength() <= 0.0) return;
  VehicleInfo ego = ego_vehicle_->getVehicleState();

  // 规划
  Trajectory candidate_traj;
  const bool planning_success = lattice_planner_->plan(ego, *ref_line_, candidate_traj);

  if (planning_success && !candidate_traj.empty()) {
    planned_traj_ = candidate_traj;

    // 轨迹可视化
    publishTrajectory(planned_traj_);
    // 跟踪
    trackTrajectory(dt);
    return;
  }

  planned_traj_.clear();

  RCLCPP_WARN_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "Planning failed; cleared stale trajectory and applying fallback decel %.2f",
    planning_failure_fallback_decel_);

  ego_vehicle_->setCommand(planning_failure_fallback_decel_, 0.0);
  ego_vehicle_->updateState(dt);
}

bool PncPlannerNode::updateReferenceLine(const std::vector<geometry_msgs::msg::Point> & points)
{
  std::vector<double> x, y;
  for (auto point : points) {
    x.push_back(point.x);
    y.push_back(point.y);
  }

  if (!ref_line_->init(x, y)) {
    RCLCPP_WARN(this->get_logger(), "参考线初始化失败!");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "初始化参考线成功，总长度为 %.2f", ref_line_->getTotalLength());
  return true;
}

void PncPlannerNode::publishReferenceLine() const
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
  if (use_mock_routing_) {
    RCLCPP_WARN_ONCE(
      this->get_logger(), "Ignoring /routing_path because mock routing mode is enabled");
    return;
  }

  if (route_ready_) {
    RCLCPP_WARN_ONCE(
      this->get_logger(),
      "Ignoring repeated /routing_path. "
      "Restart planner to switch scenarios.");
    return;
  }

  if (msg->poses.size() < 3) {
    RCLCPP_ERROR(
      this->get_logger(), "Rejected /routing_path: expected at least 3 poses, got %zu",
      msg->poses.size());
    return;
  }

  std::vector<geometry_msgs::msg::Point> raw_points;
  raw_points.reserve(msg->poses.size());

  for (const auto & pose_stamp : msg->poses) {
    geometry_msgs::msg::Point p;
    p.x = pose_stamp.pose.position.x;
    p.y = pose_stamp.pose.position.y;
    p.z = pose_stamp.pose.position.z;

    raw_points.push_back(p);
  }

  if (!this->updateReferenceLine(raw_points)) {
    RCLCPP_ERROR(
      this->get_logger(), "Rejected /routing_path because reference line initialization failed");
    return;
  }

  route_ready_ = true;
  logScenarioReadyIfComplete();
}

void PncPlannerNode::publishTrajectory(const Trajectory & traj) const
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

void PncPlannerNode::trackTrajectory(const double dt)
{
  VehicleInfo ego = ego_vehicle_->getVehicleState();

  // 横向控制
  double omega = lateral_ctrl_->computeYawRate(planned_traj_, ego);

  // 纵向控制
  double a = longitudinal_controller_->computeAccel(planned_traj_, ego);

  ego_vehicle_->setCommand(a, omega);
  ego_vehicle_->updateState(dt);
}

void PncPlannerNode::initialStateCallback(
  const pnc_planner::msg::ScenarioInitialState::ConstSharedPtr & msg)
{
  if (use_mock_routing_) {
    RCLCPP_WARN_ONCE(
      this->get_logger(), "Ignoring /scenario/initial_state because mock routing mode is enabled");
    return;
  }

  if (initial_state_ready_) {
    RCLCPP_WARN_ONCE(
      this->get_logger(),
      "Ignoring repeated /scenario/initial_state. "
      "Restart planner to switch scenarios.");
    return;
  }

  if (msg->state != "CRUISING") {
    RCLCPP_ERROR(this->get_logger(), "Unsupported scenario state: %s", msg->state.c_str());
    return;
  }

  const bool state_values_valid = std::isfinite(msg->x) && std::isfinite(msg->y) &&
                                  std::isfinite(msg->yaw) && std::isfinite(msg->velocity) &&
                                  std::isfinite(msg->acceleration) && msg->velocity >= 0.0;

  if (!state_values_valid) {
    RCLCPP_ERROR(this->get_logger(), "Rejected scenario initial state with invalid numeric values");
    return;
  }

  ego_vehicle_->setPose(msg->x, msg->y, msg->yaw);
  ego_vehicle_->setVelocity(msg->velocity);
  ego_vehicle_->setCommand(msg->acceleration, 0.0);
  ego_vehicle_->updateState(0.0);

  initial_state_ready_ = true;

  RCLCPP_INFO(
    this->get_logger(), "Applied initial state: x=%.2f, y=%.2f, yaw=%.2f, v=%.2f, a=%.2f, state=%s",
    msg->x, msg->y, msg->yaw, msg->velocity, msg->acceleration, msg->state.c_str());

  logScenarioReadyIfComplete();
}

void PncPlannerNode::logScenarioReadyIfComplete()
{
  if (!route_ready_ || !initial_state_ready_ || scenario_ready_logged_) {
    return;
  }

  scenario_ready_logged_ = true;
  RCLCPP_INFO(
    this->get_logger(),
    "Scenario inputs are complete; starting planning. "
    "Restart planner to switch scenarios.");
}

}  // namespace pnc_planner

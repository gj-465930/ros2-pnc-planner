#pragma once

#include "pnc_planner/common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace pnc_planner {

class EgoVehicle {
public:
  explicit EgoVehicle(rclcpp::Node *node);

  // 状态更新接口
  void updateState(double dt);

  // 位姿设置接口
  void setPose(double x, double y, double yaw);

  // 速度设置接口
  void setCommand(double v, double omega);

  // 获取当前位置
  VehicleInfo getVehicleState();

private:
  rclcpp::Node *node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  VehicleInfo vehicle_info_;
};

} // namespace pnc_planner
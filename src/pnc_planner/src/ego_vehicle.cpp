#include "pnc_planner/ego_vehicle.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include <cmath>

namespace pnc_planner {

EgoVehicle::EgoVehicle(rclcpp::Node *node) : node_(node) {
  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  vehicle_info_.pose.x = 0.0;
  vehicle_info_.pose.y = 0.0;
  vehicle_info_.pose.yaw = 0.0;

  vehicle_info_.v = 0.0;
  vehicle_info_.a = 0.0;
  vehicle_info_.omega = 0.0;
  vehicle_info_.current_state = VehicleState::INIT;
}

void EgoVehicle::updateState(double dt) {
  double v0 = vehicle_info_.v;
  double yaw0 = vehicle_info_.pose.yaw;

  double v1 = v0 + vehicle_info_.a * dt;
  double delta_s = 0.0;

  if (v1 < 0.0 && v0 >= 0.0) {
    double t_stop = v0 / std::abs(vehicle_info_.a);
    delta_s = v0 * dt + 0.5 * vehicle_info_.a * t_stop * t_stop;

    vehicle_info_.v = 0.0;
    vehicle_info_.a = 0.0;
  } else {
    delta_s = v0 * dt + 0.5 * vehicle_info_.a * dt * dt;
    vehicle_info_.v = v1;
  }

  if (std::abs(vehicle_info_.v) < 0.01 && std::abs(vehicle_info_.a) < 0.01) {
    vehicle_info_.current_state = VehicleState::STANDBY;
  } else {
    vehicle_info_.current_state = VehicleState::CRUISING;
  }

  vehicle_info_.pose.x = delta_s * std::cos(yaw0);
  vehicle_info_.pose.y = delta_s * std::sin(yaw0);
  vehicle_info_.pose.yaw += vehicle_info_.omega * dt;

  tf2::Quaternion qtn;
  qtn.setRPY(0.0, 0.0, vehicle_info_.pose.yaw);

  geometry_msgs::msg::TransformStamped transform;

  transform.header.frame_id = "map";
  transform.header.stamp = node_->now();

  transform.child_frame_id = "base_link";

  transform.transform.translation.x = vehicle_info_.pose.x;
  transform.transform.translation.y = vehicle_info_.pose.y;
  transform.transform.translation.z = 0.0;

  transform.transform.rotation.x = qtn.getX();
  transform.transform.rotation.y = qtn.getY();
  transform.transform.rotation.z = qtn.getZ();
  transform.transform.rotation.w = qtn.getW();

  broadcaster_->sendTransform(transform);
}

void EgoVehicle::setPose(double x, double y, double yaw) {
  vehicle_info_.pose.x = x;
  vehicle_info_.pose.y = y;
  vehicle_info_.pose.yaw = yaw;
}

void EgoVehicle::setCommand(double v, double a, double omega) {
  vehicle_info_.v = v;
  vehicle_info_.a = a;
  vehicle_info_.omega = omega;
}

VehicleInfo EgoVehicle::getVehicleState() { return vehicle_info_; }

} // namespace pnc_planner
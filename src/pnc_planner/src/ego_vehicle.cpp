#include "pnc_planner/ego_vehicle.hpp"

#include "tf2/LinearMath/Quaternion.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include <cmath>

namespace pnc_planner
{

EgoVehicle::EgoVehicle(rclcpp::Node * node) : node_(node)
{
  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  vehicle_info_.pose.x = 0.0;
  vehicle_info_.pose.y = 0.0;
  vehicle_info_.pose.yaw = 0.0;

  vehicle_info_.v = 0.0;
  vehicle_info_.a = 0.0;
  vehicle_info_.omega = 0.0;
  vehicle_info_.current_state = VehicleState::INIT;
}

void EgoVehicle::updateState(double dt)
{
  double v0 = vehicle_info_.v;
  double yaw0 = vehicle_info_.pose.yaw;

  const double a0 = vehicle_info_.a;
  const double a1 = commanded_accel_;
  double delta_s = 0.0;

  if (dt == 0.0) {
    // 初始化场景状态
    vehicle_info_.a = a1;
  } else if (v0 >= 0.0 && v0 < 0.01 && a1 <= 0.0) {
    // 仿真的静止死区：低速且仍在制动时，不再累计位移
    vehicle_info_.v = 0.0;
    vehicle_info_.a = 0.0;
  } else {
    const double jerk = (a1 - a0) / dt;
    const double v1 = v0 + (a0 + a1) / 2.0 * dt;

    if (v1 < 0.0 && v0 >= 0.0) {
      // 找到本轮内速度刚好是0的时刻
      double low = 0.0;
      double high = dt;
      for (std::size_t i = 0; i < 40; ++i) {
        const double mid = (low + high) / 2.0;
        const double mid_a = a0 + jerk * mid;
        const double mid_v = v0 + (a0 + mid_a) / 2.0 * mid;
        if (mid_v > 0.0) {
          low = mid;
        } else {
          high = mid;
        }
      }

      const double t_stop = high;
      delta_s = v0 * t_stop + 0.5 * a0 * t_stop * t_stop + jerk * t_stop * t_stop * t_stop / 6.0;
      vehicle_info_.v = 0.0;
      vehicle_info_.a = 0.0;
    } else {
      delta_s = v0 * dt + 0.5 * a0 * dt * dt + jerk * dt * dt * dt / 6.0;
      vehicle_info_.v = v1;
      vehicle_info_.a = a1;
    }
  }

  if (std::abs(vehicle_info_.v) < 0.01 && std::abs(vehicle_info_.a) < 0.01) {
    vehicle_info_.current_state = VehicleState::STANDBY;
  } else {
    vehicle_info_.current_state = VehicleState::CRUISING;
  }

  vehicle_info_.pose.x += delta_s * std::cos(yaw0);
  vehicle_info_.pose.y += delta_s * std::sin(yaw0);
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

void EgoVehicle::setPose(double x, double y, double yaw)
{
  vehicle_info_.pose.x = x;
  vehicle_info_.pose.y = y;
  vehicle_info_.pose.yaw = yaw;
}

void EgoVehicle::setVelocity(double v)
{
  vehicle_info_.v = v;
}

void EgoVehicle::setCommand(double a, double omega)
{
  commanded_accel_ = a;
  vehicle_info_.omega = omega;
}

VehicleInfo EgoVehicle::getVehicleState()
{
  return vehicle_info_;
}

}  // namespace pnc_planner
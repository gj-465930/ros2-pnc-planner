#include "pnc_planner/reference_line.hpp"
#include "pnc_planner/math/cartesian_frenet.hpp"

namespace pnc_planner {

bool Spline2D::init(const std::vector<double> &x,
                    const std::vector<double> &y) {
  if (x.size() != y.size() || x.size() < 3) {
    return false;
  }

  s_.clear();
  s_.push_back(0.0);

  for (size_t i = 1; i < x.size(); ++i) {
    double dx = x[i] - x[i - 1];
    double dy = y[i] - y[i - 1];

    double ds = std::hypot(dx, dy);
    s_.push_back(s_.back() + ds);
  }

  x_spline_.init(s_, x);
  y_spline_.init(s_, y);

  return true;
}

std::vector<double> Spline2D::calcPosition(double s) const {
  return {x_spline_.calc(s), y_spline_.calc(s)};
}

double Spline2D::calcHeading(double s) const {
  double dx = x_spline_.calcDerivative(s);
  double dy = y_spline_.calcDerivative(s);

  return std::atan2(dy, dx);
}

double Spline2D::calcKappa(double s) const {
  double dx = x_spline_.calcDerivative(s);
  double dy = y_spline_.calcDerivative(s);

  double ddx = x_spline_.calcSecondDerivative(s);
  double ddy = y_spline_.calcSecondDerivative(s);

  double tmp = dx * dx + dy * dy;
  double denominator = std::pow(tmp, 1.5);

  return std::abs(dx * ddy - ddx * dy) / denominator;
}

double Spline2D::getTotalLength() const { return s_.empty() ? 0.0 : s_.back(); }

bool ReferenceLine::init(const std::vector<double> &x,
                         const std::vector<double> &y) {
  if (!spline_.init(x, y)) {
    return false;
  }

  length_ = spline_.getTotalLength();
  return true;
}

WayPoint ReferenceLine::getWayPoint(double s) const {
  if (s < 0.0) {
    s = 0.0;
  }
  if (s > length_) {
    s = length_;
  }

  WayPoint wp;
  wp.s = s;
  auto pos = spline_.calcPosition(s);
  wp.x = pos[0];
  wp.y = pos[1];

  wp.heading = spline_.calcHeading(s);

  wp.kappa = spline_.calcKappa(s);
  return wp;
}
/**
 * @brief
 *
 * @param x[in]
 * @param y[in]
 * @param s[out]
 * @param l[out]
 * @return true 转换成功
 * @return false 转换失败
 */
bool ReferenceLine::getFrenetPoint(double x, double y, double &s,
                                   double &l) const {
  auto eval_func = [this](double &s, double &x, double &y, double &heading) {
    WayPoint wp = this->getWayPoint(s);
    x = wp.x;
    y = wp.y;
    heading = wp.heading;
  };

  return math::CartesianFrenetConverter::cartesianToFrenet(

      x, y, length_, eval_func, s, l);
}

bool ReferenceLine::getCartesianPoint(double s, double l, double &x, double &y,
                                      double &yaw) const {
  auto eval_func = [this](double &s, double &x, double &y, double &heading) {
    WayPoint wp = this->getWayPoint(s);
    x = wp.x;
    y = wp.y;
    heading = wp.heading;
  };

  bool flag = math::CartesianFrenetConverter::frenetToCartesian(
      s, l, length_, eval_func, x, y);

  if (flag) {
    yaw = spline_.calcHeading(s);
  }

  return flag;
}

} // namespace pnc_planner
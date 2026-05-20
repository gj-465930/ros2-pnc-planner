#include "pnc_planner/math/cartesian_frenet.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace pnc_planner {
namespace math {

bool CartesianFrenetConverter::cartesianToFrenet(
    const double target_x, const double target_y, const double max_s,
    const EvaluateCurveFunc &eval_func, double &s, double &l) {
  if (max_s < 0.0 || !eval_func)
    return false;

  // 找到垂足并且计算最目标点与垂足的距离

  double best_s = 0.0; // 当前找到的最近点的s值
  double min_dist_sq = std::numeric_limits<double>::max();
  // 步长
  std::vector<double> steps = {1.0, 0.1, 0.01};

  double serach_start = 0.0;
  double serach_end = max_s;

  double temp_x = 0.0, temp_y = 0.0, temp_heading = 0.0;

  for (double step : steps) {
    double curr_best_s = best_s;
    for (double curr_s = serach_start; curr_s <= serach_end; curr_s += step) {
      eval_func(curr_s, temp_x, temp_y, temp_heading);

      double dx = temp_x - target_x;
      double dy = temp_y - target_y;
      double dist_sq = dx * dx + dy * dy;

      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        curr_best_s = curr_s;
      }
    }

    best_s = curr_best_s;
    serach_start = std::max(0.0, best_s - 2 * step);
    serach_end = std::min(max_s, best_s + 2 * step);
  }

  s = best_s;

  // 判断l的正负（根据叉乘出来z的方向向量判断）

  eval_func(s, temp_x, temp_y, temp_heading);

  // 切向量
  double tau_x = std::cos(temp_heading);
  double tau_y = std::sin(temp_heading);

  double dx = target_x - temp_x;
  double dy = target_y - temp_y;

  double distance = std::sqrt(min_dist_sq);

  double cross_product = tau_x * dy - tau_y * dx;

  l = (cross_product > 0) ? distance : -distance;
  return true;
}

bool CartesianFrenetConverter::frenetToCartesian(
    const double s, const double l, const double max_s,
    const EvaluateCurveFunc &eval_func, double &x, double &y) {
  if (s < 0 || s > max_s || !eval_func) {
    return false;
  }

  double ref_x = 0.0, ref_y = 0.0, ref_heading = 0.0;
  eval_func(s, ref_x, ref_y, ref_heading);

  // 切向量
  double tau_x = std::cos(ref_heading);
  double tau_y = std::sin(ref_heading);

  // 法向量
  double nor_x = -tau_y;
  double nor_y = tau_x;

  x = ref_x + l * nor_x;
  y = ref_y + l * nor_y;

  return true;
}

} // namespace math
} // namespace pnc_planner
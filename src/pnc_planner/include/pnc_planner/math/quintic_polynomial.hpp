/**
 * @file quintic_polynomial.hpp
 * @author jguo1030
 * @brief 五次多项式求解器，用于生成基于起终点状态约束的平滑过渡曲线
 * @version 0.1
 * @date 2026-05-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

namespace pnc_planner {
namespace math {

class QuinticPolynomial {
public:
  QuinticPolynomial() = default;

  /**
   * @brief 核心构造函数，瞬间解开 6 个多项式系数
   * @param[in] x0 起点状态：位置 (0阶导)
   * @param[in] v0 起点状态：速度 (1阶导)
   * @param[in] a0 起点状态：加速度 (2阶导)
   * @param[in] x1 终点状态：位置 (0阶导)
   * @param[in] v1 终点状态：速度 (1阶导)
   * @param[in] a1 终点状态：加速度 (2阶导)
   * @param[in] T  规划自变量跨度
   */
  QuinticPolynomial(double x0, double v0, double a0, double x1, double v1,
                    double a1, double T);

  // 计算任意里程 s 处的横向偏差 L
  double evaluate(double s) const;
  // 计算速度
  double evaluate_d(double s) const;
  // 计算加速度
  double evaluate_dd(double s) const;
  // 计算加加速度
  double evaluate_ddd(double s) const;
  // 获取T
  double get_T() const { return T_; }

private:
  double c0_ = 0.0;
  double c1_ = 0.0;
  double c2_ = 0.0;
  double c3_ = 0.0;
  double c4_ = 0.0;
  double c5_ = 0.0;

  double T_;
};

} // namespace math
} // namespace pnc_planner
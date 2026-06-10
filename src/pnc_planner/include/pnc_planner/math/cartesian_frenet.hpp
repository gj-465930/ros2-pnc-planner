#pragma once

#include <functional>

namespace pnc_planner ::math
{

/**
 * @brief 实现笛卡尔坐标系与frenet坐标系的转换
 */
class CartesianFrenetConverter
{
public:
  CartesianFrenetConverter() = delete;

  using EvaluateCurveFunc =
    std::function<void(double & s, double & x, double & y, double & heading)>;

  /**
   * @brief 笛卡尔坐标系转换frenet坐标系
   * @param target_x 目标点笛卡尔坐标 X
   * @param target_y 目标点笛卡尔坐标 Y
   * @param max_s 曲线最大长度
   * @param eval_func 曲线求值回调函数
   * @param s [out] 算出的纵向里程
   * @param l [out] 算出的横向偏差
   * @return true 转换成功; false 转换失败
   */
  static bool cartesianToFrenet(
    const double target_x, const double target_y, const double max_s,
    const EvaluateCurveFunc & eval_func, double & s, double & l);

  /**
   * @brief Frenet 坐标转笛卡尔坐标 (SL -> XY)
   * @param s 目标点的纵向里程
   * @param l 目标点的横向偏差
   * @param max_s 曲线的最大长度
   * @param eval_func 曲线求值回调函数 (闭包)
   * @param x [out] 算出的世界坐标 X
   * @param y [out] 算出的世界坐标 Y
   * @return true 转换成功; false 越界等异常
   */
  static bool frenetToCartesian(
    const double s, const double l, const double max_s, const EvaluateCurveFunc & eval_func,
    double & x, double & y);
};

}  // namespace pnc_planner::math
#ifndef SRM_IC_2023_MODULES_BALLISTIC_SOLVER_BALLISTIC_SOLVER_H_
#define SRM_IC_2023_MODULES_BALLISTIC_SOLVER_BALLISTIC_SOLVER_H_

#include <memory>
#include <Eigen/Core>
#include "common/syntactic-sugar.h"
#include "common/rk4-solver.h"

namespace ballistic_solver {
using SVec = Eigen::Vector3d;  ///< 球坐标，以 (phi, theta, r) 表示，正方向依次为：右偏、上仰
using CVec = Eigen::Vector3d;  ///< 直角坐标，以 (x, y, z) 表示，正方向依次为：右移、下移、前移

/// 模型计算接口类
class Model {
 public:
  /**
   * @brief 计算阻力加速度
   * @param t 当前时间，单位：s
   * @param [in] v 当前速度，单位：m/s, m/s, m/s
   * @return 当前阻力加速度，单位：m/s^2, m/s^2, m/s^2
   */
  virtual CVec operator()(double t, CVec REF_IN v) const = 0;
};

/// 空气阻力模型，假设：对流层；低马赫数；弹丸无自转、无攻角
class AirResistanceModel final : public Model {
 public:
  /**
   * @brief 设定空气状态参数
   * @param c 空气阻力系数，只与弹丸形状有关：理想球体为 0.48，高尔夫球为 0.26
   * @param p 气压，单位：hPa
   * @param t 气温，单位：摄氏度
   * @param d 弹丸直径，单位：m
   * @param m 弹丸质量，单位：kg
   */
  void SetParam(double c, double p, double t, double d, double m);

  CVec operator()(double t, CVec REF_IN v) const final;

 private:
  double c_{};  ///< 最终空气阻力系数 C，满足 f = C * v^2
};

/// 重力模型，假设：弹丸近似为球体；海拔高度近似为 0
class GravityModel final : public Model {
 public:
  /**
   * @brief 设置重力模型参数
   * @param p 当地纬度，单位：度
   */
  void SetParam(double p);

  CVec operator()(double t, CVec REF_IN v) const final;

 private:
  double g_{};  ///< 当前重力加速度，单位：m/s^2
};

/// 弹道微分方程
class BallisticEquation {
 public:
  /**
   * @brief 增加一个受力模型
   * @param [in] model 模型指针
   */
  void AddModel(std::shared_ptr<Model> REF_IN model);

  /**
   * @brief 计算合力加速度
   * @param t 当前时间，单位：s
   * @param v 当前速度，单位：m/s, m/s, m/s
   * @return 当前合力加速度，单位：m/s^2, m/s^2, m/s^2
   */
  CVec operator()(double t, CVec v, CVec acc = {0, 0, 0}) const;

 private:
  std::vector<std::shared_ptr<Model>> models_;  ///< 受力模型列表
};

/// 子弹命中数据
struct BallisticInfo {
  double t;  ///< 子弹飞行时间，单位：s
  SVec v_0;  ///< 子弹初速，单位：rad, rad, m/s
  CVec v;    ///< 碰到目标时的子弹速度，单位：m/s, m/s, m/s
  CVec x;    ///< 碰到目标时的子弹位置，单位：m, m, m
};

/// 弹道求解器
class BallisticSolver {
 public:
  /**
   * @brief 增加一个受力模型
   * @param [in] model 模型指针
   */
  void AddModel(std::shared_ptr<Model> REF_IN model);

  /**
   * @brief 初始化
   * @param [in] intrinsic_x 固有位置，单位：m, m, m
   * @param precision 计算精度，单位：s
   */
  void Initialize(CVec REF_IN intrinsic_x, double precision);

  /**
   * @brief 给定目标，求解落点接近目标的弹道
   * @param [in] target_x 目标位置，单位：m, m, m
   * @param initial_v 子弹相对自身的初速度，单位：m/s
   * @param [in] intrinsic_v 自身相对于地面的固有速度，单位：m/s, m/s, m/s
   * @param [out] solution_out 输出近似最优解数据
   * @param [out] error_out 输出近似最优解对应的水平面误差，单位：m
   * @return 是否存在解
   */
  bool Solve(CVec REF_IN target_x, double initial_v, CVec REF_IN intrinsic_v,
             BallisticInfo REF_OUT solution_out, double REF_OUT error_out);

 private:
  /**
   * @brief 更新初始状态参数
   * @param [in] start_v 初速度，单位：m/s, m/s, m/s
   */
  void SetParam(CVec REF_IN start_v);

  /**
   * @brief 给定条件，求解满足条件的弹丸终点
   * @param solution_cond 弹丸终点满足的条件
   * @param iter_cond 继续迭代的条件
   * @param [out] solutions 弹道数据列表
   */
  void Solve(std::function<bool(double t, CVec REF_IN v, CVec REF_IN x)> REF_IN solution_cond,
             std::function<bool(double t, CVec REF_IN v, CVec REF_IN x)> REF_IN iter_cond,
             std::vector<BallisticInfo> REF_OUT solutions);

  CVec intrinsic_x_{};  ///< 发射器在世界坐标系中的固有位置，单位：m, m, m
  CVec initial_v_{};    ///< 子弹相对发射器的初始速度，单位：m/s, m/s, m/s
  CVec intrinsic_v_{};  ///< 发射器相对地面的固有速度（实际存在且参与计算，但不计入结果），单位：m/s, m/s, m/s
  RK4Solver<double, CVec, BallisticEquation> solver_;  ///< 求解器
};
}

#endif  // SRM_IC_2023_MODULES_BALLISTIC_SOLVER_BALLISTIC_SOLVER_H_

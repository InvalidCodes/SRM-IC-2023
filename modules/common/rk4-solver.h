#ifndef SRM_IC_2023_MODULES_COMMON_RK4_SOLVER_H_
#define SRM_IC_2023_MODULES_COMMON_RK4_SOLVER_H_

/**
 * @brief 求解微分方程 y'(t) = F(t, y(t))
 * @details 使用龙格-库塔法 (RK4)，参考：https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
 * @note T、Y 和整数之间（以及 T 和 Y 与相同类型）必须能够进行四则运算，并且支持拷贝赋值函数
 * @tparam T 待求目标函数的自变量类型
 * @tparam Y 待求目标函数的因变量类型
 * @tparam F 等式右边的泛函 F 的类型，包含一个仿函数方法 Y operator() (T t, Y y)
 */
template<class T, class Y, class F>
struct RK4Solver {
  T t;  ///< 当前自变量
  T h;  ///< 自变量步长
  Y y;  ///< 当前因变量
  F f;  ///< 泛函 F，显式依赖 t 和 y(t)

  /**
   * @brief 迭代一次
   * @details 迭代后，当前自变量 t 增加一倍步长 h，并更新相应的 y
   */
  void forward() {
    Y k1 = f(t, y), k2 = f(t + h / 2, y + k1 * h / 2), k3 = f(t + h / 2, y + k2 * h / 2), k4 = f(t + h, y + k3 * h);
    y = y + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6;
    t = t + h;
  }
};

#endif  // SRM_IC_2023_MODULES_COMMON_RK4_SOLVER_H_

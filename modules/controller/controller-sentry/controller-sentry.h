#ifndef SRM_IC_2023_MODULES_CONTROLLER_HERO_CONTROLLER_SENTRY_H_
#define SRM_IC_2023_MODULES_CONTROLLER_HERO_CONTROLLER_SENTRY_H_

#include "controller-base/controller-base.h"

namespace controller::hero {
/**
 * @brief 英雄机器人主控接口类
 * @warning 禁止直接构造此类，请使用 @code controller::CreateController("sentry") @endcode 获取该类的公共接口指针
 */
class HeroController final : public Controller {
 public:
  bool Initialize() final;
  int Run() final;

 private:
  static Registry<HeroController> registry_;  ///< 主控注册信息
};
}

#endif  // SRM_IC_2023_MODULES_CONTROLLER_HERO_CONTROLLER_SENTRY_H_

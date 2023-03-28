#ifndef SRM_IC_2023_MODULES_CONTROLLER_BASE_CONTROLLER_BASE_H_
#define SRM_IC_2023_MODULES_CONTROLLER_BASE_CONTROLLER_BASE_H_

#include "serial/serial.h"
#include "video-source-base/video-source-base.h"
#include "video-writer/video-writer.h"
#include "coordinate/coordinate.h"
#include "detector-armor/detector_armor.h"

/// 抓取并处理来自操作系统的控制信号
void SignalHandler(int);

enable_factory(controller, Controller)

namespace controller {
/// 机器人主控公共接口类
class Controller {
  friend void::SignalHandler(int);
 public:
  Controller() = default;
  virtual ~Controller() = default;

  /**
   * @brief 初始化机器人
   * @return 是否初始化成功
   */
  virtual bool Initialize() = 0;

  /**
   * @brief 执行主控制循环
   * @return 错误码，可作为 main() 的返回值
   */
  virtual int Run() = 0;

 protected:
  bool Initialize(std::string REF_IN type_name);

  static std::atomic_bool exit_signal_;  ///< 主循环退出信号

  std::unique_ptr<video_source::VideoSource> video_source_;  ///< 视频源
  std::unique_ptr<serial::Serial> serial_;                   ///< 串口
  coordinate::CoordSolver coord_solver_;                     ///< 坐标求解器
  Frame frame_;                                              ///< 帧数据
  std::vector<bbox_t> boxes_;
  std::vector<Armor> armors_;                         
  ReceivePacket receive_packet_;
  video_writer::VideoWriter video_writer_;                   ///< 视频写入接口
  ArmorDetector armor_detector_;

 private:
  static std::function<void(void *, Frame &)> FrameCallback;  ///< 取图回调函数
};
}

#endif  // SRM_IC_2023_MODULES_CONTROLLER_BASE_CONTROLLER_BASE_H_

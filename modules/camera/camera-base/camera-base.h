#ifndef SRM_IC_2023_MODULES_CAMERA_BASE_CAMERA_BASE_H_
#define SRM_IC_2023_MODULES_CAMERA_BASE_CAMERA_BASE_H_

#include <atomic>
#include "common/factory.h"
#include "common/frame.h"
#include "common/buffer.h"

enable_factory(camera, Camera)

namespace camera {
/// 相机公共接口类
class Camera {
  static constexpr size_t BUFFER_SIZE = 2;  ///< 缓冲区大小
 public:
  Camera() = default;
  virtual ~Camera() = default;

  /**
   * @brief 打开指定序列号的相机
   * @param [in] serial_number 序列号
   * @param [in] config_file 相机配置文件路径
   * @return 是否成功打开相机
   */
  virtual bool OpenCamera(std::string REF_IN serial_number, std::string REF_IN config_file) = 0;

  /**
   * @brief 关闭相机
   * @return 相机是否成功关闭
   */
  virtual bool CloseCamera() = 0;

  /**
   * @brief 从缓冲区中取出一帧
   * @param [out] frame 帧结构体
   * @return 是否成功取出（缓冲区中是否有数据）
   */
  virtual bool GetFrame(Frame REF_OUT frame) = 0;

  /**
   * @brief 开启视频流
   * @return 是否成功开启视频流
   */
  virtual bool StartStream() = 0;

  /**
   * @brief 关闭视频流
   * @return 是否成功关闭视频流
   */
  virtual bool StopStream() = 0;

  /**
   * @brief 导入相机配置文件
   * @param [in] config_file 配置文件路径
   * @return 是否成功导入配置
   */
  virtual bool ImportConfigurationFile(std::string REF_IN config_file) = 0;

  /**
   * @brief 导出相机配置文件
   * @param [in] config_file 配置文件路径
   * @return 是否成功导出配置
   */
  virtual bool ExportConfigurationFile(std::string REF_IN config_file) = 0;

  /**
   * @brief 检查相机连接状态
   * @return 相机是否连接
   */
  virtual bool IsConnected() = 0;

  /**
   * @brief 设置曝光时间
   * @param exposure_time 曝光时间，单位 us
   * @return 是否设置成功
   */
  virtual bool SetExposureTime(uint32_t exposure_time) = 0;

  /**
   * @brief 设置增益
   * @param exposure_time 增益，单位 db
   * @return 是否设置成功
   */
  virtual bool SetGainValue(float gain) = 0;

  /**
   * @brief 注册相机回调，取图完毕时可立即修改 frame 的内容
   * @param callback 回调函数指针
   * @param obj 回调函数持有者指针
   */
  void RegisterFrameCallback(FrameCallback callback, void *obj);

  /**
   * @brief 反注册相机回调
   * @param callback 回调函数指针
   */
  void UnregisterFrameCallback(FrameCallback callback);

 protected:
  /// 注册的回调函数列表
  std::vector<std::pair<FrameCallback, void *>> callback_list_;
  std::string serial_number_;          ///< 相机序列号
  bool stream_running_{};              ///< 视频流运行标记
  std::atomic_bool stop_flag_{};       ///< 停止守护线程信号
  pthread_t daemon_thread_id_{};       ///< 守护线程句柄
  Buffer<Frame, BUFFER_SIZE> buffer_;  ///< 帧数据缓冲区
};
}

#endif  // SRM_IC_2023_MODULES_CAMERA_BASE_CAMERA_BASE_H_

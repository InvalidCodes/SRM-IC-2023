#ifndef SRM_IC_2023_MODULES_CAMERA_DH_CAMERA_DH_H_
#define SRM_IC_2023_MODULES_CAMERA_DH_CAMERA_DH_H_

#include "camera-base/camera-base.h"

namespace camera::dh {
/**
 * @brief 大恒相机接口类
 * @warning 禁止直接构造此类，请使用 @code camera::CreateCamera("DHCamera") @endcode 获取该类的公共接口指针
 */
class DHCamera final : public Camera {
 public:
  DHCamera() = default;
  ~DHCamera() final;

  bool OpenCamera(std::string REF_IN serial_number, std::string REF_IN config_file) final;
  bool CloseCamera() final;
  bool StartStream() final;
  bool StopStream() final;
  bool IsConnected() final;
  bool GetFrame(Frame REF_OUT frame) final;
  bool ExportConfigurationFile(std::string REF_IN config_file) final;
  bool ImportConfigurationFile(std::string REF_IN config_file) final;
  bool SetExposureTime(uint32_t exposure_time) final;
  bool SetGainValue(float gain_value) final;

 private:
  static void GX_STDC DefaultCaptureCallback(GX_FRAME_CALLBACK_PARAM *frame_callback);
  static void *DaemonThreadFunction(void *obj);

  bool RegisterCaptureCallback(GXCaptureCallBack callback);
  bool UnregisterCaptureCallback();
  bool SetExposureTimeDHImplementation(int64_t exposure_time);
  bool SetExposureMode(GX_EXPOSURE_MODE_ENTRY gx_exposure_mode_entry);
  bool SetExposureTimeMode(GX_EXPOSURE_TIME_MODE_ENTRY gx_exposure_time_mode_entry);
  bool SetGainValueDHImplementation(double gain);
  bool SetGainAuto(GX_GAIN_AUTO_ENTRY gx_gain_auto_entry);
  static std::string GetErrorInfo(GX_STATUS error_status_code);
  bool Raw8Raw16ToRGB24(GX_FRAME_CALLBACK_PARAM *frame_callback);

  static Registry<DHCamera> registry_;  ///< 相机注册信息
  static uint16_t camera_count_;        ///< 全局相机计数

  GX_DEV_HANDLE device_{};                  ///< 设备句柄
  int64_t color_filter_{};                  ///< 像素颜色格式
  int64_t payload_size_{};                  ///< 数据包大小
  unsigned char *raw_8_to_rgb_24_cache_{};  ///< 颜色转换缓存
  unsigned char *raw_16_to_8_cache_{};      ///< 颜色转换缓存
};
}

#endif  // SRM_IC_2023_MODULES_CAMERA_DH_CAMERA_DH_H_

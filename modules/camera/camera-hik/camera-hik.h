#ifndef SRM_IC_2023_MODULES_CAMERA_HIK_CAMERA_HIK_H_
#define SRM_IC_2023_MODULES_CAMERA_HIK_CAMERA_HIK_H_

#include "camera-base/camera-base.h"

namespace camera::hik {
/**
 * @brief 海康相机接口类
 * @warning 禁止直接构造此类，请使用 @code camera::CreateCamera("HikCamera") @endcode 获取该类的公共接口指针
 */
class HikCamera final : public Camera {
 public:
  HikCamera() = default;
  ~HikCamera() final;

  bool OpenCamera(std::string REF_IN serial_number, std::string REF_IN config_file) final;
  bool CloseCamera() final;
  bool StartStream() final;
  bool StopStream() final;
  bool GetFrame(Frame REF_OUT frame) final;
  bool IsConnected() final;
  bool ExportConfigurationFile(std::string REF_IN config_file) final;
  bool ImportConfigurationFile(std::string REF_IN config_file) final;
  bool SetExposureTime(uint32_t exposure_time) final;
  bool SetGainValue(float gain) final;

 private:
  static void __stdcall ImageCallbackEx(unsigned char *image_data, MV_FRAME_OUT_INFO_EX *frame_info, void *obj);
  static void *DaemonThreadFunction(void *obj);

  bool SetExposureTimeHikImplementation(float exposure_time);

  static Registry<HikCamera> registry_;  ///< 相机注册信息

  void *device_{};  ///< 设备句柄
};
}

#endif  // SRM_IC_2023_MODULES_CAMERA_HIK_CAMERA_HIK_H_

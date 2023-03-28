#ifndef SRM_IC_2023_MODULES_VIDEO_SOURCE_CAMERA_VIDEO_SOURCE_CAMERA_H_
#define SRM_IC_2023_MODULES_VIDEO_SOURCE_CAMERA_VIDEO_SOURCE_CAMERA_H_

#include "video-source-base/video-source-base.h"

namespace video_source::camera {
/**
 * @brief 相机视频源接口类
 * @warning 禁止直接构造此类，请使用 @code video_source::CreateVideoSource("camera") @endcode 获取该类的公共接口指针
 */
class CameraVideoSource final : public VideoSource {
 public:
  CameraVideoSource() = default;
  ~CameraVideoSource() final = default;

  bool Initialize(std::string REF_IN config_file) final;
  bool GetFrame(Frame REF_OUT frame) final;
  void RegisterFrameCallback(FrameCallback callback, void *obj) final;
  void UnregisterFrameCallback(FrameCallback callback) final;

 private:
  static Registry<CameraVideoSource> registry_;  ///< 视频源注册信息

  std::unique_ptr<::camera::Camera> camera_{};  ///< 相机公共接口指针
};
}

#endif  // SRM_IC_2023_MODULES_VIDEO_SOURCE_CAMERA_VIDEO_SOURCE_CAMERA_H_

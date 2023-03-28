#ifndef SRM_IC_2023_MODULES_VIDEO_SOURCE_BASE_VIDEO_SOURCE_BASE_H_
#define SRM_IC_2023_MODULES_VIDEO_SOURCE_BASE_VIDEO_SOURCE_BASE_H_

#include "common/factory.h"
#include "common/frame.h"

enable_factory(video_source, VideoSource)

namespace video_source {
/// 视频源公共接口类
class VideoSource {
 public:
  VideoSource() = default;
  virtual ~VideoSource() = default;

  /// 相机内参矩阵
  attr_reader_ref(intrinsic_mat_, IntrinsicMat)

  /// 相机外参矩阵
  attr_reader_ref(distortion_mat_, DistortionMat)

  /**
   * @brief 初始化视频源
   * @param [in] config_file 配置文件路径
   * @return 是否初始化成功
   */
  virtual bool Initialize(std::string REF_IN config_file) = 0;

  /**
   * @brief 获取帧数据
   * @param [out] frame 帧数据输出
   * @return 是否获取成功
   */
  virtual bool GetFrame(Frame REF_OUT frame) = 0;

  /**
   * @brief 注册帧数据回调函数
   * @param callback 回调函数
   * @param obj 回调函数持有者指针
   */
  virtual void RegisterFrameCallback(FrameCallback callback, void *obj) = 0;

  /**
   * @brief 反注册帧数据回调函数
   * @param callback 回调函数
   */
  virtual void UnregisterFrameCallback(FrameCallback callback) = 0;

 protected:
  cv::Mat intrinsic_mat_;   ///< 相机内参矩阵
  cv::Mat distortion_mat_;  ///< 相机外参矩阵
};
}

#endif  // SRM_IC_2023_MODULES_VIDEO_SOURCE_BASE_VIDEO_SOURCE_BASE_H_

#ifndef SRM_IC_2023_MODULES_VIDEO_SOURCE_FILE_VIDEO_SOURCE_FILE_H_
#define SRM_IC_2023_MODULES_VIDEO_SOURCE_FILE_VIDEO_SOURCE_FILE_H_

#include "video-source-base/video-source-base.h"

namespace video_source::file {
/**
 * @brief 文件读取视频源接口类
 * @warning 禁止直接构造此类，请使用 @code video_source::CreateVideoSource("file") @endcode 获取该类的公共接口指针
 */
class FileVideoSource final : public VideoSource {
 public:
  FileVideoSource() = default;
  ~FileVideoSource() final = default;

  bool Initialize(std::string REF_IN config_file) final;
  bool GetFrame(Frame REF_OUT frame) final;
  void RegisterFrameCallback(FrameCallback callback, void *obj) final;
  void UnregisterFrameCallback(FrameCallback callback) final;

 private:
  static Registry<FileVideoSource> registry_;  ///< 相机公共接口指针

  /// 注册回调函数列表
  std::vector<std::pair<FrameCallback, void *>> callback_list_;
  double frame_rate_{};     ///< 帧率
  cv::VideoCapture video_;  ///< 视频读取接口
  uint64_t time_stamp_{};   ///< 时间戳
};
}

#endif  // SRM_IC_2023_MODULES_VIDEO_SOURCE_FILE_VIDEO_SOURCE_FILE_H_

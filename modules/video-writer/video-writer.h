#ifndef SRM_IC_2023_MODULES_VIDEO_WRITER_VIDEO_WRITER_H_
#define SRM_IC_2023_MODULES_VIDEO_WRITER_VIDEO_WRITER_H_

#include <thread>
#include <atomic>
#include <opencv2/videoio.hpp>
#include "common/buffer.h"

namespace video_writer {
/// 视频录像接口
class VideoWriter final {
  static constexpr size_t BUFFER_SIZE = 512;  ///< 缓冲区大小
 public:
  VideoWriter() = default;
  ~VideoWriter();

  /**
   * @brief 打开或创建新视频文件
   * @param [in] video_file 文件名
   * @param frame_size 图像长宽大小
   * @param fps 录像帧率
   * @return
   */
  bool Open(std::string REF_IN video_file, cv::Size frame_size, double fps = 60);

  /**
   * @brief 写入视频
   * @param [in] frame 图像数据
   */
  void Write(cv::Mat FWD_IN frame);

 private:
  static void WritingThreadFunction(void *obj);

  std::unique_ptr<cv::VideoWriter> writer_;  ///< 视频写入接口
  Buffer<cv::Mat, BUFFER_SIZE> buffer_;      ///< 缓冲区
  std::thread thread_;                       ///< 写入线程
  std::atomic_bool stop_flag_{};             ///< 写入线程停止信号
};
}

#endif  // SRM_IC_2023_MODULES_VIDEO_WRITER_VIDEO_WRITER_H_

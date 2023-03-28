#ifndef SRM_IC_2023_MODULES_COMMON_FRAME_H_
#define SRM_IC_2023_MODULES_COMMON_FRAME_H_

#include <opencv2/core/mat.hpp>
#include "packet.h"

/// 帧信息结构体
struct Frame {
  cv::Mat image;                 ///< 获取的图像
  ReceivePacket receive_packet;  ///< 串口接收的信息
  uint64_t time_stamp;           ///< 时间戳，单位 ns
};

/// 帧回调函数类型
using FrameCallback = std::function<void(void *, Frame &)> *;

#endif  // SRM_IC_2023_MODULES_COMMON_FRAME_H_

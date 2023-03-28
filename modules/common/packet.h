#ifndef SRM_IC_2023_MODULES_COMMON_PACKET_H_
#define SRM_IC_2023_MODULES_COMMON_PACKET_H_

#include <ostream>
#include "syntactic-sugar.h"

/// 串口接收数据结构体
struct ReceivePacket {
  int mode;            ///< 当前瞄准模式
  int armor_kind;      ///< 前哨站装甲板模式
  int reserved_i;      ///< 保留整数位
  int color;           ///< 自身颜色
  float bullet_speed;  ///< 子弹速度
  float yaw;           ///< 自身 yaw
  float pitch;         ///< 自身 pitch
  float roll;          ///< 自身 roll
  float reserved_f;    ///< 保留浮点位
};

/// 串口发送数据结构体
struct SendPacket {
  float yaw;          ///< 目标 yaw
  float pitch;        ///< 目标 pitch
  float reserved_f;   ///< 保留浮点位
  int distance_mode;  ///< 哨兵距离模式
  int fire;           ///< 哨兵是否开火
  int reserved_i[8];  ///< 保留整数位
  float check_sum;    ///< 校验和
};

std::ostream &operator<<(std::ostream REF_OUT str, ReceivePacket REF_IN packet);
std::ostream &operator<<(std::ostream REF_OUT str, SendPacket REF_IN packet);

#endif  // SRM_IC_2023_MODULES_COMMON_PACKET_H_

#ifndef SRM_IC_2023_MODULES_SERIAL_SERIAL_H_
#define SRM_IC_2023_MODULES_SERIAL_SERIAL_H_

#include <mutex>
#include "common/packet.h"

namespace serial {
/// 串口通信接口
class Serial final {
 public:
  Serial() = default;
  ~Serial();

  /**
   * @brief 打开串口通信
   * @return 是否打开成功
   */
  bool Open();

  /// 关闭串口通信
  void Close();

  /**
   * @brief 读取数据
   * @param [out] data 接收数据包
   * @return 是否读取成功
   */
  bool ReadData(ReceivePacket REF_OUT data);

  /**
   * @brief 发送数据
   * @param [in] data 发送数据包
   * @return 是否发送成功
   */
  bool WriteData(SendPacket REF_IN data);

 private:
  bool OpenPort();
  void ClosePort();
  bool SerialSend();
  bool SerialReceive();

  std::string serial_port_;             ///< 串口端口号
  int serial_fd_{};                     ///< 文件描述符
  bool com_flag_{};                     ///< 通信标志
  std::timed_mutex send_data_lock_;     ///< 发送锁
  std::timed_mutex receive_data_lock_;  ///< 接收锁
  SendPacket send_data_{};              ///< 发送数据暂存
  ReceivePacket receive_data_{};        ///< 接收数据暂存
};
}

#endif  // SRM_IC_2023_MODULES_SERIAL_SERIAL_H_

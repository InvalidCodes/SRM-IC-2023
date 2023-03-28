#ifndef SRM_IC_2023_MODULES_CLI_ARG_PARSER_CLI_ARG_PARSER_H_
#define SRM_IC_2023_MODULES_CLI_ARG_PARSER_CLI_ARG_PARSER_H_

#include "common/syntactic-sugar.h"

namespace cli {
/// 命令行参数解析、封装类
class CliArgParser final {
 public:
  /**
   * @brief 获取 CliArgParser 类唯一实例
   * @return 唯一实例的引用
   */
  static CliArgParser &Instance();

  /// 机器人类型
  attr_reader_ref(controller_type_, ControllerType)
  /// 视频源类型
  attr_reader_ref(video_source_type_, VideoSourceType)
  /// 是否开启视频录制
  attr_reader_val(record_, Record)
  /// 是否开启串口通信
  attr_reader_val(serial_, Serial)
  /// 是否显示界面
  attr_reader_val(ui_, UI)

  /**
   * @brief 解析命令行参数
   * @param argc 命令行参数 ARGC，从主函数中获取
   * @param [in, out] argv 命令行参数 ARGV，从主函数中获取
   */
  void Parse(int argc, char **argv);

 private:
  /// @note 该类用于封装全局变量，故采用单例模式
  CliArgParser() = default;
  ~CliArgParser() = default;

  std::string controller_type_;    ///< 机器人类型
  std::string video_source_type_;  ///< 视频源类型
  bool record_{};                  ///< 是否开启视频录制
  bool serial_{};                  ///< 是否开启串口通信
  bool ui_{};                      ///< 是否显示界面
};
}

extern cli::CliArgParser &cli_argv;  ///< 封装命令行参数的全局变量

#endif  // SRM_IC_2023_MODULES_CLI_ARG_PARSER_CLI_ARG_PARSER_H_

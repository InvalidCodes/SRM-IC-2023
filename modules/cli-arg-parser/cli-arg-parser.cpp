#include <glog/logging.h>
#include "cli-arg-parser.h"

DEFINE_string(controller_type, "sentry", "controller type");
DEFINE_string(video_source_type, "file", "video source type");
DEFINE_bool(record, false, "record ui to video in cache directory");
DEFINE_bool(serial, false, "open serial control");
DEFINE_bool(ui, true, "with opencv ui window");

cli::CliArgParser &cli_argv = cli::CliArgParser::Instance();

cli::CliArgParser &cli::CliArgParser::Instance() {
  static CliArgParser parser;
  return parser;
}

void cli::CliArgParser::Parse(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_log_dir = "../log/";
  FLAGS_log_prefix = true;
  FLAGS_stop_logging_if_full_disk = true;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  controller_type_ = FLAGS_controller_type;
  video_source_type_ = FLAGS_video_source_type;
  std::ostringstream cli_flags;
  record_ = FLAGS_record;
  serial_ = FLAGS_serial;
  ui_ = FLAGS_ui;
}

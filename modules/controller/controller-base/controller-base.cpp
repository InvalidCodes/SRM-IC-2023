#include <glog/logging.h>
#include "controller-base.h"
#include "cli-arg-parser/cli-arg-parser.h"

bool controller::Controller::Initialize(std::string REF_IN type_name) {
  video_source_.reset(video_source::CreateVideoSource(cli_argv.VideoSourceType()));
  if (!video_source_) {
    LOG(ERROR) << "Failed to create type " << type_name << " video source.";
    return false;
  }
  if (!video_source_->Initialize("../config/" + type_name + "/" + cli_argv.VideoSourceType() + "-init.yaml")) {
    LOG(ERROR) << "Failed to initialize video source.";
    video_source_.reset();
    return false;
  }
  if (!coord_solver_.Initialize("../config/" + type_name + "/coord-init.yaml",
                                video_source_->IntrinsicMat(), video_source_->DistortionMat())) {
    LOG(ERROR) << "Failed to initialize coordinate solver.";
    video_source_.reset();
    return false;
  }
  if (cli_argv.Serial()) {
    serial_ = std::make_unique<serial::Serial>();
    if (!serial_->Open()) {
      LOG(ERROR) << "Failed to open serial communication.";
      video_source_.reset();
      serial_->Close();
      return false;
    }
  }
  video_source_->RegisterFrameCallback(&FrameCallback, this);
  Frame frame;
  while (!video_source_->GetFrame(frame)) sleep(1);
  if (cli_argv.Record()) {
    time_t t = time(nullptr);
    char t_str[32];
    strftime(t_str, sizeof(t_str), "%Y-%m-%d-%H.%M.%S", localtime(&t));
    std::string video_file = "../cache/" + type_name + "-" + t_str + ".mp4";
    video_writer_.Open("../cache/" + type_name + "-" + t_str + ".mp4", {frame.image.cols, frame.image.rows});
  }
  LOG(INFO) << "Initialized base environment of " << type_name << " controller.";
  return true;
}

std::function<void(void *obj, Frame &)> controller::Controller::FrameCallback = [](void *obj, Frame &frame) {
  auto self = static_cast<Controller *>(obj);
  if (self->serial_ && !self->serial_->ReadData(frame.receive_packet))
    LOG(WARNING) << "Failed to read data from serial port in frame callback function.";
};

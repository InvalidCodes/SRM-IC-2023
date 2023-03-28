#include <glog/logging.h>
#include <opencv2/core/persistence.hpp>
#include "camera-base/camera-base.h"
#include "video-source-camera.h"

video_source::Registry<video_source::camera::CameraVideoSource>
    video_source::camera::CameraVideoSource::registry_("camera");

bool video_source::camera::CameraVideoSource::Initialize(std::string REF_IN config_file) {
  cv::FileStorage camera_init_config;
  camera_init_config.open(config_file, cv::FileStorage::READ);
  if (!camera_init_config.isOpened()) {
    LOG(ERROR) << "Failed to open camera initialization file " << config_file << ".";
    return false;
  }
  std::string all_cams_config_file;
  camera_init_config["ALL_CAMS_CONFIG_FILE"] >> all_cams_config_file;
  if (all_cams_config_file.empty()) {
    LOG(ERROR) << "All cameras' config file configuration not found.";
    return false;
  }
  cv::FileStorage all_cams_config;
  all_cams_config.open(all_cams_config_file, cv::FileStorage::READ);
  if (!all_cams_config.isOpened()) {
    LOG(ERROR) << "Failed to open all cameras' config file " << all_cams_config_file << ".";
    return false;
  }
  std::string all_lens_config_file;
  camera_init_config["ALL_LENS_CONFIG_FILE"] >> all_lens_config_file;
  if (all_lens_config_file.empty()) {
    LOG(ERROR) << "All lens' config file configuration not found.";
    return false;
  }
  cv::FileStorage all_lens_config;
  all_lens_config.open(all_lens_config_file, cv::FileStorage::READ);
  if (!all_lens_config.isOpened()) {
    LOG(ERROR) << "Failed to open all lens' config file " << all_lens_config_file << ".";
    return false;
  }
  std::string camera_type;
  all_cams_config[camera_init_config["CAMERA"]]["TYPE"] >> camera_type;
  if (camera_type.empty()) {
    LOG(ERROR) << "Camera type configuration not found.";
    return false;
  }
  camera_.reset(::camera::CreateCamera(camera_type));
  if (!camera_) {
    LOG(ERROR) << "Failed to create camera object of type " << camera_type << ".";
    return false;
  }
  auto clean = [&]() {
    camera_.reset();
    intrinsic_mat_.release();
    distortion_mat_.release();
  };
  std::string len_type;
  all_cams_config[camera_init_config["CAMERA"]]["LEN"] >> len_type;
  all_lens_config[len_type]["IntrinsicMatrix"] >> intrinsic_mat_;
  all_lens_config[len_type]["DistortionMatrix"] >> distortion_mat_;
  if (intrinsic_mat_.empty() || distortion_mat_.empty()) {
    LOG(ERROR) << "Camera len configurations not found.";
    clean();
    return false;
  }
  std::string serial_number, camera_config_file;
  all_cams_config[camera_init_config["CAMERA"]]["SN"] >> serial_number;
  all_cams_config[camera_init_config["CAMERA"]]["CONFIG"] >> camera_config_file;
  if (serial_number.empty() || camera_config_file.empty()) {
    LOG(ERROR) << "Camera configurations not found.";
    clean();
    return false;
  }
  if (!camera_->OpenCamera(serial_number, camera_config_file)) {
    clean();
    return false;
  }
  int exposure_time;
  all_cams_config[camera_init_config["CAMERA"]]["EXPOSURE_TIME"] >> exposure_time;
  if (exposure_time > 0) camera_->SetExposureTime(static_cast<uint32_t>(exposure_time));
  float gain_value;
  all_cams_config[camera_init_config["CAMERA"]]["GAIN_VALUE"] >> gain_value;
  if (gain_value > 0) camera_->SetGainValue(gain_value);
  if (!camera_->StartStream()) {
    clean();
    return false;
  }
  LOG(INFO) << "Initialized camera video source.";
  return true;
}

bool video_source::camera::CameraVideoSource::GetFrame(Frame REF_OUT frame) {
  return camera_ != nullptr && camera_->GetFrame(frame);
}

void video_source::camera::CameraVideoSource::RegisterFrameCallback(FrameCallback callback, void *obj) {
  if (camera_) camera_->RegisterFrameCallback(callback, obj);
  DLOG(INFO) << "Registered video source callback FUNC " << callback << " OBJ " << obj << ".";
}

void video_source::camera::CameraVideoSource::UnregisterFrameCallback(FrameCallback callback) {
  if (camera_) camera_->UnregisterFrameCallback(callback);
  DLOG(INFO) << "Unregistered video source callback FUNC " << callback << ".";
}

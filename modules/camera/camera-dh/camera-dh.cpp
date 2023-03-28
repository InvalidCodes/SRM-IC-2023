#include <glog/logging.h>
#include <GxIAPI.h>
#include <DxImageProc.h>
#include "camera-dh.h"

#define GX_OPEN_CAMERA_CHECK_STATUS(status_code)  \
  if ((status_code) != GX_STATUS_SUCCESS) {       \
    LOG(ERROR) << GetErrorInfo(status_code);      \
    (status_code) = GXCloseDevice(device_);       \
    if ((status_code) != GX_STATUS_SUCCESS)       \
      LOG(ERROR) << GetErrorInfo(status_code);    \
    device_ = nullptr;                            \
    serial_number_ = "";                          \
    if (!camera_count_) {                         \
      (status_code) = GXCloseLib();               \
      if ((status_code) != GX_STATUS_SUCCESS)     \
        LOG(ERROR) << GetErrorInfo(status_code);  \
    }                                             \
    return false;                                 \
  }

#define GX_CHECK_STATUS(status_code)          \
  if ((status_code) != GX_STATUS_SUCCESS) {   \
    LOG(ERROR) << GetErrorInfo(status_code);  \
    return false;                             \
  }

#define GX_START_STOP_STREAM_CHECK_STATUS(status_code)  \
  if ((status_code) != GX_STATUS_SUCCESS) {             \
    if (raw_16_to_8_cache_) {                           \
      delete[] raw_16_to_8_cache_;                      \
      raw_16_to_8_cache_ = nullptr;                     \
    }                                                   \
    if (raw_8_to_rgb_24_cache_) {                       \
      delete[] raw_8_to_rgb_24_cache_;                  \
      raw_8_to_rgb_24_cache_ = nullptr;                 \
    }                                                   \
    LOG(ERROR) << GetErrorInfo(status_code);            \
    return false;                                       \
  }

uint16_t camera::dh::DHCamera::camera_count_ = 0;
camera::Registry<camera::dh::DHCamera> camera::dh::DHCamera::registry_("DHCamera");

camera::dh::DHCamera::~DHCamera() {
  if (stream_running_) StopStream();
  if (device_) CloseCamera();
}

bool camera::dh::DHCamera::OpenCamera(std::string REF_IN serial_number, std::string REF_IN config_file) {
  if (device_) return false;
  if (!camera_count_ && GXInitLib() != GX_STATUS_SUCCESS)
    throw std::runtime_error("GX libraries not found.");
  GX_STATUS status_code;
  GX_OPEN_PARAM open_param;
  std::unique_ptr<char[]> sn(new char[1 + serial_number.size()]);
  strcpy(sn.get(), serial_number.c_str());
  open_param.accessMode = GX_ACCESS_EXCLUSIVE;
  open_param.openMode = GX_OPEN_SN;
  open_param.pszContent = sn.get();
  status_code = GXOpenDevice(&open_param, &device_);
  if (status_code != GX_STATUS_SUCCESS) {
    LOG(ERROR) << GetErrorInfo(status_code);
    device_ = nullptr;
    if (!camera_count_) {
      status_code = GXCloseLib();
      if (status_code != GX_STATUS_SUCCESS)
        LOG(ERROR) << GetErrorInfo(status_code);
    }
    return false;
  }
  serial_number_ = serial_number;
  bool has_color_filter = false;
  status_code = GXIsImplemented(device_, GX_ENUM_PIXEL_COLOR_FILTER, &has_color_filter);
  GX_OPEN_CAMERA_CHECK_STATUS(status_code)
  if (!has_color_filter) {
    LOG(ERROR) << "{!}{Mono cameras are NOT supported}";
    status_code = GXCloseDevice(device_);
    if (status_code != GX_STATUS_SUCCESS)
      LOG(ERROR) << GetErrorInfo(status_code);
    device_ = nullptr;
    serial_number_ = "";
    if (!camera_count_) {
      status_code = GXCloseLib();
      if (status_code != GX_STATUS_SUCCESS)
        LOG(ERROR) << GetErrorInfo(status_code);
    }
    return false;
  } else {
    status_code = GXGetEnum(device_, GX_ENUM_PIXEL_COLOR_FILTER, &color_filter_);
    GX_OPEN_CAMERA_CHECK_STATUS(status_code)
  }
  status_code = GXGetInt(device_, GX_INT_PAYLOAD_SIZE, &payload_size_);
  GX_OPEN_CAMERA_CHECK_STATUS(status_code)
  if (!config_file.empty() && !ImportConfigurationFile(config_file)) {
    status_code = GXCloseDevice(device_);
    if (status_code != GX_STATUS_SUCCESS)
      LOG(ERROR) << GetErrorInfo(status_code);
    device_ = nullptr;
    serial_number_ = "";
    if (!camera_count_) {
      status_code = GXCloseLib();
      if (status_code != GX_STATUS_SUCCESS)
        LOG(ERROR) << GetErrorInfo(status_code);
    }
    return false;
  }
  ++camera_count_;
  RegisterCaptureCallback(&DefaultCaptureCallback);
  if (!daemon_thread_id_) {
    stop_flag_ = false;
    pthread_create(&daemon_thread_id_, nullptr, DaemonThreadFunction, this);
    DLOG(INFO) << serial_number_ << "'s daemon thread " << std::to_string(daemon_thread_id_) << " started.";
  }
  LOG(INFO) << "Opened camera " << serial_number_ << ".";
  return true;
}

bool camera::dh::DHCamera::CloseCamera() {
  if (stream_running_) return false;
  if (!device_) return false;
  stop_flag_ = true;
  pthread_join(daemon_thread_id_, nullptr);
  DLOG(INFO) << serial_number_ << "'s daemon thread " << std::to_string(daemon_thread_id_) << " stopped.";
  daemon_thread_id_ = 0;
  stop_flag_ = false;
  GX_STATUS status_code = GXCloseDevice(device_);
  device_ = nullptr;
  --camera_count_;
  if (status_code != GX_STATUS_SUCCESS) {
    LOG(ERROR) << GetErrorInfo(status_code);
    if (!camera_count_) {
      status_code = GXCloseLib();
      if (status_code != GX_STATUS_SUCCESS)
        LOG(ERROR) << GetErrorInfo(status_code);
    }
    return false;
  }
  if (!camera_count_) {
    status_code = GXCloseLib();
    if (status_code != GX_STATUS_SUCCESS) {
      LOG(ERROR) << GetErrorInfo(status_code);
      return false;
    }
  }
  LOG(INFO) << "Closed camera " << serial_number_ << ".";
  color_filter_ = GX_COLOR_FILTER_NONE;
  payload_size_ = 0;
  serial_number_ = "";
  return true;
}

bool camera::dh::DHCamera::StartStream() {
  if (!device_) return false;
  if (stream_running_) return false;
  ExportConfigurationFile("../cache/" + serial_number_ + ".txt");
  raw_8_to_rgb_24_cache_ = new unsigned char[payload_size_ * 3];
  raw_16_to_8_cache_ = new unsigned char[payload_size_];
  GX_STATUS status_code = GXStreamOn(device_);
  GX_START_STOP_STREAM_CHECK_STATUS(status_code)
  stream_running_ = true;
  LOG(INFO) << serial_number_ << "'s stream started.";
  return true;
}

bool camera::dh::DHCamera::StopStream() {
  if (!device_) return false;
  if (!stream_running_) return false;
  stream_running_ = false;
  GX_STATUS status_code = GXStreamOff(device_);
  GX_START_STOP_STREAM_CHECK_STATUS(status_code)
  if (raw_16_to_8_cache_ != nullptr) {
    delete[] raw_16_to_8_cache_;
    raw_16_to_8_cache_ = nullptr;
  }
  if (raw_8_to_rgb_24_cache_ != nullptr) {
    delete[] raw_8_to_rgb_24_cache_;
    raw_8_to_rgb_24_cache_ = nullptr;
  }
  LOG(INFO) << serial_number_ << "'s stream stopped.";
  return true;
}

bool camera::dh::DHCamera::IsConnected() {
  if (!device_) return false;
  GX_STATUS status_code;
  uint32_t device_num;
  status_code = GXUpdateDeviceList(&device_num, 1000);
  if (status_code != GX_STATUS_SUCCESS || device_num <= 0) {
    LOG(ERROR) << GetErrorInfo(status_code);
    return false;
  }
  std::unique_ptr<GX_DEVICE_BASE_INFO[]> device_list(new GX_DEVICE_BASE_INFO[device_num]);
  size_t act_size = device_num * sizeof(GX_DEVICE_BASE_INFO);
  status_code = GXGetAllDeviceBaseInfo(device_list.get(), &act_size);
  if (status_code != GX_STATUS_SUCCESS) {
    LOG(ERROR) << GetErrorInfo(status_code);
    return false;
  }
  for (uint32_t i = 0; i < device_num; ++i)
    if (device_list[i].szSN == this->serial_number_)
      return true;
  return false;
}

bool camera::dh::DHCamera::GetFrame(Frame REF_OUT frame) {
  if (!device_) return false;
  return buffer_.Pop(frame);
}

bool camera::dh::DHCamera::ExportConfigurationFile(std::string REF_IN config_file) {
  if (!device_) return false;
  GX_STATUS status_code = GXExportConfigFile(device_, config_file.c_str());
  GX_CHECK_STATUS(status_code)
  LOG(INFO) << "Saved " << serial_number_ << "'s configuration to " << config_file << ".";
  return true;
}

bool camera::dh::DHCamera::ImportConfigurationFile(std::string REF_IN config_file) {
  if (!device_) return false;
  GX_STATUS status_code = GXImportConfigFile(device_, config_file.c_str());
  GX_CHECK_STATUS(status_code)
  LOG(INFO) << "Imported " << serial_number_ << "'s configuration from " << config_file << ".";
  return true;
}

bool camera::dh::DHCamera::SetExposureTime(uint32_t exposure_time) {
  if (!device_) return false;
  if (!SetExposureMode(GX_EXPOSURE_MODE_TIMED)) {
    LOG(ERROR) << "Failed to set " << serial_number_ << "'s exposure mode to timed.";
    return false;
  }
  if (!SetExposureTimeMode(GX_EXPOSURE_TIME_MODE_STANDARD)) {
    LOG(ERROR) << "Failed to set " << serial_number_ << "'s exposure time mode to standard.";
    return false;
  }
  return SetExposureTimeDHImplementation(static_cast<int64_t>(exposure_time));
}

bool camera::dh::DHCamera::SetGainValue(float gain_value) {
  if (!device_) return false;
  if (!SetGainAuto(GX_GAIN_AUTO_OFF)) {
    LOG(ERROR) << "Failed to turn " << serial_number_ << "'s gain auto mode off.";
    return false;
  }
  return SetGainValueDHImplementation(static_cast<double>(gain_value));
}

void camera::dh::DHCamera::DefaultCaptureCallback(GX_FRAME_CALLBACK_PARAM *frame_callback) {
  auto *self = (DHCamera *) frame_callback->pUserParam;
  if (frame_callback->status != GX_STATUS_SUCCESS) {
    LOG(ERROR) << GetErrorInfo(frame_callback->status);
    return;
  }
  if (!self->Raw8Raw16ToRGB24(frame_callback)) return;
  Frame frame;
  cv::Mat image(frame_callback->nHeight, frame_callback->nWidth, CV_8UC3, self->raw_8_to_rgb_24_cache_);
  frame.image = image.clone();
  frame.time_stamp = frame_callback->nTimestamp;
  for (auto p : self->callback_list_)
    (*p.first)(p.second, frame);
  self->buffer_.Push(std::move(frame));
}

void *camera::dh::DHCamera::DaemonThreadFunction(void *obj) {
  auto self = static_cast<DHCamera *>(obj);
  while (!self->stop_flag_) {
    sleep(1);
    if (!self->IsConnected()) {
      LOG(ERROR) << self->serial_number_ << " is disconnected unexpectedly.";
      LOG(INFO) << "Preparing for reconnection...";
      if (self->stream_running_) {
        GXStreamOff(self->device_);
        if (self->raw_16_to_8_cache_) {
          delete[] self->raw_16_to_8_cache_;
          self->raw_16_to_8_cache_ = nullptr;
        }
        if (self->raw_8_to_rgb_24_cache_) {
          delete[] self->raw_8_to_rgb_24_cache_;
          self->raw_8_to_rgb_24_cache_ = nullptr;
        }
      }
      self->UnregisterCaptureCallback();
      --camera_count_;
      GXCloseDevice(self->device_);
      self->device_ = nullptr;
      self->payload_size_ = 0;
      self->color_filter_ = GX_COLOR_FILTER_NONE;
      self->daemon_thread_id_ = 0;
      if (!camera_count_)
        GXCloseLib();
      while (!self->OpenCamera(self->serial_number_, "../cache/" + self->serial_number_ + ".txt"))
        sleep(1);
      if (self->stream_running_) {
        self->raw_8_to_rgb_24_cache_ = new unsigned char[self->payload_size_ * 3];
        self->raw_16_to_8_cache_ = new unsigned char[self->payload_size_];
        GX_STATUS status_code = GXStreamOn(self->device_);
        if (status_code != GX_STATUS_SUCCESS) {
          LOG(ERROR) << GetErrorInfo(status_code);
          GXStreamOff(self->device_);
          if (self->raw_16_to_8_cache_) {
            delete[] self->raw_16_to_8_cache_;
            self->raw_16_to_8_cache_ = nullptr;
          }
          if (self->raw_8_to_rgb_24_cache_) {
            delete[] self->raw_8_to_rgb_24_cache_;
            self->raw_8_to_rgb_24_cache_ = nullptr;
          }
          self->stream_running_ = false;
        }
      }
      LOG(INFO) << self->serial_number_ << " is successfully reconnected.";
    }
  }
  return nullptr;
}

bool camera::dh::DHCamera::RegisterCaptureCallback(GXCaptureCallBack callback) {
  if (!device_) return false;
  GX_STATUS status_code;
  status_code = GXRegisterCaptureCallback(device_, this, callback);
  GX_CHECK_STATUS(status_code)
  DLOG(INFO) << "Registered " << serial_number_ << "'s capture callback.";
  return true;
}

bool camera::dh::DHCamera::UnregisterCaptureCallback() {
  if (!device_) return false;
  GX_STATUS status_code;
  status_code = GXUnregisterCaptureCallback(device_);
  GX_CHECK_STATUS(status_code)
  DLOG(INFO) << "Unregistered " << serial_number_ << "'s capture callback.";
  return true;
}

bool camera::dh::DHCamera::SetExposureTimeDHImplementation(int64_t exposure_time) {
  GX_STATUS status_code;
  status_code = GXSetEnum(device_, GX_FLOAT_EXPOSURE_TIME, exposure_time);
  GX_CHECK_STATUS(status_code)
  DLOG(INFO) << "Set " << serial_number_ << "'s exposure time to " << std::to_string(exposure_time) << ".";
  return true;
}

bool camera::dh::DHCamera::SetExposureMode(GX_EXPOSURE_MODE_ENTRY gx_exposure_mode_entry) {
  GX_STATUS status_code;
  status_code = GXSetEnum(device_, GX_ENUM_EXPOSURE_MODE, gx_exposure_mode_entry);
  GX_CHECK_STATUS(status_code)
  DLOG(INFO) << "Set " << serial_number_ << "'s exposure mode to " << std::to_string(gx_exposure_mode_entry) << ".";
  return true;
}

bool camera::dh::DHCamera::SetExposureTimeMode(GX_EXPOSURE_TIME_MODE_ENTRY gx_exposure_time_mode_entry) {
  GX_STATUS status_code;
  status_code = GXSetEnum(device_, GX_ENUM_EXPOSURE_TIME_MODE, gx_exposure_time_mode_entry);
  GX_CHECK_STATUS(status_code)
  DLOG(INFO) << "Set " << serial_number_ << "'s exposure time mode to "
             << std::to_string(gx_exposure_time_mode_entry) << ".";
  return true;
}

bool camera::dh::DHCamera::SetGainValueDHImplementation(double gain) {
  GX_STATUS status_code;
  status_code = GXSetEnum(device_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
  GX_CHECK_STATUS(status_code)
  status_code = GXSetFloat(device_, GX_FLOAT_GAIN, static_cast<float>(gain));
  GX_CHECK_STATUS(status_code)
  double value = 0;
  status_code = GXGetFloat(device_, GX_FLOAT_GAIN, &value);
  GX_CHECK_STATUS(status_code)
  DLOG(INFO) << "Set " << serial_number_ << "'s gain value to " << std::to_string(gain) << ".";
  return true;
}

bool camera::dh::DHCamera::SetGainAuto(GX_GAIN_AUTO_ENTRY gx_gain_auto_entry) {
  GX_STATUS status_code;
  status_code = GXSetEnum(device_, GX_ENUM_GAIN_AUTO, gx_gain_auto_entry);
  GX_CHECK_STATUS(status_code)
  DLOG(INFO) << "Set " << serial_number_ << "'s gain to auto mode " << std::to_string(gx_gain_auto_entry) << ".";
  return true;
}

std::string camera::dh::DHCamera::GetErrorInfo(GX_STATUS error_status_code) {
  size_t str_size = 0;
  if (GXGetLastError(&error_status_code, nullptr, &str_size) != GX_STATUS_SUCCESS) {
    return "{!}{Memory full}";
  }
  char *error_info = new char[str_size];
  if (GXGetLastError(&error_status_code, error_info, &str_size) != GX_STATUS_SUCCESS) {
    delete[] error_info;
    return "{?}{Unknown error}";
  }
  return error_info;
}

bool camera::dh::DHCamera::Raw8Raw16ToRGB24(GX_FRAME_CALLBACK_PARAM *frame_callback) {
  VxInt32 dx_status_code;
  switch (frame_callback->nPixelFormat) {
    case GX_PIXEL_FORMAT_BAYER_GR8:
    case GX_PIXEL_FORMAT_BAYER_RG8:
    case GX_PIXEL_FORMAT_BAYER_GB8:
    case GX_PIXEL_FORMAT_BAYER_BG8: {
      dx_status_code = DxRaw8toRGB24Ex((unsigned char *) frame_callback->pImgBuf,
                                       raw_8_to_rgb_24_cache_,
                                       frame_callback->nWidth,
                                       frame_callback->nHeight,
                                       RAW2RGB_NEIGHBOUR,
                                       DX_PIXEL_COLOR_FILTER(color_filter_),
                                       false, DX_ORDER_BGR);
      if (dx_status_code != DX_OK) {
        LOG(ERROR) << "DxRaw8toRGB24Ex failed with error " << std::to_string(dx_status_code) << ".";
        return false;
      }
      break;
    }
    case GX_PIXEL_FORMAT_BAYER_GR10:
    case GX_PIXEL_FORMAT_BAYER_RG10:
    case GX_PIXEL_FORMAT_BAYER_GB10:
    case GX_PIXEL_FORMAT_BAYER_BG10:
    case GX_PIXEL_FORMAT_BAYER_GR12:
    case GX_PIXEL_FORMAT_BAYER_RG12:
    case GX_PIXEL_FORMAT_BAYER_GB12:
    case GX_PIXEL_FORMAT_BAYER_BG12: {
      dx_status_code = DxRaw16toRaw8((unsigned char *) frame_callback->pImgBuf,
                                     raw_16_to_8_cache_,
                                     frame_callback->nWidth,
                                     frame_callback->nHeight,
                                     DX_BIT_2_9);
      if (dx_status_code != DX_OK) {
        LOG(ERROR) << "DxRaw8toRGB24 failed with error " << std::to_string(dx_status_code) << ".";
        return false;
      }
      dx_status_code = DxRaw8toRGB24Ex(raw_16_to_8_cache_,
                                       raw_8_to_rgb_24_cache_,
                                       frame_callback->nWidth,
                                       frame_callback->nHeight,
                                       RAW2RGB_NEIGHBOUR,
                                       DX_PIXEL_COLOR_FILTER(color_filter_),
                                       false, DX_ORDER_BGR);
      if (dx_status_code != DX_OK) {
        LOG(ERROR) << "DxRaw8toRGB24Ex failed with error " << std::to_string(dx_status_code) << ".";
        return false;
      }
      break;
    }
    default: {
      LOG(ERROR) << "Pixel format of this camera is not supported.";
      return false;
    }
  }
  return true;
}

#include <glog/logging.h>
#include <opencv2/imgproc.hpp>
#include <MvCameraControl.h>
#include "camera-hik.h"

camera::Registry<camera::hik::HikCamera> camera::hik::HikCamera::registry_("HikCamera");

camera::hik::HikCamera::~HikCamera() {
  if (stream_running_) StopStream();
  if (device_) CloseCamera();
}

bool camera::hik::HikCamera::OpenCamera(std::string REF_IN serial_number, std::string REF_IN config_file) {
  if (device_) return false;
  MV_CC_DEVICE_INFO_LIST device_list;
  memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  auto status_code = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
  if (MV_OK != status_code) {
    LOG(ERROR) << "Failed to enumerate devices with error code " << "0x" << std::hex << status_code << ".";
    return false;
  }
  if (device_list.nDeviceNum > 0) {
    for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
      MV_CC_DEVICE_INFO *device_info = device_list.pDeviceInfo[i];
      std::string sn;
      switch (device_info->nTLayerType) {
        case MV_GIGE_DEVICE: {
          sn = std::string(reinterpret_cast<char *>(device_info->SpecialInfo.stGigEInfo.chSerialNumber));
          DLOG(INFO) << "GigE device with serial number " << sn << " found.";
          break;
        }
        case MV_USB_DEVICE: {
          sn = std::string(reinterpret_cast<char *>(device_info->SpecialInfo.stUsb3VInfo.chSerialNumber));
          DLOG(INFO) << "USB device with serial number " << sn << " found.";
          break;
        }
        default: {
          LOG(WARNING) << "Device with unsupported device transport layer protocol type " << sn << "found.";
        }
      }
      if (sn == serial_number) {
        LOG(INFO) << "Found device with serial number " << serial_number << ".";
        status_code = MV_CC_CreateHandle(&device_, device_info);
        if (MV_OK != status_code) {
          LOG(ERROR) << "Failed to create device handle with error " << "0x" << std::hex << status_code << ".";
          return false;
        }
        break;
      }
    }
  } else {
    LOG(ERROR) << "No device found.";
    return false;
  }
  if (!device_) {
    LOG(ERROR) << "Device with serial number " << serial_number << " not found.";
    return false;
  }
  status_code = MV_CC_OpenDevice(device_, MV_ACCESS_Exclusive, 1);
  if (MV_OK != status_code) {
    LOG(ERROR) << "Failed to open device with error " << "0x" << std::hex << status_code << ".";
    status_code = MV_CC_DestroyHandle(device_);
    if (MV_OK != status_code)
      LOG(ERROR) << "Failed to destroy device handle with error " << "0x" << std::hex << status_code << ".";
    return false;
  }
  if (!config_file.empty() && !ImportConfigurationFile(config_file)) {
    status_code = MV_CC_CloseDevice(device_);
    if (MV_OK != status_code)
      LOG(ERROR) << "Failed to close device with error " << "0x" << std::hex << status_code << ".";
    status_code = MV_CC_DestroyHandle(device_);
    if (MV_OK != status_code)
      LOG(ERROR) << "Failed to destroy device handle with error " << "0x" << std::hex << status_code << ".";
    return false;
  }
  status_code = MV_CC_RegisterImageCallBackEx(device_, ImageCallbackEx, this);
  if (MV_OK != status_code) {
    LOG(ERROR) << "Failed to register image callback with error " << "0x" << std::hex << status_code << ".";
    status_code = MV_CC_CloseDevice(device_);
    if (MV_OK != status_code)
      LOG(ERROR) << "Failed to close device with error " << "0x" << std::hex << status_code << ".";
    status_code = MV_CC_DestroyHandle(device_);
    if (MV_OK != status_code)
      LOG(ERROR) << "Failed to destroy device handle with error " << "0x" << std::hex << status_code << ".";
    return false;
  } else {
    DLOG(INFO) << "Registered " << serial_number_ << "'s capture callback.";
  }
  serial_number_ = serial_number;
  if (!daemon_thread_id_) {
    stop_flag_ = false;
    pthread_create(&daemon_thread_id_, nullptr, DaemonThreadFunction, this);
    DLOG(INFO) << serial_number_ << "'s daemon thread " << std::to_string(daemon_thread_id_) << " started.";
  }
  LOG(INFO) << "Opened camera " << serial_number << ".";
  return true;
}

bool camera::hik::HikCamera::CloseCamera() {
  if (stream_running_) return false;
  if (!device_) return false;
  stop_flag_ = true;
  pthread_join(daemon_thread_id_, nullptr);
  stop_flag_ = false;
  DLOG(INFO) << serial_number_ << "'s daemon thread " << std::to_string(daemon_thread_id_) << " stopped.";
  auto status_code = MV_CC_CloseDevice(device_);
  if (MV_OK != status_code)
    LOG(ERROR) << "Failed to close camera with error " << "0x" << std::hex << status_code << ".";
  status_code = MV_CC_DestroyHandle(device_);
  if (MV_OK != status_code)
    LOG(ERROR) << "Failed to destroy handle with error " << "0x" << std::hex << status_code << ".";
  LOG(INFO) << "Closed camera " << serial_number_ << ".";
  serial_number_ = "";
  device_ = nullptr;
  daemon_thread_id_ = 0;
  return true;
}

bool camera::hik::HikCamera::StartStream() {
  if (!device_) return false;
  ExportConfigurationFile("../cache/" + serial_number_ + ".txt");
  auto status_code = MV_CC_StartGrabbing(device_);
  if (MV_OK != status_code) {
    LOG(ERROR) << "Failed to start stream with error " << "0x" << std::hex << status_code << ".";
    return false;
  }
  stream_running_ = true;
  LOG(INFO) << serial_number_ << "'s stream started.";
  return true;
}

bool camera::hik::HikCamera::StopStream() {
  if (!device_) return false;
  if (!stream_running_) return false;
  stream_running_ = false;
  auto status_code = MV_CC_StopGrabbing(device_);
  if (MV_OK != status_code) {
    LOG(ERROR) << "Failed to stop stream with error " << "0x" << std::hex << status_code << ".";
    return false;
  }
  LOG(INFO) << serial_number_ << "'s stream stopped.";
  return true;
}

bool camera::hik::HikCamera::GetFrame(Frame REF_OUT frame) {
  if (!device_) return false;
  return buffer_.Pop(frame);
}

bool camera::hik::HikCamera::IsConnected() {
  if (!device_) return false;
  return MV_CC_IsDeviceConnected(device_);
}

bool camera::hik::HikCamera::ImportConfigurationFile(std::string REF_IN config_file) {
  if (!device_) return false;
  auto status_code = MV_CC_FeatureLoad(device_, config_file.c_str());
  if (status_code != MV_OK) {
    LOG(INFO) << "Failed to import " << serial_number_ << "'s configuration to "
              << config_file << " with error 0x" << std::hex << status_code << ".";
    return false;
  }
  LOG(INFO) << "Imported " << serial_number_ << "'s configuration to " << config_file << ".";
  return true;
}

bool camera::hik::HikCamera::ExportConfigurationFile(std::string REF_IN config_file) {
  if (!device_) return false;
  auto status_code = MV_CC_FeatureSave(device_, config_file.c_str());
  if (status_code != MV_OK) {
    LOG(INFO) << "Failed to save " << serial_number_ << "'s configuration to "
              << config_file << " with error 0x" << std::hex << status_code << ".";
    return false;
  }
  LOG(INFO) << "Saved " << serial_number_ << "'s configuration to " << config_file << ".";
  return true;
}

bool camera::hik::HikCamera::SetExposureTime(uint32_t exposure_time) {
  if (!device_) return false;
  return SetExposureTimeHikImplementation((float) exposure_time);
}

bool camera::hik::HikCamera::SetGainValue(float gain) {
  if (!device_) return false;
  if (MV_CC_SetFloatValue(device_, "Gain", gain) != MV_OK) {
    LOG(ERROR) << "Failed to set " << serial_number_ << "'s gain to " << std::to_string(gain) << ".";
    return false;
  }
  DLOG(INFO) << "Set " << serial_number_ << "'s gain to " << std::to_string(gain) << ".";
  return true;
}

void camera::hik::HikCamera::ImageCallbackEx(unsigned char *image_data, MV_FRAME_OUT_INFO_EX *frame_info, void *obj) {
  auto self = static_cast<HikCamera *>(obj);
  Frame frame;
  switch (frame_info->enPixelType) {
    case PixelType_Gvsp_BayerRG8: {
      cv::Mat image(frame_info->nHeight, frame_info->nWidth, CV_8UC1, image_data);
      frame.image = image.clone();
      cv::cvtColor(frame.image, frame.image, cv::COLOR_BayerRG2RGB);
      break;
    }
    case PixelType_Gvsp_BGR8_Packed: {
      cv::Mat image(frame_info->nHeight, frame_info->nWidth, CV_8UC3, image_data);
      frame.image = image.clone();
      break;
    }
    default: LOG(WARNING) << "Unknown pixel type 0x" << std::hex << frame_info->enPixelType << " detected.";
  }
  frame.time_stamp = (uint64_t) frame_info->nDevTimeStampHigh;
  frame.time_stamp <<= 32;
  frame.time_stamp += frame_info->nDevTimeStampLow;
  for (auto p : self->callback_list_)
    (*p.first)(p.second, frame);
  self->buffer_.Push(std::move(frame));
}

void *camera::hik::HikCamera::DaemonThreadFunction(void *obj) {
  auto self = (HikCamera *) obj;
  while (!self->stop_flag_) {
    sleep(1);
    if (!MV_CC_IsDeviceConnected(self->device_)) {
      LOG(ERROR) << self->serial_number_ << " is disconnected unexpectedly.";
      LOG(INFO) << "Preparing for reconnection...";
      MV_CC_StopGrabbing(self->device_);
      MV_CC_CloseDevice(self->device_);
      MV_CC_DestroyHandle(self->device_);
      self->device_ = nullptr;
      while (!self->OpenCamera(self->serial_number_, "../cache/" + self->serial_number_ + ".txt"))
        sleep(1);
      LOG(INFO) << self->serial_number_ << " is successfully reconnected.";
      if (self->stream_running_)
        self->StartStream();
    }
  }
  return nullptr;
}

bool camera::hik::HikCamera::SetExposureTimeHikImplementation(float exposure_time) {
  if (MV_CC_SetFloatValue(device_, "ExposureTime", exposure_time) != MV_OK) {
    LOG(ERROR) << "Failed to set " << serial_number_ << "'s exposure time to " << std::to_string(exposure_time) << ".";
    return false;
  }
  DLOG(INFO) << "Set " << serial_number_ << "'s exposure time to " << std::to_string(exposure_time) << ".";
  return true;
}

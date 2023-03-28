#include <glog/logging.h>
#include "camera-base.h"

void camera::Camera::RegisterFrameCallback(FrameCallback callback, void *obj) {
  callback_list_.emplace_back(callback, obj);
  DLOG(INFO) << "Registered camera callback FUNC " << callback << " OBJ " << obj << ".";
}

void camera::Camera::UnregisterFrameCallback(FrameCallback callback) {
  auto filter = [callback](auto p) { return p.first == callback; };
  callback_list_.erase(std::remove_if(callback_list_.begin(), callback_list_.end(), filter), callback_list_.end());
  DLOG(INFO) << "Unregistered camera callback FUNC " << callback << ".";
}

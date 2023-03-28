#include "packet.h"

std::ostream &operator<<(std::ostream REF_OUT str, ReceivePacket REF_IN packet) {
  return str << packet.mode << " | " << packet.armor_kind << " | " << packet.color << " | " << packet.bullet_speed
             << " | " << packet.yaw << " | " << packet.pitch << " | " << packet.roll;
}

std::ostream &operator<<(std::ostream REF_OUT str, SendPacket REF_IN packet) {
  return str << packet.yaw << " | " << packet.pitch << " | " << packet.distance_mode << " | " << packet.fire;
}

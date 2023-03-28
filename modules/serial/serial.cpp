#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <chrono>
#include <glog/logging.h>
#include "serial.h"

#define LOCK_TIMEOUT 8
#define IO_TIMEOUT 4
#define BAUD_RATE B4000000

std::string GetUartDeviceName() {
  FILE *ls = popen("ls /dev/ttyACM* --color=never", "r");
  char name[127];
  auto ret = fscanf(ls, "%s", name);
  pclose(ls);
  if (ret == -1) {
    LOG(ERROR) << "No UART device found. Please check the device connection.";
    return "";
  } else
    return name;
}

serial::Serial::~Serial() {
  Close();
}

bool serial::Serial::Open() {
  if (com_flag_) return false;
  serial_port_ = GetUartDeviceName();
  if (serial_port_.empty()) return false;
  if (!OpenPort()) {
    serial_port_ = "";
    return false;
  }
  com_flag_ = true;
  return true;
}

void serial::Serial::Close() {
  if (!com_flag_) return;
  ClosePort();
  serial_port_ = "";
  com_flag_ = false;
}

bool serial::Serial::ReadData(ReceivePacket REF_OUT data) {
  if (!com_flag_) return false;
  std::unique_lock<std::timed_mutex> lock(receive_data_lock_, std::chrono::milliseconds(LOCK_TIMEOUT));
  if (lock.owns_lock()) {
    bool ret = SerialReceive();
    data = receive_data_;
    return ret;
  } else {
    LOG(WARNING) << "Reading data from serial port " << serial_port_ << " timed out due to failure to acquire lock.";
    return false;
  }
}

bool serial::Serial::WriteData(SendPacket REF_IN data) {
  if (!com_flag_) return false;
  std::unique_lock<std::timed_mutex> lock(send_data_lock_, std::chrono::milliseconds(LOCK_TIMEOUT));
  if (lock.owns_lock()) {
    send_data_ = data;
    return SerialSend();
  } else {
    LOG(WARNING) << "Writing data to serial port " << serial_port_ << " timed out due to failure to acquire lock.";
    return false;
  }
}

bool serial::Serial::OpenPort() {
  if (chmod(serial_port_.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) == -1) {
    LOG(WARNING) << "Running in user mode, manually setting permission is required.";
    LOG(WARNING) << "To set permission of current serial port, run this command as root:";
    LOG(WARNING) << "  $ chmod 777 " << serial_port_;
  }
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ == -1) {
    LOG(ERROR) << "Failed to open serial port " << serial_port_ << ".";
    serial_fd_ = 0;
    return false;
  }
  auto baud_rate = BAUD_RATE;
  termios termios_option{};
  tcgetattr(serial_fd_, &termios_option);
  cfmakeraw(&termios_option);
  cfsetispeed(&termios_option, baud_rate);
  cfsetospeed(&termios_option, baud_rate);
  tcsetattr(serial_fd_, TCSANOW, &termios_option);
  termios_option.c_cflag &= ~PARENB;
  termios_option.c_cflag &= ~CSTOPB;
  termios_option.c_cflag &= ~CSIZE;
  termios_option.c_cflag |= CS8;
  termios_option.c_cflag &= ~INPCK;
  termios_option.c_cflag |= (baud_rate | CLOCAL | CREAD);
  termios_option.c_cflag &= ~(INLCR | ICRNL);
  termios_option.c_cflag &= ~(IXON);
  termios_option.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  termios_option.c_oflag &= ~OPOST;
  termios_option.c_oflag &= ~(ONLCR | OCRNL);
  termios_option.c_iflag &= ~(ICRNL | INLCR);
  termios_option.c_iflag &= ~(IXON | IXOFF | IXANY);
  termios_option.c_cc[VTIME] = 1;
  termios_option.c_cc[VMIN] = 1;
  tcflush(serial_fd_, TCIOFLUSH);
  LOG(INFO) << "Serial port " << serial_port_ << " is open.";
  return true;
}

void serial::Serial::ClosePort() {
  tcflush(serial_fd_, TCIOFLUSH);
  if (close(serial_fd_) == -1)
    LOG(WARNING) << "Failed to close serial port " << serial_port_ << ".";
  else
    LOG(INFO) << "Serial port " << serial_port_ << " is closed.";
  serial_fd_ = 0;
}

bool serial::Serial::SerialSend() {
  constexpr auto size = sizeof(SendPacket);
  tcflush(serial_fd_, TCOFLUSH);
  ssize_t send_count = write(serial_fd_, &send_data_, size);
  if (send_count < size) {
    LOG(ERROR) << "Failed to send " << size - send_count << " / " << size
               << " bytes of data to serial port " << serial_port_ << ".";
    return false;
  }
  DLOG(INFO) << "Sent " << size << " bytes of data to serial port " << serial_port_ << ".";
  DLOG(INFO) << send_data_;
  return true;
}

bool serial::Serial::SerialReceive() {
  constexpr auto size = sizeof(SendPacket);
  memset(&receive_data_, 0, size);
  ssize_t read_count = 0;
  const auto start_time = std::chrono::high_resolution_clock::now();
  while (read_count < size) {
    auto current_time = std::chrono::high_resolution_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() > IO_TIMEOUT) {
      LOG(ERROR) << "Receiving " << size - read_count << " / " << size
                 << " bytes of data from serial port " << serial_port_ << " timed out.";
      return false;
    }
    ssize_t once_read_count;
    once_read_count = read(serial_fd_, ((unsigned char *) (&receive_data_)) + read_count, size - read_count);
    if (once_read_count == -1) {
      if (errno == EAGAIN) {
        DLOG(WARNING) << "Receiving " << size - read_count << " / " << size
                      << " bytes of data from serial port " << serial_port_ << " delayed.";
        continue;
      } else {
        LOG(ERROR) << "Failed to receive " << size - read_count << " / " << size
                   << " bytes of data from serial port " << serial_port_ << ".";
        return false;
      }
    }
    read_count += once_read_count;
  }
  tcflush(serial_fd_, TCIFLUSH);
  DLOG(INFO) << "Received " << size << " bytes of data from serial port " << serial_port_ << ".";
  DLOG(INFO) << receive_data_;
  return true;
}

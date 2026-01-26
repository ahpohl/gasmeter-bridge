#include "firmware.h"
#include "config_yaml.h"
#include "firmware_types.h"
#include "firmware_utils.h"
#include "meter_error.h"
#include "signal_handler.h"
#include <asm-generic/ioctls.h>
#include <cstdint>
#include <expected>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>

Firmware::Firmware(const MeterConfig &cfg, SignalHandler &signalHandler)
    : cfg_(cfg), handler_(signalHandler) {};

std::expected<void, MeterError> Firmware::connect(void) {
  if (!handler_.isRunning()) {
    return std::unexpected(
        MeterError::custom(EINTR, "tryConnect(): Shutdown in progress"));
  }

  if (serialPort_ >= 0)
    return {};

  serialPort_ = open(cfg_.device.c_str(), O_RDONLY | O_NOCTTY);
  if (serialPort_ == -1) {
    return std::unexpected(
        MeterError::fromErrno("Opening serial device failed"));
  }

  if (!isatty(serialPort_)) {
    int saved_errno = errno;
    close(serialPort_);
    errno = saved_errno;
    return std::unexpected(MeterError::fromErrno("Device is not a tty"));
  }

  if (flock(serialPort_, LOCK_EX | LOCK_NB) == -1) {
    int saved_errno = errno;
    close(serialPort_);
    errno = saved_errno;
    return std::unexpected(
        MeterError::fromErrno("Failed to lock serial device"));
  }

  if (ioctl(serialPort_, TIOCEXCL) == -1) {
    int saved_errno = errno;
    close(serialPort_);
    errno = saved_errno;
    return std::unexpected(
        MeterError::fromErrno("Failed to set exclusive lock"));
  }

  termios serialPortSettings;
  if (tcgetattr(serialPort_, &serialPortSettings) == -1) {
    int saved_errno = errno;
    close(serialPort_);
    errno = saved_errno;
    return std::unexpected(
        MeterError::fromErrno("Failed to get serial port attributes"));
  }

  cfmakeraw(&serialPortSettings);

  // set baud (both directions)
  speed_t baudSpeed = MeterTypes::baudToSpeed(cfg_.baud);
  if (cfsetispeed(&serialPortSettings, baudSpeed) < 0 ||
      cfsetospeed(&serialPortSettings, baudSpeed) < 0) {
    int saved_errno = errno;
    close(serialPort_);
    errno = saved_errno;
    return std::unexpected(MeterError::fromErrno(
        "Failed to set serial port speed {} baud", cfg_.baud));
  }

  // Base flags: enable receiver, ignore modem control lines
  serialPortSettings.c_cflag |= (CLOCAL | CREAD);

  // Clear size/parity/stop/flow flags first to avoid unexpected bits
  serialPortSettings.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);

  // Set data bits
  serialPortSettings.c_cflag |= MeterTypes::dataBitsToFlag(cfg_.dataBits);

  // Set parity
  switch (cfg_.parity) {
  case MeterTypes::Parity::Even:
    serialPortSettings.c_cflag |= PARENB;
    serialPortSettings.c_cflag &= ~PARODD;
    break;
  case MeterTypes::Parity::Odd:
    serialPortSettings.c_cflag |= PARENB;
    serialPortSettings.c_cflag |= PARODD;
    break;
  case MeterTypes::Parity::None:
  default:
    // PARENB already cleared above
    break;
  }

  // Set stop bits (2 stop bits if stopBits == 2, otherwise 1)
  if (cfg_.stopBits == 2) {
    serialPortSettings.c_cflag |= CSTOPB;
  }

  // Non-blocking read:  return immediately with available data (VMIN=0), 0.5s
  // timeout for first byte (VTIME=5)
  serialPortSettings.c_cc[VMIN] = BUFFER_SIZE;
  serialPortSettings.c_cc[VTIME] = 5;

  if (tcsetattr(serialPort_, TCSANOW, &serialPortSettings)) {
    int saved_errno = errno;
    close(serialPort_);
    errno = saved_errno;
    return std::unexpected(
        MeterError::fromErrno("Failed to set serial port attributes"));
  }

  // flush both directions if desired after applying settings
  tcflush(serialPort_, TCIOFLUSH);

  return {};
}

std::expected<void, MeterError> Firmware::send(FirmwareTypes::Command cmd,
                                               uint8_t b1, uint8_t b2,
                                               uint8_t b3, uint8_t b4,
                                               uint8_t b5) {

  txBuffer[0] = static_cast<uint8_t>(cmd);
  txBuffer[1] = b1;
  txBuffer[2] = b2;
  txBuffer[3] = b3;
  txBuffer[4] = b4;
  txBuffer[5] = b5;

  uint16_t crc = FirmwareUtils::crc16(txBuffer, 6);
  txBuffer[6] = FirmwareUtils::lowByte(crc);
  txBuffer[7] = FirmwareUtils::highByte(crc);

  if (writeBytes(txBuffer, SEND_BUFFER_SIZE) < 0) {
    ErrorMessage =
        std::string("Write bytes failed: ") + Serial->GetErrorMessage();
    Serial->Flush();
    return false;
  }
  if (Log) {
    std::cout << "Send: ";
    FirmwareUtils::logBuffer(txBuffer, SEND_BUFFER_SIZE);
  }

  if (readBytes(ReceiveData, RECEIVE_BUFFER_SIZE) < 0) {
    ErrorMessage =
        std::string("Read bytes failed: ") + Serial->GetErrorMessage();
    Serial->Flush();
    return false;
  }
  if (Log) {
    std::cout << "Receive: ";
    FirmwareUtils::logBuffer(rxBuffer, RECEIVE_BUFFER_SIZE);
  }
  if (!(word(ReceiveData[5], ReceiveData[6]) == crc16(ReceiveData, 5))) {
    ErrorMessage = "Received serial package with CRC mismatch";
    Serial->Flush();
    return false;
  }
  if (ReceiveData[0]) {
    ErrorMessage = std::string("Transmission error: ") +
                   TransmissionState(ReceiveData[0]) + " (" +
                   std::to_string(ReceiveData[0]) + ")";
    return false;
  }
  return {};
}
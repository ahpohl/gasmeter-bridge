#include "firmware.h"
#include "config_yaml.h"
#include "firmware_types.h"
#include "firmware_utils.h"
#include "meter_error.h"
#include "signal_handler.h"
#include <asm-generic/ioctls.h>
#include <cerrno>
#include <cstdint>
#include <expected>
#include <fcntl.h>
#include <spdlog/logger.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>

Firmware::Firmware(const MeterConfig &cfg, SignalHandler &signalHandler)
    : cfg_(cfg), handler_(signalHandler) {

  meterLogger_ = spdlog::get("meter");
  if (!meterLogger_)
    meterLogger_ = spdlog::default_logger();
};

Firmware::~Firmware(void) {
  if (serialPort_ != -1) {
    close(serialPort_);
    serialPort_ = -1;
  }
}

std::expected<void, MeterError> Firmware::connect(void) {
  if (!handler_.isRunning()) {
    return std::unexpected(
        MeterError::custom(EINTR, "connect(): Shutdown in progress"));
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

  // Disable reset after modem hangup
  serialPortSettings.c_cflag &= ~HUPCL;

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

  txBuffer_[0] = static_cast<uint8_t>(cmd);
  txBuffer_[1] = b1;
  txBuffer_[2] = b2;
  txBuffer_[3] = b3;
  txBuffer_[4] = b4;
  txBuffer_[5] = b5;

  uint16_t crc = FirmwareUtils::crc16(txBuffer_.data(), 6);
  txBuffer_[6] = FirmwareUtils::lowByte(crc);
  txBuffer_[7] = FirmwareUtils::highByte(crc);

  auto wbytes = writeBytes(txBuffer_.data(), txBuffer_.size());
  if (!wbytes)
    return std::unexpected(wbytes.error());
  meterLogger_->trace("Send: {}", FirmwareUtils::logBuffer(txBuffer_));

  auto rbytes = readBytes(rxBuffer_.data(), rxBuffer_.size());
  if (!rbytes)
    return std::unexpected(rbytes.error());
  meterLogger_->trace("Receive: {}", FirmwareUtils::logBuffer(rxBuffer_));

  if (!(FirmwareUtils::word(rxBuffer_[5], rxBuffer_[6]) ==
        FirmwareUtils::crc16(rxBuffer_.data(), 5))) {
    return std::unexpected(MeterError::custom(
        EPROTO, "Received serial package with CRC mismatch"));
  }
  if (rxBuffer_[0]) {
    return std::unexpected(MeterError::custom(
        EPROTO, "Command failed: {} (0x{:02X})",
        FirmwareTypes::statusToString(
            static_cast<FirmwareTypes::Status>(rxBuffer_[0])),
        rxBuffer_[0]));
  }

  return {};
}

std::expected<int, MeterError> Firmware::readBytes(uint8_t *buffer,
                                                   const int &length) {
  int iterations = 0;
  const int maxIterations = 500;

  while (iterations < maxIterations) {
    int bytesAvailable = 0;
    int rc = ioctl(serialPort_, FIONREAD, &bytesAvailable);
    if (rc < 0)
      return std::unexpected(MeterError::fromErrno("FIONREAD failed"));

    // intercharacter delay: 1 / baud rate * 1e6 = 17.4 µs
    int intercharacterDelay = 1.0 / cfg_.baud * 1e6;
    std::this_thread::sleep_for(std::chrono::microseconds(intercharacterDelay));
    if (bytesAvailable >= length)
      break;
    iterations++;
  }

  if (iterations == maxIterations) {
    return std::unexpected(
        MeterError::custom(ETIMEDOUT, "Timeout, firmware did not respond"));
  }

  int bytesReceived = read(serialPort_, buffer, length);
  if (bytesReceived < 0) {
    return std::unexpected(MeterError::fromErrno("Reading from device failed"));
  }

  return bytesReceived;
}

std::expected<int, MeterError> Firmware::writeBytes(uint8_t const *buffer,
                                                    const int &length) {

  int bytesSent = write(serialPort_, buffer, length);
  if (bytesSent < 0) {
    return std::unexpected(MeterError::fromErrno("Failed to write bytes"));
  }
  tcdrain(serialPort_);

  return bytesSent;
}

std::expected<float, MeterError>
Firmware::readDspValue(const FirmwareTypes::DspValue &type) {

  auto dsp = send(FirmwareTypes::Command::MeasureRequestDsp,
                  static_cast<uint8_t>(type), 0, 0, 0, 0);
  if (!dsp)
    return std::unexpected(dsp.error());

  float value = FirmwareUtils::bytesToFloat(rxBuffer_[1], rxBuffer_[2],
                                            rxBuffer_[3], rxBuffer_[4]);
  return value / 100.0f;
}

/* ----- public api ----- */

std::expected<float, MeterError> Firmware::getVolume(void) {
  auto value = readDspValue(FirmwareTypes::DspValue::GasVolume);
  if (!value)
    return std::unexpected(value.error());
  return value;
}
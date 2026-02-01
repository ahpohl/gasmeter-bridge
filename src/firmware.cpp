#include "firmware.h"
#include "config_yaml.h"
#include "firmware_types.h"
#include "firmware_utils.h"
#include "meter_error.h"
#include "signal_handler.h"
#include <asm-generic/ioctls.h>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <expected>
#include <fcntl.h>
#include <spdlog/logger.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>

Firmware::Firmware(const MeterConfig &cfg, SignalHandler &signalHandler,
                   std::shared_ptr<spdlog::logger> logger)
    : cfg_(cfg), handler_(signalHandler), firmwareLogger_(logger) {}

Firmware::~Firmware(void) { disconnect(); }

void Firmware::disconnect(void) {
  // Stop worker thread first
  if (workerRunning_) {
    workerRunning_.store(false);
    queueCV_.notify_all();

    if (serialWorker_.joinable()) {
      serialWorker_.join();
    }
  }
  // Then close serial port
  if (serialPort_ != -1) {
    close(serialPort_);
    serialPort_ = -1;
    firmwareLogger_->info("Meter disconnected");
  }
}

std::expected<void, MeterError> Firmware::connect(void) {
  if (!handler_.isRunning()) {
    return std::unexpected(
        MeterError::custom(EINTR, "connect(): Shutdown in progress"));
  }

  if (serialPort_ >= 0)
    return {};

  serialPort_ = open(cfg_.device.c_str(), O_RDWR | O_NOCTTY);
  if (serialPort_ == -1) {
    return std::unexpected(
        MeterError::fromErrno("Opening serial device failed"));
  }

  if (!isatty(serialPort_)) {
    int savedErrno = errno;
    close(serialPort_);
    errno = savedErrno;
    return std::unexpected(MeterError::fromErrno("Device is not a tty"));
  }

  if (flock(serialPort_, LOCK_EX | LOCK_NB) == -1) {
    int savedErrno = errno;
    close(serialPort_);
    errno = savedErrno;
    return std::unexpected(
        MeterError::fromErrno("Failed to lock serial device"));
  }

  if (ioctl(serialPort_, TIOCEXCL) == -1) {
    int savedErrno = errno;
    close(serialPort_);
    errno = savedErrno;
    return std::unexpected(
        MeterError::fromErrno("Failed to set exclusive lock"));
  }

  termios serialPortSettings;
  if (tcgetattr(serialPort_, &serialPortSettings) == -1) {
    int savedErrno = errno;
    close(serialPort_);
    errno = savedErrno;
    return std::unexpected(
        MeterError::fromErrno("Failed to get serial port attributes"));
  }

  cfmakeraw(&serialPortSettings);

  // set baud (both directions)
  speed_t baudSpeed = B19200;
  if (cfsetispeed(&serialPortSettings, baudSpeed) < 0 ||
      cfsetospeed(&serialPortSettings, baudSpeed) < 0) {
    int savedErrno = errno;
    close(serialPort_);
    errno = savedErrno;
    return std::unexpected(MeterError::fromErrno(
        "Failed to set serial port speed {} baud", baudSpeed));
  }

  // Base flags: enable receiver, ignore modem control lines
  serialPortSettings.c_cflag |= (CLOCAL | CREAD);

  // Set 8N1 port configuration
  serialPortSettings.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);
  serialPortSettings.c_cflag |= CS8;

  // Disable reset after modem hangup
  serialPortSettings.c_cflag &= ~HUPCL;

  // non-blocking read
  serialPortSettings.c_cc[VMIN] = 0;
  serialPortSettings.c_cc[VTIME] = 0;

  if (tcsetattr(serialPort_, TCSANOW, &serialPortSettings)) {
    int savedErrno = errno;
    close(serialPort_);
    errno = savedErrno;
    return std::unexpected(
        MeterError::fromErrno("Failed to set serial port attributes"));
  }

  // --- reset µC ---
  firmwareLogger_->debug("Resetting internal µC ...");

  int status;
  ioctl(serialPort_, TIOCMGET, &status);
  status &= ~TIOCM_DTR; // Lower DTR
  ioctl(serialPort_, TIOCMSET, &status);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  status |= TIOCM_DTR; // Raise DTR
  ioctl(serialPort_, TIOCMSET, &status);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // flush both directions after applying settings
  tcflush(serialPort_, TCIOFLUSH);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  firmwareLogger_->info("Meter connected (8N1, {} baud)", baudSpeed);

  // Start serial worker thread
  serialWorker_ = std::thread(&Firmware::serialWorkerLoop, this);
  workerRunning_.store(true);

  return {};
}

void Firmware::serialWorkerLoop() {
  firmwareLogger_->debug("Serial worker thread started");

  while (workerRunning_.load() == true && handler_.isRunning()) {
    std::shared_ptr<SerialCommand> cmd;

    // Wait for command in queue
    {
      std::unique_lock<std::mutex> lock(queueMutex_);
      queueCV_.wait(lock, [this] {
        return !commandQueue_.empty() || workerRunning_.load() == false ||
               !handler_.isRunning();
      });

      if (workerRunning_.load() == false || !handler_.isRunning()) {
        break;
      }

      if (!commandQueue_.empty()) {
        cmd = commandQueue_.front();
        commandQueue_.pop();
      }
    }

    // Process command if we have one
    if (cmd) {
      auto result = processCommand(*cmd);
      cmd->promise.set_value(std::move(result));
    }
  }

  firmwareLogger_->debug("Serial worker thread stopped");
}

std::expected<std::array<uint8_t, Firmware::RECEIVE_BUFFER_SIZE>, MeterError>
Firmware::processCommand(const SerialCommand &cmd) {

  auto replyStart = std::chrono::steady_clock::now();

  std::array<uint8_t, SEND_BUFFER_SIZE> txBuffer;
  txBuffer[0] = static_cast<uint8_t>(cmd.cmd);
  txBuffer[1] = cmd.params[0];
  txBuffer[2] = cmd.params[1];
  txBuffer[3] = cmd.params[2];
  txBuffer[4] = cmd.params[3];
  txBuffer[5] = cmd.params[4];

  uint16_t checksum = FirmwareUtils::crc16(txBuffer.data(), 6);
  txBuffer[6] = FirmwareUtils::lowByte(checksum);
  txBuffer[7] = FirmwareUtils::highByte(checksum);

  auto writeResult = writeBytes(txBuffer.data(), txBuffer.size());
  if (!writeResult)
    return std::unexpected(writeResult.error());

  firmwareLogger_->trace("Sent bytes  {}", FirmwareUtils::logBuffer(txBuffer));

  // Read response
  std::array<uint8_t, RECEIVE_BUFFER_SIZE> rxBuffer;
  rxBuffer.fill(0);

  auto readResult = readBytes(rxBuffer.data(), rxBuffer.size());
  if (!readResult)
    return std::unexpected(readResult.error());

  firmwareLogger_->trace("Received bytes {}",
                         FirmwareUtils::logBuffer(rxBuffer));

  // Validate checksum
  uint16_t receivedChecksum = FirmwareUtils::word(rxBuffer[5], rxBuffer[6]);
  uint16_t calculatedChecksum = FirmwareUtils::crc16(rxBuffer.data(), 5);

  if (receivedChecksum != calculatedChecksum) {
    return std::unexpected(MeterError::custom(
        EPROTO, "Invalid CRC checksum: received 0x{:04X}, expected 0x{:04X}",
        receivedChecksum, calculatedChecksum));
  }

  // Check status byte
  if (rxBuffer[0]) {
    return std::unexpected(
        MeterError::custom(EPROTO, "Command failed: {} (0x{:02X})",
                           FirmwareTypes::statusToString(
                               static_cast<FirmwareTypes::Status>(rxBuffer[0])),
                           rxBuffer[0]));
  }

  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - replyStart);
  firmwareLogger_->trace("Send command took {} ms", elapsed.count());

  // Return response buffer
  return rxBuffer;
}

std::expected<std::array<uint8_t, Firmware::RECEIVE_BUFFER_SIZE>, MeterError>
Firmware::sendCommand(FirmwareTypes::Command cmd, uint8_t b1, uint8_t b2,
                      uint8_t b3, uint8_t b4, uint8_t b5) {

  if (workerRunning_.load() == false) {
    return std::unexpected(
        MeterError::custom(EINVAL, "Serial worker not running"));
  }

  // Create command with promise/future pair
  auto cmdPtr = std::make_shared<SerialCommand>();
  cmdPtr->cmd = cmd;
  cmdPtr->params = {b1, b2, b3, b4, b5};

  auto future = cmdPtr->promise.get_future();

  // Enqueue command
  {
    std::lock_guard<std::mutex> lock(queueMutex_);
    commandQueue_.push(cmdPtr);
  }
  queueCV_.notify_one();

  // Wait for result (blocks until worker processes command)
  return future.get();
}

std::expected<int, MeterError> Firmware::readBytes(uint8_t *buffer,
                                                   int length) {
  if (serialPort_ < 0 || fcntl(serialPort_, F_GETFD) == -1) {
    return std::unexpected(MeterError::fromErrno(
        "readBytes(): Serial port not open or already closed"));
  }

  int totalReceived = 0;

  while (totalReceived < length) {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(serialPort_, &readfds);

    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    int ret = select(serialPort_ + 1, &readfds, NULL, NULL, &timeout);
    if (ret < 0)
      return std::unexpected(MeterError::fromErrno("select() failed"));
    if (ret == 0)
      return std::unexpected(
          MeterError::custom(ETIMEDOUT, "Timeout: expected {} bytes, got {}",
                             length, totalReceived));

    int bytesReceived =
        read(serialPort_, buffer + totalReceived, length - totalReceived);
    if (bytesReceived < 0)
      return std::unexpected(MeterError::fromErrno("read() failed"));

    totalReceived += bytesReceived;
  }

  return totalReceived;
}

std::expected<int, MeterError> Firmware::writeBytes(uint8_t const *buffer,
                                                    int length) {

  // Check validity before using
  if (serialPort_ < 0 || fcntl(serialPort_, F_GETFD) == -1) {
    return std::unexpected(MeterError::fromErrno(
        "writeBytes(): Serial port not open or already closed"));
  }

  int bytesSent = write(serialPort_, buffer, length);
  if (bytesSent < 0) {
    return std::unexpected(MeterError::fromErrno("write() failed"));
  }
  tcdrain(serialPort_);

  return bytesSent;
}

std::expected<double, MeterError>
Firmware::readDspValue(FirmwareTypes::DspValue measurement) {

  auto cmdResult = sendCommand(FirmwareTypes::Command::MeasureRequestDsp,
                               static_cast<uint8_t>(measurement), 0, 0, 0, 0);
  if (!cmdResult)
    return std::unexpected(cmdResult.error());

  // Extract response bytes from returned buffer
  const auto &rxBuffer = cmdResult.value();

  return FirmwareUtils::bytesToDouble(rxBuffer[1], rxBuffer[2], rxBuffer[3],
                                      rxBuffer[4]);
}

std::expected<double, MeterError> Firmware::getVolume(void) {
  auto vol = readDspValue(FirmwareTypes::DspValue::Volume);
  if (!vol)
    return std::unexpected(vol.error());
  return vol;
}

std::expected<void, MeterError> Firmware::setVolume(double volume) {
  std::array b = FirmwareUtils::doubleToBytes(volume);
  auto cmdResult = sendCommand(FirmwareTypes::Command::SetMeterVolume, 0, b[0],
                               b[1], b[2], b[3]);
  if (!cmdResult)
    return std::unexpected(cmdResult.error());

  return {};
}

std::expected<void, MeterError> Firmware::clearVolume(void) {
  auto cmdResult =
      sendCommand(FirmwareTypes::Command::ClearMeterVolume, 0, 0, 0, 0, 0);
  if (!cmdResult)
    return std::unexpected(cmdResult.error());

  return {};
}

std::expected<void, MeterError> Firmware::setThresholdLevels(int16_t low,
                                                             int16_t high) {
  std::array<uint8_t, 2> l{static_cast<uint8_t>(low & 0xFF),
                           static_cast<uint8_t>((low >> 8) & 0xFF)};

  std::array<uint8_t, 2> h{static_cast<uint8_t>(high & 0xFF),
                           static_cast<uint8_t>((high >> 8) & 0xFF)};

  auto cmdResult = sendCommand(FirmwareTypes::Command::ClearMeterVolume, 0,
                               l[0], l[1], h[0], h[1]);
  if (!cmdResult)
    return std::unexpected(cmdResult.error());

  return {};
}

std::expected<int, MeterError> Firmware::getRawIR(void) {
  auto cmdResult = sendCommand(
      FirmwareTypes::Command::MeasureRequestDsp,
      static_cast<uint8_t>(FirmwareTypes::DspValue::RawIr), 0, 0, 0, 0);
  if (!cmdResult)
    return std::unexpected(cmdResult.error());

  // Extract response bytes from returned buffer
  const auto &rxBuffer = cmdResult.value();

  return FirmwareUtils::bytesToInt(rxBuffer[1], rxBuffer[2], rxBuffer[3],
                                   rxBuffer[4]);
}

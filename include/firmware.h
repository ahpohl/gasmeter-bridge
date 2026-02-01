#ifndef FIRMWARE_H
#define FIRMWARE_H

#include "config_yaml.h"
#include "firmware_types.h"
#include "meter_error.h"
#include "signal_handler.h"
#include <array>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <mutex>
#include <queue>
#include <spdlog/logger.h>

class Firmware {
public:
  explicit Firmware(const MeterConfig &cfg, SignalHandler &signalHandler,
                    std::shared_ptr<spdlog::logger> logger);
  virtual ~Firmware();

  std::expected<void, MeterError> connect(void);
  void disconnect(void);
  std::expected<double, MeterError> getVolume(void);
  std::expected<void, MeterError> setVolume(double volume);
  std::expected<void, MeterError> clearVolume(void);
  std::expected<void, MeterError> setThresholdLevels(int16_t low, int16_t high);
  std::expected<int, MeterError> getRawIR(void);
  bool isConnected(void) { return workerRunning_.load(); };

  static constexpr int SEND_BUFFER_SIZE = 8;
  static constexpr int RECEIVE_BUFFER_SIZE = 7;

private:
  // Command structure for queue
  struct SerialCommand {
    FirmwareTypes::Command cmd;
    std::array<uint8_t, 5> params;
    std::promise<
        std::expected<std::array<uint8_t, RECEIVE_BUFFER_SIZE>, MeterError>>
        promise;
  };

  // Configuration and state
  int serialPort_{-1};
  const MeterConfig &cfg_;
  SignalHandler &handler_;
  std::shared_ptr<spdlog::logger> firmwareLogger_;

  // Serial worker thread
  std::thread serialWorker_;
  std::atomic<bool> workerRunning_{false};

  // Command queue (thread-safe)
  std::queue<std::shared_ptr<SerialCommand>> commandQueue_;
  std::mutex queueMutex_;
  std::condition_variable queueCV_;

  // Worker thread methods
  void serialWorkerLoop();
  std::expected<std::array<uint8_t, RECEIVE_BUFFER_SIZE>, MeterError>
  processCommand(const SerialCommand &cmd);

  // Low-level serial I/O (called only from worker thread)
  std::expected<int, MeterError> writeBytes(const uint8_t *buffer, int length);
  std::expected<int, MeterError> readBytes(uint8_t *buffer, int length);

  // Command submission (thread-safe)
  std::expected<std::array<uint8_t, RECEIVE_BUFFER_SIZE>, MeterError>
  sendCommand(FirmwareTypes::Command cmd, uint8_t b1, uint8_t b2, uint8_t b3,
              uint8_t b4, uint8_t b5);
};

#endif /* FIRMWARE_H */
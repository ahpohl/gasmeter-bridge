#ifndef FIRMWARE_H
#define FIRMWARE_H

#include "config_yaml.h"
#include "firmware_types.h"
#include "meter_error.h"
#include "signal_handler.h"
#include <array>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <spdlog/logger.h>

class Firmware {
public:
  explicit Firmware(const MeterConfig &cfg, SignalHandler &signalHandler,
                    std::shared_ptr<spdlog::logger> logger);
  virtual ~Firmware();

  std::expected<void, MeterError> connect(void);
  void disconnect(void);
  std::expected<void, MeterError> setThresholdLevels(const int &low,
                                                     const int &high);
  std::expected<void, MeterError> setVolume(const float &volume);
  std::expected<float, MeterError> getVolume(void);
  std::expected<void, MeterError> clearVolume(void);

  static constexpr int SEND_BUFFER_SIZE = 8;
  static constexpr int RECEIVE_BUFFER_SIZE = 7;

private:
  int serialPort_{-1};
  const MeterConfig &cfg_;
  SignalHandler &handler_;
  std::shared_ptr<spdlog::logger> logger_;
  std::array<uint8_t, SEND_BUFFER_SIZE> txBuffer_;
  std::array<uint8_t, RECEIVE_BUFFER_SIZE> rxBuffer_;
  mutable std::mutex mtx_;
  std::condition_variable cv_;

  std::expected<void, MeterError> sendCommand(FirmwareTypes::Command cmd,
                                              uint8_t b1, uint8_t b2,
                                              uint8_t b3, uint8_t b4,
                                              uint8_t b5);

  std::expected<int, MeterError> writeBytes(uint8_t const *buffer,
                                            const int &length);
  std::expected<int, MeterError> readBytes(uint8_t *buffer, const int &length);

  std::expected<float, MeterError>
  readDspValue(const FirmwareTypes::DspValue &measurement);
};

#endif /* FIRMWARE_H */
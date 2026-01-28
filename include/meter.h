#ifndef METER_H_
#define METER_H_

#include "config_yaml.h"
#include "firmware.h"
#include "meter_error.h"
#include "meter_types.h"
#include "signal_handler.h"
#include <condition_variable>
#include <expected>
#include <functional>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <spdlog/logger.h>
#include <string>
#include <thread>

class Meter {
public:
  explicit Meter(const MeterConfig &cfg, SignalHandler &signalHandler);
  virtual ~Meter();

  std::string getJsonDump(void) const;
  MeterTypes::Values getValues(void) const;
  void
  setUpdateCallback(std::function<void(std::string, MeterTypes::Values)> cb);
  void
  setDeviceCallback(std::function<void(std::string, MeterTypes::Device)> cb);
  void setAvailabilityCallback(std::function<void(std::string)> cb);

  static constexpr size_t BUFFER_SIZE = 64;
  static constexpr size_t TELEGRAM_SIZE = 368;

private:
  void runLoop();
  MeterTypes::ErrorAction
  handleResult(std::expected<void, MeterError> &&result);
  void disconnect(void);
  std::expected<void, MeterError> updateValuesAndJson(void);
  std::expected<void, MeterError> updateDeviceAndJson(void);

  const MeterConfig &cfg_;
  MeterTypes::Values values_;
  MeterTypes::Device device_;
  nlohmann::ordered_json jsonValues_;
  nlohmann::json jsonDevice_;
  Firmware firmware_;
  std::optional<float> previousVolume_;
  bool clearVolume_{false};

  // --- threading / callbacks ---
  std::function<void(std::string, MeterTypes::Values)> updateCallback_;
  std::function<void(std::string, MeterTypes::Device)> deviceCallback_;
  std::function<void(std::string)> availabilityCallback_;
  SignalHandler &handler_;
  mutable std::mutex cbMutex_;
  std::condition_variable cv_;
  std::thread worker_;

  // --- logger ---
  std::shared_ptr<spdlog::logger> meterLogger_;
  static std::shared_ptr<spdlog::logger> getLogger() {
    auto logger = spdlog::get("meter");
    return logger ? logger : spdlog::default_logger();
  }
};

#endif /* METER_H_ */

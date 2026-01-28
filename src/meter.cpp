#include "meter.h"
#include "config.h"
#include "config_yaml.h"
#include "json_utils.h"
#include "meter_error.h"
#include "meter_types.h"
#include "signal_handler.h"
#include <asm-generic/ioctls.h>
#include <chrono>
#include <expected>
#include <nlohmann/json.hpp>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>

using json = nlohmann::ordered_json;

Meter::Meter(const MeterConfig &cfg, SignalHandler &signalHandler)
    : cfg_(cfg), handler_(signalHandler), meterLogger_(getLogger()),
      firmware_(cfg, signalHandler, getLogger()) {

  clearVolume_ = cfg_.gas.reset;

  // Start update loop thread
  worker_ = std::thread(&Meter::runLoop, this);
}

Meter::~Meter() {
  cv_.notify_all();
  if (worker_.joinable())
    worker_.join();
  disconnect();
}

void Meter::disconnect(void) {
  firmware_.disconnect();

  if (availabilityCallback_)
    availabilityCallback_("disconnected");
}

void Meter::setUpdateCallback(
    std::function<void(std::string, MeterTypes::Values)> cb) {
  std::lock_guard<std::mutex> lock(cbMutex_);
  updateCallback_ = std::move(cb);
}

void Meter::setDeviceCallback(
    std::function<void(std::string, MeterTypes::Device)> cb) {
  std::lock_guard<std::mutex> lock(cbMutex_);
  deviceCallback_ = std::move(cb);
}

void Meter::setAvailabilityCallback(std::function<void(std::string)> cb) {
  std::lock_guard<std::mutex> lock(cbMutex_);
  availabilityCallback_ = std::move(cb);
}

MeterTypes::ErrorAction
Meter::handleResult(std::expected<void, MeterError> &&result) {
  if (result) {
    return MeterTypes::ErrorAction::NONE;
  }

  const MeterError &err = result.error();

  if (err.severity == MeterError::Severity::FATAL) {
    // Fatal error occurred - initiate shutdown sequence
    meterLogger_->error("FATAL Meter error: {}", err.describe());
    handler_.shutdown();
    return MeterTypes::ErrorAction::SHUTDOWN;

  } else if (err.severity == MeterError::Severity::TRANSIENT) {
    // Temporary error - disconnect, wait and reconnect
    meterLogger_->warn("Transient Meter error: {}", err.describe());
    disconnect();
    return MeterTypes::ErrorAction::RECONNECT;

  } else if (err.severity == MeterError::Severity::SHUTDOWN) {
    // Shutdown already in progress - just exit cleanly
    meterLogger_->trace("Meter operation cancelled due to shutdown: {}",
                        err.describe());
    return MeterTypes::ErrorAction::SHUTDOWN;
  }

  return MeterTypes::ErrorAction::NONE;
}

std::expected<void, MeterError> Meter::updateValuesAndJson() {
  if (!handler_.isRunning()) {
    return std::unexpected(MeterError::custom(
        EINTR, "updateValuesAndJson(): Shutdown in progress"));
  }

  MeterTypes::Values values{};

  values.time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();

  if (clearVolume_) {
    auto clearResult = firmware_.clearVolume();
    if (!clearResult)
      return std::unexpected(clearResult.error());
    clearVolume_ = false;
  }

  try {
    values.volume = MeterError::getOrThrow(firmware_.getVolume());
  } catch (const MeterError &err) {
    return std::unexpected(err);
  }

  if (values.volume < cfg_.gas.initial) {
    auto setResult = firmware_.setVolume(cfg_.gas.initial);
    if (!setResult)
      return std::unexpected(setResult.error());
    values.volume = cfg_.gas.initial;
  }

  if (previousVolume_.has_value())
    values.flow = (values.volume > previousVolume_.value());
  previousVolume_ = values.volume;

  json newJson;
  json phases = json::array();

  newJson["time"] = values.time;
  newJson["volume"] = JsonUtils::roundTo(values.volume, 2);
  newJson["flow"] = values.flow;

  // Update shared values and JSON with lock
  {
    std::lock_guard<std::mutex> lock(cbMutex_);
    values_ = std::move(values);
    jsonValues_ = std::move(newJson);
  }

  meterLogger_->debug("{}", jsonValues_.dump());

  return {};
}

std::expected<void, MeterError> Meter::updateDeviceAndJson() {
  if (!handler_.isRunning()) {
    return std::unexpected(MeterError::custom(
        EINTR, "updateDeviceAndJson(): Shutdown in progress"));
  }

  MeterTypes::Device newDevice{};

  newDevice.manufacturer = "Pipersberg";
  newDevice.model = "G4 RF1c";
  newDevice.bridgeVersion =
      std::string(PROJECT_VERSION) + "-" + GIT_COMMIT_HASH;
  newDevice.serialNumber = "42010646";

  // ---- Build ordered JSON ----
  json newJson;

  newJson["manufacturer"] = newDevice.manufacturer;
  newJson["model"] = newDevice.model;
  newJson["serial_number"] = newDevice.serialNumber;
  newJson["bridge_version"] = newDevice.bridgeVersion;

  meterLogger_->debug("{}", newJson.dump());

  // ---- Commit values ----
  {
    std::lock_guard<std::mutex> lock(cbMutex_);
    jsonDevice_ = std::move(newJson);
    device_ = std::move(newDevice);
  }

  return {};
}

void Meter::runLoop() {

  while (handler_.isRunning()) {

    // Connect to meter
    auto connectAction = handleResult(firmware_.connect());
    if (connectAction == MeterTypes::ErrorAction::SHUTDOWN)
      break;

    if (availabilityCallback_)
      availabilityCallback_("connected");

    // Update device
    auto deviceAction = handleResult(updateDeviceAndJson());
    if (deviceAction == MeterTypes::ErrorAction::SHUTDOWN)
      break;
    else if (deviceAction == MeterTypes::ErrorAction::RECONNECT)
      continue;

    if (handler_.isRunning()) {
      std::lock_guard<std::mutex> lock(cbMutex_);
      if (deviceCallback_) {
        deviceCallback_(jsonDevice_.dump(), device_);
      }
    }

    // Update values
    auto updateAction = handleResult(updateValuesAndJson());
    if (updateAction == MeterTypes::ErrorAction::SHUTDOWN)
      break;
    else if (updateAction == MeterTypes::ErrorAction::RECONNECT)
      continue;

    if (handler_.isRunning()) {
      std::lock_guard<std::mutex> lock(cbMutex_);
      if (updateCallback_) {
        updateCallback_(jsonValues_.dump(), values_);
      }
    }

    // --- Wait for next update interval ---
    std::unique_lock<std::mutex> lock(cbMutex_);
    cv_.wait_for(lock, std::chrono::seconds(cfg_.updateInterval),
                 [this] { return !handler_.isRunning(); });
  }

  meterLogger_->debug("Meter run loop stopped.");
}

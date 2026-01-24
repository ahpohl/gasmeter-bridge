#include "config_yaml.h"
#include "meter_types.h"
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

static std::optional<ReconnectDelayConfig>
parseReconnectDelay(const YAML::Node &node) {
  if (!node)
    return std::nullopt;

  ReconnectDelayConfig cfg;
  cfg.min = node["min"].as<int>(5);
  cfg.max = node["max"].as<int>(365);
  cfg.exponential = node["exponential"].as<bool>(true);

  if (cfg.min <= 0)
    throw std::invalid_argument("reconnect_delay.min must be positive");
  if (cfg.max <= 0)
    throw std::invalid_argument("reconnect_delay.max must be positive");
  if (cfg.min >= cfg.max)
    throw std::invalid_argument("reconnect_delay.min must be smaller than max");

  return cfg;
}

static LevelConfig parseLevel(const YAML::Node &node) {
  if (!node)
    throw std::runtime_error("Missing 'meter.level' section in config");

  LevelConfig cfg;

  if (!node["low"])
    throw std::runtime_error("Missing required field: meter.level.low");
  if (!node["high"])
    throw std::runtime_error("Missing required field: meter.level.high");

  cfg.low = node["low"].as<int>();
  cfg.high = node["high"].as<int>();

  // Optional calibration log
  if (node["calibration_log"])
    cfg.calibrationLog = node["calibration_log"].as<std::string>();

  // Validate
  if (cfg.low < 0)
    throw std::invalid_argument("meter.level.low must be non-negative");
  if (cfg.high < 0)
    throw std::invalid_argument("meter.level.high must be non-negative");
  if (cfg.low >= cfg.high)
    throw std::invalid_argument("meter.level.low must be less than high");

  return cfg;
}

static GasConfig parseGas(const YAML::Node &node) {
  if (!node)
    throw std::runtime_error("Missing 'meter.gas' section in config");

  GasConfig cfg;

  if (!node["initial"])
    throw std::runtime_error("Missing required field: meter.gas.initial");

  cfg.initial = node["initial"].as<double>();
  cfg.reset = node["reset"].as<bool>(false);

  // Validate
  if (cfg.initial < 0.0)
    throw std::invalid_argument("meter.gas.initial must be non-negative");

  return cfg;
}

static MeterConfig parseMeter(const YAML::Node &node) {
  if (!node)
    throw std::runtime_error("Missing 'meter' section in config");

  MeterConfig cfg;
  cfg.device = node["device"].as<std::string>("/dev/ttyUSB0");

  // Start with defaults
  cfg.baud = 9600;
  cfg.dataBits = 8;
  cfg.stopBits = 1;
  cfg.parity = MeterTypes::Parity::None;

  // Apply preset if specified
  if (node["preset"]) {
    auto preset = MeterTypes::parsePreset(node["preset"].as<std::string>());
    auto defaults = MeterTypes::getPresetDefaults(preset.value());
    cfg.baud = defaults.baud;
    cfg.dataBits = defaults.dataBits;
    cfg.stopBits = defaults.stopBits;
    cfg.parity = defaults.parity;
  }

  // Apply manual overrides
  if (node["baud"])
    cfg.baud = node["baud"].as<int>();
  if (node["data_bits"])
    cfg.dataBits = node["data_bits"].as<int>();
  if (node["stop_bits"])
    cfg.stopBits = node["stop_bits"].as<int>();
  if (node["parity"])
    cfg.parity = MeterTypes::parseParity(node["parity"].as<std::string>());

  // Parse nested level and gas sections
  cfg.level = parseLevel(node["level"]);
  cfg.gas = parseGas(node["gas"]);

  // Validate
  if (cfg.baud <= 0)
    throw std::invalid_argument("meter.baud must be positive");
  if (cfg.dataBits < 5 || cfg.dataBits > 8)
    throw std::invalid_argument("meter.data_bits must be between 5 and 8");
  if (!(cfg.stopBits == 1 || cfg.stopBits == 2))
    throw std::invalid_argument("meter.stop_bits must be 1 or 2");

  return cfg;
}

static MqttConfig parseMqtt(const YAML::Node &node) {
  if (!node)
    throw std::runtime_error("Missing 'mqtt' section in config");

  if (!node["topic"])
    throw std::runtime_error("Missing required field: mqtt.topic");

  MqttConfig cfg;

  // --- Basic parameters ---
  cfg.broker = node["broker"].as<std::string>("localhost");
  cfg.port = node["port"].as<int>(1883);
  cfg.topic = node["topic"].as<std::string>();
  cfg.queueSize = node["queue_size"].as<size_t>(1000);

  // --- Optional credentials ---
  if (node["user"])
    cfg.user = node["user"].as<std::string>();
  if (node["password"])
    cfg.password = node["password"].as<std::string>();

  // --- Optional reconnect delay ---
  if (node["reconnect_delay"])
    cfg.reconnectDelay = parseReconnectDelay(node["reconnect_delay"]);

  // --- Validation ---
  if (cfg.port <= 0 || cfg.port > 65535)
    throw std::invalid_argument("mqtt.port must be in range 1–65535");

  if (cfg.queueSize == 0)
    throw std::invalid_argument("mqtt.queue_size must be greater than zero");

  return cfg;
}

static spdlog::level::level_enum parseLogLevel(const std::string &s) {
  if (s == "off")
    return spdlog::level::off;
  if (s == "error")
    return spdlog::level::err;
  if (s == "warn")
    return spdlog::level::warn;
  if (s == "info")
    return spdlog::level::info;
  if (s == "debug")
    return spdlog::level::debug;
  if (s == "trace")
    return spdlog::level::trace;
  return spdlog::level::info;
}

static LoggerConfig parseLogger(const YAML::Node &node) {
  LoggerConfig cfg;
  if (!node)
    return cfg;

  if (node["level"])
    cfg.globalLevel = parseLogLevel(node["level"].as<std::string>());

  if (node["modules"]) {
    for (auto it : node["modules"]) {
      std::string module = it.first.as<std::string>();
      cfg.moduleLevels[module] = parseLogLevel(it.second.as<std::string>());
    }
  }
  return cfg;
}

Config loadConfig(const std::string &path) {
  YAML::Node root = YAML::LoadFile(path);
  Config cfg;

  cfg.mqtt = parseMqtt(root["mqtt"]);
  cfg.logger = parseLogger(root["logger"]);
  cfg.meter = parseMeter(root["meter"]);

  return cfg;
}
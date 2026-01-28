#ifndef METER_TYPES_H_
#define METER_TYPES_H_

#include <cstdint>
#include <string>
#include <termios.h>

struct MeterTypes {

  // --- Meter value types ---
  struct Values {
    uint64_t time{0};
    double volume{0};
    bool flow{false};
  };

  struct Device {
    std::string manufacturer;
    std::string model;
    std::string serialNumber;
    std::string firmwareVersion;
    std::string bridgeVersion;
  };

  enum class ErrorAction { NONE, RECONNECT, SHUTDOWN };
};

#endif /* METER_TYPES_H_ */
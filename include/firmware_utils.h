#ifndef FIRMWARE_UTILS_H
#define FIRMWARE_UTILS_H

#include <array>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace FirmwareUtils {

inline uint16_t crc16(const uint8_t *packet, size_t length) {
  // crc16 polynomial, 1021H bit reversed
  const uint16_t POLY = 0x8408;
  uint16_t crc = 0xffff;
  if (!length) {
    return (~crc);
  }
  uint16_t data;
  uint8_t i;

  do {
    for (i = 0, data = 0xff & *packet++; i < 8; i++, data >>= 1) {
      if ((crc & 0x0001) ^ (data & 0x0001)) {
        crc = (crc >> 1) ^ POLY;
      } else {
        crc >>= 1;
      }
    }
  } while (--length);
  crc = ~crc;

  return crc;
}

inline uint8_t lowByte(const uint16_t &bytes) {
  return static_cast<uint8_t>(bytes);
}

inline uint8_t highByte(const uint16_t &bytes) {
  return static_cast<uint8_t>((bytes >> 8) & 0xFF);
}

inline uint16_t word(const uint8_t &msb, const uint8_t &lsb) {
  return ((msb & 0xFF) << 8) | lsb;
}

template <std::size_t N>
inline std::string logBuffer(const std::array<uint8_t, N> &buffer) {
  std::ostringstream oss;
  for (const auto &byte : buffer) {
    oss << std::uppercase << std::hex << std::setfill('0') << std::setw(2)
        << (static_cast<int>(byte) & 0xFF) << " ";
  }
  return oss.str();
}

inline float bytesToFloat(const uint8_t &b1, const uint8_t &b2,
                          const uint8_t &b3, const uint8_t &b4) {
  // Assumes little-endian byte order
  int32_t valueInt =
      static_cast<int32_t>(b1) | (static_cast<int32_t>(b2) << 8) |
      (static_cast<int32_t>(b3) << 16) | (static_cast<int32_t>(b4) << 24);
  return static_cast<float>(valueInt);
}

} // namespace FirmwareUtils

#endif /* FIRMWARE_UTILS_H */
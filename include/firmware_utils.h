#ifndef FIRMWARE_UTILS_H
#define FIRMWARE_UTILS_H

#include <cstdint>
#include <iomanip>
#include <iostream>

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

inline void logBuffer(const uint8_t *const buffer, const int size) {
  for (int i = 0; i < size; ++i) {
    std::cout << std::uppercase << std::hex << std::setfill('0') << std::setw(2)
              << (((int)buffer[i]) & 0xFF) << " ";
  }
  std::cout << std::endl;
}

} // namespace FirmwareUtils

#endif /* FIRMWARE_UTILS_H */
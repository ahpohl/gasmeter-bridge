#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "meter_error.h"
#include <cstdint>
#include <string>

class Protocol {
  static constexpr int SEND_BUFFER_SIZE = 8;
  static constexpr int RECEIVE_BUFFER_SIZE = 7;

public:
  explicit Protocol();
  virtual ~Protocol();

  std::expected<void, MeterError> setMeterVolume(const float &volume);
  std::expected<void, MeterError> clearMeterVolume(void);
  std::expected<void, MeterError> setThresholdLevels(const short int &low,
                                                     const short int &high);

  enum class DspValue : unsigned char { GasVolume = 1, RawIr = 2 };
  std::expected<void, MeterError> readDspValue(float &value,
                                               const DspValue &type);

private:
  enum class Status : unsigned char {
    OK = 0x00,
    UartNoData = 0x01,
    UartBufferOverflow = 0x02,
    UartParityError = 0x04,
    UartOverrunError = 0x08,
    UartFrameError = 0x10,
    UartCrcError = 0x20,
    CommandNotImplemented = 0x40,
    VariableNotExist = 0x80
  };

  enum class Command : unsigned char {
    ClearMeterVolume = 1,
    SetMeterVolume = 2,
    SetThresholds = 3,
    MeasureRequestDsp = 4
  };

  // Helper functions
  inline std::string statusToString(Status status) {
    switch (status) {
    case Status::OK:
      return "Everything is OK";
    case Status::UartNoData:
      return "UART no data";
    case Status::UartBufferOverflow:
      return "UART buffer overflow";
    case Status::UartParityError:
      return "UART parity error";
    case Status::UartOverrunError:
      return "UART overrun error";
    case Status::UartFrameError:
      return "UART frame error";
    case Status::UartCrcError:
      return "UART CRC error";
    case Status::CommandNotImplemented:
      return "Command is not implemented";
    case Status::VariableNotExist:
      return "Variable does not exist";
    default:
      return "Unknown";
    }
  }

  std::expected<void, MeterError> send(Command cmd, uint8_t b1, uint8_t b2,
                                       uint8_t b3, uint8_t b4, uint8_t b5);

  int writeBytes(int fd, uint8_t const *buffer, const int &length);
  int readBytes(int fd, uint8_t *buffer, const int &length);
};

#endif /* PROTOCOL_H */
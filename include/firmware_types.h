#ifndef FIRMWARE_TYPES_H_
#define FIRMWARE_TYPES_H_

#include <string>

struct FirmwareTypes {
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

  enum class DspValue : unsigned char { GasVolume = 1, RawIr = 2 };

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
};

#endif /* FIRMWARE_TYPES_H_ */
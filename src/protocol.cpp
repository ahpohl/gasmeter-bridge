#include "protocol.h"
#include "meter_error.h"
#include "meter_utils.h"
#include <cstdint>
#include <expected>

std::expected<void, MeterError> Protocol::send(Protocol::Command cmd,
                                               uint8_t b1, uint8_t b2,
                                               uint8_t b3, uint8_t b4,
                                               uint8_t b5) {
  uint8_t txBuffer[SEND_BUFFER_SIZE] = {0};

  txBuffer[0] = static_cast<uint8_t>(cmd);
  txBuffer[1] = b1;
  txBuffer[2] = b2;
  txBuffer[3] = b3;
  txBuffer[4] = b4;
  txBuffer[5] = b5;

  uint16_t crc = MeterUtils::crc16(txBuffer, 6);
  txBuffer[6] = MeterUtils::lowByte(crc);
  txBuffer[7] = MeterUtils::highByte(crc);

  if (writeBytes(txBuffer, SEND_BUFFER_SIZE) < 0) {
    ErrorMessage =
        std::string("Write bytes failed: ") + Serial->GetErrorMessage();
    Serial->Flush();
    return false;
  }
  if (Log) {
    std::cout << "Send: ";
    logBuffer(txBuffer, SEND_BUFFER_SIZE);
  }

  memset(rxBuffer, '\0', RECEIVE_BUFFER_SIZE);

  if (readBytes(ReceiveData, RECEIVE_BUFFER_SIZE) < 0) {
    ErrorMessage =
        std::string("Read bytes failed: ") + Serial->GetErrorMessage();
    Serial->Flush();
    return false;
  }
  if (Log) {
    std::cout << "Receive: ";
    logBuffer(ReceiveData, RECEIVE_BUFFER_SIZE);
  }
  if (!(word(ReceiveData[5], ReceiveData[6]) == crc16(ReceiveData, 5))) {
    ErrorMessage = "Received serial package with CRC mismatch";
    Serial->Flush();
    return false;
  }
  if (ReceiveData[0]) {
    ErrorMessage = std::string("Transmission error: ") +
                   TransmissionState(ReceiveData[0]) + " (" +
                   std::to_string(ReceiveData[0]) + ")";
    return false;
  }
  return {};
}
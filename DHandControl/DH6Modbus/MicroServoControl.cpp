/*
  MicroServoControl.cpp - PRJ-01-TS-U-007 V2.0.4 implementation
*/

#include "MicroServoControl.h"

namespace {
constexpr uint8_t REQUEST_HEADER_0 = 0x55;
constexpr uint8_t REQUEST_HEADER_1 = 0xAA;
constexpr uint8_t RESPONSE_HEADER_0 = 0xAA;
constexpr uint8_t RESPONSE_HEADER_1 = 0x55;
constexpr uint8_t STATUS_DATA_LENGTH = 0x0F;
constexpr size_t STATUS_FRAME_LENGTH = 20;
constexpr uint32_t MIN_COMMAND_INTERVAL_US = 1000;
}

MicroServoController::MicroServoController(HardwareSerial &serial, uint32_t baud)
    : _serial(&serial),
      _baudRate(baud),
      _lastCommandMicros(0),
      _lastResult(MICRO_SERVO_OK) {}

void MicroServoController::InitServo(int8_t rxPin, int8_t txPin) {
  _serial->begin(_baudRate, SERIAL_8N1, rxPin, txPin);
  delay(100);
}

MicroServoResult MicroServoController::lastResult() const {
  return _lastResult;
}

uint8_t MicroServoController::calculateChecksum(const uint8_t *data,
                                                size_t firstIndex,
                                                size_t lastIndex) const {
  uint8_t checksum = 0;
  for (size_t i = firstIndex; i <= lastIndex; ++i) {
    checksum = (uint8_t)(checksum + data[i]);
  }
  return checksum;
}

void MicroServoController::clearInput() {
  while (_serial->available() > 0) {
    _serial->read();
  }
}

void MicroServoController::waitForCommandInterval() {
  if (_lastCommandMicros == 0) {
    return;
  }

  uint32_t elapsed = micros() - _lastCommandMicros;
  if (elapsed < MIN_COMMAND_INTERVAL_US) {
    delayMicroseconds(MIN_COMMAND_INTERVAL_US - elapsed);
  }
}

bool MicroServoController::readResponseFrame(uint8_t *frame, size_t capacity,
                                             size_t *frameLength,
                                             uint16_t timeoutMs) {
  if (frame == nullptr || frameLength == nullptr || capacity < 6) {
    _lastResult = MICRO_SERVO_INVALID_ARGUMENT;
    return false;
  }

  size_t index = 0;
  size_t expectedLength = 0;
  uint32_t startTime = millis();

  while ((millis() - startTime) < timeoutMs) {
    while (_serial->available() > 0) {
      uint8_t value = (uint8_t)_serial->read();

      if (index == 0) {
        if (value == RESPONSE_HEADER_0) {
          frame[index++] = value;
        }
        continue;
      }

      if (index == 1) {
        if (value == RESPONSE_HEADER_1) {
          frame[index++] = value;
        } else if (value == RESPONSE_HEADER_0) {
          frame[0] = value;
          index = 1;
        } else {
          index = 0;
        }
        continue;
      }

      if (index >= capacity) {
        _lastResult = MICRO_SERVO_FORMAT_ERROR;
        return false;
      }

      frame[index++] = value;
      if (index == 3) {
        // Total bytes = two-byte header + length byte + ID + data field + checksum.
        expectedLength = (size_t)frame[2] + 5;
        if (expectedLength < 6 || expectedLength > capacity) {
          _lastResult = MICRO_SERVO_FORMAT_ERROR;
          return false;
        }
      }

      if (expectedLength != 0 && index >= expectedLength) {
        *frameLength = expectedLength;
        return true;
      }
    }
  }

  _lastResult = MICRO_SERVO_TIMEOUT;
  return false;
}

bool MicroServoController::validateCommonResponse(const uint8_t *frame,
                                                  size_t frameLength,
                                                  uint8_t expectedId,
                                                  uint8_t expectedCommand,
                                                  uint16_t expectedRegister) {
  if (frame == nullptr || frameLength < 8 ||
      frame[0] != RESPONSE_HEADER_0 || frame[1] != RESPONSE_HEADER_1 ||
      frameLength != (size_t)frame[2] + 5 || frame[3] != expectedId ||
      frame[4] != expectedCommand ||
      frame[5] != GET_LOW_BYTE(expectedRegister) ||
      frame[6] != GET_HIGH_BYTE(expectedRegister)) {
    _lastResult = MICRO_SERVO_FORMAT_ERROR;
    return false;
  }

  uint8_t expectedChecksum = calculateChecksum(frame, 2, frameLength - 2);
  if (frame[frameLength - 1] != expectedChecksum) {
    _lastResult = MICRO_SERVO_CHECKSUM_ERROR;
    return false;
  }

  return true;
}

bool MicroServoController::parseStatusResponse(const uint8_t *frame,
                                               size_t frameLength,
                                               uint8_t expectedId,
                                               uint8_t expectedCommand,
                                               uint16_t expectedRegister,
                                               MicroServoStatus *status) {
  if (frameLength != STATUS_FRAME_LENGTH || frame[2] != STATUS_DATA_LENGTH ||
      !validateCommonResponse(frame, frameLength, expectedId, expectedCommand,
                              expectedRegister)) {
    if (_lastResult == MICRO_SERVO_OK) {
      _lastResult = MICRO_SERVO_FORMAT_ERROR;
    }
    return false;
  }

  if (status != nullptr) {
    status->actuatorId = frame[3];
    status->targetPosition =
        (int16_t)((uint16_t)frame[7] | ((uint16_t)frame[8] << 8));
    status->currentPosition =
        (int16_t)((uint16_t)frame[9] | ((uint16_t)frame[10] << 8));
    status->currentMa =
        (uint16_t)frame[11] | ((uint16_t)frame[12] << 8);
    status->forceG =
        (int16_t)((uint16_t)frame[13] | ((uint16_t)frame[14] << 8));
    status->forceRaw =
        (uint16_t)frame[15] | ((uint16_t)frame[16] << 8);
    status->temperatureC = (int8_t)frame[17];
    status->errorFlags = frame[18];
  }

  _lastResult = MICRO_SERVO_OK;
  return true;
}

bool MicroServoController::readStatus(uint8_t id, MicroServoStatus *status,
                                      uint16_t timeoutMs) {
  _lastResult = MICRO_SERVO_OK;
  if (id == 0 || id == 0xFF || status == nullptr) {
    _lastResult = MICRO_SERVO_INVALID_ARGUMENT;
    return false;
  }

  // Section 3.5.1 byte example: 55 AA 01 ID 30 CheckSum.
  uint8_t request[6] = {
      REQUEST_HEADER_0, REQUEST_HEADER_1, 0x01, id, CMD_RD_STATUS, 0x00};
  request[5] = calculateChecksum(request, 2, 4);

  uint8_t response[STATUS_FRAME_LENGTH] = {0};
  size_t responseLength = 0;

  clearInput();
  waitForCommandInterval();
  _lastCommandMicros = micros();
  _serial->write(request, sizeof(request));
  _serial->flush();

  if (!readResponseFrame(response, sizeof(response), &responseLength, timeoutMs)) {
    return false;
  }

  return parseStatusResponse(response, responseLength, id, CMD_RD_STATUS, 0,
                             status);
}

bool MicroServoController::readRegisters(uint8_t id, uint16_t registerAddress,
                                         uint8_t registerCount, uint16_t *values,
                                         uint16_t timeoutMs) {
  _lastResult = MICRO_SERVO_OK;
  if (id == 0 || id == 0xFF || values == nullptr || registerCount == 0 ||
      registerCount > MICRO_SERVO_MAX_REGISTERS) {
    _lastResult = MICRO_SERVO_INVALID_ARGUMENT;
    return false;
  }

  uint8_t request[9] = {
      REQUEST_HEADER_0, REQUEST_HEADER_1, 0x04, id, CMD_RD_REGISTER,
      GET_LOW_BYTE(registerAddress), GET_HIGH_BYTE(registerAddress),
      registerCount, 0x00};
  request[8] = calculateChecksum(request, 2, 7);

  uint8_t response[8 + MICRO_SERVO_MAX_REGISTERS * 2] = {0};
  size_t responseLength = 0;

  clearInput();
  waitForCommandInterval();
  _lastCommandMicros = micros();
  _serial->write(request, sizeof(request));
  _serial->flush();

  if (!readResponseFrame(response, sizeof(response), &responseLength, timeoutMs)) {
    return false;
  }

  size_t expectedLength = 8 + (size_t)registerCount * 2;
  uint8_t expectedDataLength = (uint8_t)(3 + registerCount * 2);
  if (responseLength != expectedLength || response[2] != expectedDataLength ||
      !validateCommonResponse(response, responseLength, id, CMD_RD_REGISTER,
                              registerAddress)) {
    if (_lastResult == MICRO_SERVO_OK) {
      _lastResult = MICRO_SERVO_FORMAT_ERROR;
    }
    return false;
  }

  for (uint8_t i = 0; i < registerCount; ++i) {
    size_t dataIndex = 7 + (size_t)i * 2;
    values[i] = (uint16_t)response[dataIndex] |
                ((uint16_t)response[dataIndex + 1] << 8);
  }

  _lastResult = MICRO_SERVO_OK;
  return true;
}

bool MicroServoController::writeRegisters(uint8_t id, uint16_t registerAddress,
                                          uint8_t registerCount,
                                          const uint16_t *values,
                                          MicroServoStatus *status,
                                          uint16_t timeoutMs) {
  _lastResult = MICRO_SERVO_OK;
  if (id == 0 || id == 0xFF || values == nullptr || registerCount == 0 ||
      registerCount > MICRO_SERVO_MAX_REGISTERS) {
    _lastResult = MICRO_SERVO_INVALID_ARGUMENT;
    return false;
  }

  uint8_t request[8 + MICRO_SERVO_MAX_REGISTERS * 2] = {0};
  size_t requestLength = 8 + (size_t)registerCount * 2;
  request[0] = REQUEST_HEADER_0;
  request[1] = REQUEST_HEADER_1;
  request[2] = (uint8_t)(3 + registerCount * 2);
  request[3] = id;
  request[4] = CMD_WR_REGISTER;
  request[5] = GET_LOW_BYTE(registerAddress);
  request[6] = GET_HIGH_BYTE(registerAddress);
  for (uint8_t i = 0; i < registerCount; ++i) {
    size_t dataIndex = 7 + (size_t)i * 2;
    request[dataIndex] = GET_LOW_BYTE(values[i]);
    request[dataIndex + 1] = GET_HIGH_BYTE(values[i]);
  }
  request[requestLength - 1] =
      calculateChecksum(request, 2, requestLength - 2);

  uint8_t response[STATUS_FRAME_LENGTH] = {0};
  size_t responseLength = 0;

  clearInput();
  waitForCommandInterval();
  _lastCommandMicros = micros();
  _serial->write(request, requestLength);
  _serial->flush();

  if (!readResponseFrame(response, sizeof(response), &responseLength, timeoutMs)) {
    return false;
  }

  return parseStatusResponse(response, responseLength, id, CMD_WR_REGISTER,
                             registerAddress, status);
}

bool MicroServoController::writeRegister(uint8_t id, uint16_t registerAddress,
                                         uint16_t value,
                                         MicroServoStatus *status,
                                         uint16_t timeoutMs) {
  return writeRegisters(id, registerAddress, 1, &value, status, timeoutMs);
}

bool MicroServoController::ParameterSave(uint8_t id) {
  return writeRegister(id, SERVO_REG_SAVE, 1);
}

int MicroServoController::readDeviceID(uint8_t id) {
  uint16_t value = 0;
  if (!readRegisters(id, SERVO_REG_DEVICE_ID, 1, &value)) {
    return -1;
  }
  return (int)(value & 0xFF);
}

bool MicroServoController::setDeviceID(uint8_t id, uint8_t newID) {
  if (newID == 0 || newID == 0xFF) {
    _lastResult = MICRO_SERVO_INVALID_ARGUMENT;
    return false;
  }
  return writeRegister(id, SERVO_REG_DEVICE_ID, newID);
}

bool MicroServoController::setPosition(uint8_t id, int16_t position) {
  if (position < 0 || position > 2000) {
    _lastResult = MICRO_SERVO_INVALID_ARGUMENT;
    return false;
  }

  // Protocol method 2: set positioning mode and target position in one frame.
  const uint16_t values[5] = {
      0,  // 0x25: positioning mode
      0,  // 0x26: unused in positioning mode
      0,  // 0x27: unused in positioning mode
      0,  // 0x28: unused in positioning mode
      (uint16_t)position  // 0x29: target position
  };
  return writeRegisters(id, SERVO_REG_CONTROL_MODE, 5, values);
}

bool MicroServoController::clearError(uint8_t id) {
  return writeRegister(id, SERVO_REG_CLEAR_FAULT, 1);
}

bool MicroServoController::moveFingers(uint8_t num, uint8_t idList[],
                                      int16_t positionList[]) {
  if (num == 0 || idList == nullptr || positionList == nullptr) {
    _lastResult = MICRO_SERVO_INVALID_ARGUMENT;
    return false;
  }

  bool allSucceeded = true;
  for (uint8_t i = 0; i < num; ++i) {
    if (!setPosition(idList[i], positionList[i])) {
      allSucceeded = false;
    }
  }
  return allSucceeded;
}

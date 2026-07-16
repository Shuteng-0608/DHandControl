/*
  MicroServoControl_v2.cpp
  MicroServoControlProtocal v2.0.4 implementation.
*/

#include "MicroServoControl_v2.h"

namespace {
const uint8_t REQUEST_HEADER_0 = 0x55;
const uint8_t REQUEST_HEADER_1 = 0xAA;
const uint8_t RESPONSE_HEADER_0 = 0xAA;
const uint8_t RESPONSE_HEADER_1 = 0x55;
const uint8_t STATUS_RESPONSE_LENGTH = 0x0F;
const size_t STATUS_FRAME_SIZE = 20;
const uint16_t COMMAND_VALUE = 1;
const uint16_t UNUSED_VALUE = 0;
}  // namespace

MicroServoController::MicroServoController(HardwareSerial &serial,
                                           uint32_t baud)
    : _serial(&serial),
      _baudRate(baud),
      _lastOperationSucceeded(false) {}

void MicroServoController::InitServo(int8_t rxPin, int8_t txPin) {
  _serial->begin(_baudRate, SERIAL_8N1, rxPin, txPin);
  delay(100);
  flushInput();
  _lastOperationSucceeded = true;
}

void MicroServoController::ParameterSave(uint8_t id) {
  _lastOperationSucceeded = saveParameters(id);
}

int MicroServoController::readDeviceID(uint8_t id) {
  uint16_t value = 0;
  if (!readRegister(id, REG_DEVICE_ID, value)) {
    return -1;
  }
  if (value < 1 || value > 254) {
    _lastOperationSucceeded = false;
    return -1;
  }
  return static_cast<int>(value);
}

void MicroServoController::setDeviceID(uint8_t id, uint8_t newID) {
  _lastOperationSucceeded = changeDeviceID(id, newID);
}

void MicroServoController::setPosition(uint8_t id, int16_t position) {
  _lastOperationSucceeded = moveToPosition(id, position);
}

void MicroServoController::moveFingers(uint8_t num, uint8_t id_list[],
                                       int16_t pos_list[]) {
  _lastOperationSucceeded = setPositions(num, id_list, pos_list);
}

void MicroServoController::clearError(uint8_t id) {
  _lastOperationSucceeded = clearFault(id);
}

bool MicroServoController::readStatus(uint8_t id, ServoStatus &status,
                                      uint32_t timeoutMs) {
  _lastOperationSucceeded = false;
  if (id == BROADCAST_ID) {
    return false;
  }

  flushInput();
  if (!sendCommand(id, CMD_READ_STATUS, 0x0000, NULL, 0)) {
    return false;
  }

  uint8_t frame[MAX_FRAME_SIZE];
  size_t frameLength = 0;
  if (!readFrame(frame, sizeof(frame), frameLength, timeoutMs)) {
    return false;
  }

  if (!validateResponse(frame, frameLength, id, CMD_READ_STATUS, 0x0000) ||
      !parseStatus(frame, frameLength, status)) {
    return false;
  }

  delay(1);
  _lastOperationSucceeded = true;
  return true;
}

bool MicroServoController::readRegister(uint8_t id, uint16_t address,
                                        uint16_t &value,
                                        uint32_t timeoutMs) {
  return readRegisters(id, address, &value, 1, timeoutMs);
}

bool MicroServoController::readRegisters(uint8_t id, uint16_t startAddress,
                                         uint16_t *values, uint8_t count,
                                         uint32_t timeoutMs) {
  _lastOperationSucceeded = false;
  if (id == BROADCAST_ID || values == NULL || count == 0 ||
      count > MAX_REGISTER_COUNT) {
    return false;
  }

  const uint8_t requestData[1] = {count};
  flushInput();
  if (!sendCommand(id, CMD_READ_REGISTER, startAddress, requestData,
                   sizeof(requestData))) {
    return false;
  }

  uint8_t frame[MAX_FRAME_SIZE];
  size_t frameLength = 0;
  if (!readFrame(frame, sizeof(frame), frameLength, timeoutMs)) {
    return false;
  }

  const size_t expectedFrameLength = 8U + static_cast<size_t>(count) * 2U;
  if (frameLength != expectedFrameLength ||
      frame[2] != static_cast<uint8_t>(3U + count * 2U) ||
      !validateResponse(frame, frameLength, id, CMD_READ_REGISTER,
                        startAddress)) {
    return false;
  }

  for (uint8_t i = 0; i < count; ++i) {
    values[i] = readUint16LE(&frame[7U + static_cast<size_t>(i) * 2U]);
  }

  delay(1);
  _lastOperationSucceeded = true;
  return true;
}

bool MicroServoController::writeRegister(uint8_t id, uint16_t address,
                                         uint16_t value, ServoStatus *status,
                                         uint32_t timeoutMs) {
  return writeRegisters(id, address, &value, 1, status, timeoutMs);
}

bool MicroServoController::writeRegisters(uint8_t id, uint16_t startAddress,
                                          const uint16_t *values,
                                          uint8_t count, ServoStatus *status,
                                          uint32_t timeoutMs) {
  _lastOperationSucceeded = false;
  if (values == NULL || count == 0 || count > MAX_REGISTER_COUNT) {
    return false;
  }

  uint8_t data[MAX_REGISTER_COUNT * 2];
  for (uint8_t i = 0; i < count; ++i) {
    data[static_cast<size_t>(i) * 2U] =
        static_cast<uint8_t>(values[i] & 0xFFU);
    data[static_cast<size_t>(i) * 2U + 1U] =
        static_cast<uint8_t>((values[i] >> 8) & 0xFFU);
  }

  flushInput();
  if (!sendCommand(id, CMD_WRITE_REGISTER, startAddress, data,
                   static_cast<size_t>(count) * 2U)) {
    return false;
  }

  // Broadcast commands are executed by every device and have no response.
  if (id == BROADCAST_ID) {
    delay(1);
    _lastOperationSucceeded = true;
    return true;
  }

  uint8_t frame[MAX_FRAME_SIZE];
  size_t frameLength = 0;
  if (!readFrame(frame, sizeof(frame), frameLength, timeoutMs)) {
    return false;
  }

  const bool oldIdMatches =
      validateResponse(frame, frameLength, id, CMD_WRITE_REGISTER,
                       startAddress);
  const bool newIdMatches =
      startAddress == REG_DEVICE_ID && count > 0 && values[0] <= 0xFFU &&
      validateResponse(frame, frameLength, static_cast<uint8_t>(values[0]),
                       CMD_WRITE_REGISTER, startAddress);
  if ((!oldIdMatches && !newIdMatches) || frameLength != STATUS_FRAME_SIZE ||
      frame[2] != STATUS_RESPONSE_LENGTH) {
    return false;
  }

  ServoStatus parsedStatus;
  if (!parseStatus(frame, frameLength, parsedStatus)) {
    return false;
  }
  if (status != NULL) {
    *status = parsedStatus;
  }

  delay(1);
  _lastOperationSucceeded = true;
  return true;
}

bool MicroServoController::setControlMode(uint8_t id, ControlMode mode,
                                          ServoStatus *status) {
  return writeRegister(id, REG_CONTROL_MODE, static_cast<uint16_t>(mode),
                       status);
}

bool MicroServoController::moveToPosition(uint8_t id, int16_t position,
                                          ServoStatus *status) {
  const uint16_t values[5] = {
      MODE_POSITION, UNUSED_VALUE, UNUSED_VALUE, UNUSED_VALUE,
      asUint16(position)};
  return writeRegisters(id, REG_CONTROL_MODE, values, 5, status);
}

bool MicroServoController::setServoPosition(uint8_t id, int16_t position,
                                            ServoStatus *status) {
  const uint16_t values[5] = {
      MODE_SERVO, UNUSED_VALUE, UNUSED_VALUE, UNUSED_VALUE,
      asUint16(position)};
  return writeRegisters(id, REG_CONTROL_MODE, values, 5, status);
}

bool MicroServoController::moveAtSpeed(uint8_t id, uint16_t speed,
                                       int16_t position,
                                       ServoStatus *status) {
  const uint16_t values[5] = {MODE_SPEED, UNUSED_VALUE, UNUSED_VALUE, speed,
                              asUint16(position)};
  return writeRegisters(id, REG_CONTROL_MODE, values, 5, status);
}

bool MicroServoController::holdForce(uint8_t id, int16_t forceGrams,
                                     ServoStatus *status) {
  const uint16_t values[3] = {MODE_FORCE, UNUSED_VALUE,
                              asUint16(forceGrams)};
  return writeRegisters(id, REG_CONTROL_MODE, values, 3, status);
}

bool MicroServoController::driveVoltage(uint8_t id, int16_t voltage,
                                        ServoStatus *status) {
  const uint16_t values[2] = {MODE_VOLTAGE, asUint16(voltage)};
  return writeRegisters(id, REG_CONTROL_MODE, values, 2, status);
}

bool MicroServoController::moveWithSpeedForce(uint8_t id, uint16_t speed,
                                              int16_t position,
                                              int16_t forceGrams,
                                              ServoStatus *status) {
  const uint16_t values[5] = {MODE_SPEED_FORCE, UNUSED_VALUE,
                              asUint16(forceGrams), speed,
                              asUint16(position)};
  return writeRegisters(id, REG_CONTROL_MODE, values, 5, status);
}

bool MicroServoController::setPositions(uint8_t count, const uint8_t *ids,
                                        const int16_t *positions) {
  if (count == 0) {
    _lastOperationSucceeded = true;
    return true;
  }
  if (ids == NULL || positions == NULL) {
    _lastOperationSucceeded = false;
    return false;
  }

  for (uint8_t i = 0; i < count; ++i) {
    if (!moveToPosition(ids[i], positions[i])) {
      _lastOperationSucceeded = false;
      return false;
    }
  }

  _lastOperationSucceeded = true;
  return true;
}

bool MicroServoController::clearFault(uint8_t id, ServoStatus *status) {
  return writeRegister(id, REG_CLEAR_FAULT, COMMAND_VALUE, status);
}

bool MicroServoController::emergencyStop(uint8_t id, ServoStatus *status) {
  return writeRegister(id, REG_EMERGENCY_STOP, COMMAND_VALUE, status);
}

bool MicroServoController::pauseMotion(uint8_t id, ServoStatus *status) {
  return writeRegister(id, REG_PAUSE_MOTION, COMMAND_VALUE, status);
}

bool MicroServoController::restoreParameters(uint8_t id,
                                             ServoStatus *status) {
  return writeRegister(id, REG_RESTORE_PARAMETERS, COMMAND_VALUE, status);
}

bool MicroServoController::saveParameters(uint8_t id, ServoStatus *status) {
  return writeRegister(id, REG_SAVE_PARAMETERS, COMMAND_VALUE, status);
}

bool MicroServoController::changeDeviceID(uint8_t id, uint8_t newID,
                                          ServoStatus *status) {
  if (newID < 1 || newID > 254) {
    _lastOperationSucceeded = false;
    return false;
  }
  return writeRegister(id, REG_DEVICE_ID, newID, status);
}

bool MicroServoController::setBaudRate(uint8_t id, uint32_t baudRate,
                                       ServoStatus *status) {
  uint16_t code = 0;
  switch (baudRate) {
    case 19200UL:
      code = BAUD_19200;
      break;
    case 57600UL:
      code = BAUD_57600;
      break;
    case 115200UL:
      code = BAUD_115200;
      break;
    case 921600UL:
      code = BAUD_921600;
      break;
    default:
      _lastOperationSucceeded = false;
      return false;
  }
  return writeRegister(id, REG_BAUD_RATE, code, status);
}

bool MicroServoController::readBaudRate(uint8_t id, uint32_t &baudRate) {
  uint16_t code = 0;
  if (!readRegister(id, REG_BAUD_RATE, code)) {
    return false;
  }

  switch (code) {
    case BAUD_19200:
      baudRate = 19200UL;
      break;
    case BAUD_57600:
      baudRate = 57600UL;
      break;
    case BAUD_115200:
      baudRate = 115200UL;
      break;
    case BAUD_921600:
      baudRate = 921600UL;
      break;
    default:
      _lastOperationSucceeded = false;
      return false;
  }

  _lastOperationSucceeded = true;
  return true;
}

bool MicroServoController::lastOperationSucceeded() const {
  return _lastOperationSucceeded;
}

uint8_t MicroServoController::calculateChecksum(const uint8_t *frame,
                                                size_t checksumIndex) const {
  uint8_t sum = 0;
  for (size_t i = 2; i < checksumIndex; ++i) {
    sum = static_cast<uint8_t>(sum + frame[i]);
  }
  return sum;
}

void MicroServoController::flushInput() {
  while (_serial->available() > 0) {
    _serial->read();
  }
}

bool MicroServoController::sendCommand(uint8_t id, uint8_t command,
                                       uint16_t address, const uint8_t *data,
                                       size_t dataLength) {
  if (dataLength > MAX_REGISTER_COUNT * 2U) {
    return false;
  }

  uint8_t frame[MAX_FRAME_SIZE];
  const size_t frameLength = 8U + dataLength;
  frame[0] = REQUEST_HEADER_0;
  frame[1] = REQUEST_HEADER_1;
  frame[2] = static_cast<uint8_t>(3U + dataLength);
  frame[3] = id;
  frame[4] = command;
  frame[5] = static_cast<uint8_t>(address & 0xFFU);
  frame[6] = static_cast<uint8_t>((address >> 8) & 0xFFU);
  for (size_t i = 0; i < dataLength; ++i) {
    frame[7U + i] = data[i];
  }
  frame[frameLength - 1U] = calculateChecksum(frame, frameLength - 1U);

  return _serial->write(frame, frameLength) == frameLength;
}

bool MicroServoController::readFrame(uint8_t *frame, size_t capacity,
                                     size_t &frameLength,
                                     uint32_t timeoutMs) {
  frameLength = 0;
  size_t expectedLength = 0;
  const uint32_t startTime = millis();

  while (static_cast<uint32_t>(millis() - startTime) < timeoutMs) {
    if (_serial->available() <= 0) {
      continue;
    }

    const int received = _serial->read();
    if (received < 0) {
      continue;
    }
    const uint8_t value = static_cast<uint8_t>(received);

    if (frameLength == 0) {
      if (value != RESPONSE_HEADER_0) {
        continue;
      }
      frame[frameLength++] = value;
      continue;
    }

    if (frameLength == 1) {
      if (value == RESPONSE_HEADER_1) {
        frame[frameLength++] = value;
      } else if (value == RESPONSE_HEADER_0) {
        frame[0] = value;
      } else {
        frameLength = 0;
      }
      continue;
    }

    if (frameLength >= capacity) {
      frameLength = 0;
      return false;
    }
    frame[frameLength++] = value;

    if (frameLength == 3) {
      expectedLength = static_cast<size_t>(frame[2]) + 5U;
      if (expectedLength < 8U || expectedLength > capacity) {
        frameLength = 0;
        return false;
      }
    }

    if (expectedLength != 0 && frameLength == expectedLength) {
      return frame[frameLength - 1U] ==
             calculateChecksum(frame, frameLength - 1U);
    }
  }

  frameLength = 0;
  return false;
}

bool MicroServoController::validateResponse(const uint8_t *frame,
                                            size_t frameLength, uint8_t id,
                                            uint8_t command,
                                            uint16_t address) const {
  if (frame == NULL || frameLength < 8U ||
      frame[0] != RESPONSE_HEADER_0 || frame[1] != RESPONSE_HEADER_1 ||
      frame[3] != id || frame[4] != command ||
      frame[5] != static_cast<uint8_t>(address & 0xFFU) ||
      frame[6] != static_cast<uint8_t>((address >> 8) & 0xFFU)) {
    return false;
  }

  return frame[frameLength - 1U] ==
         calculateChecksum(frame, frameLength - 1U);
}

bool MicroServoController::parseStatus(const uint8_t *frame,
                                       size_t frameLength,
                                       ServoStatus &status) const {
  if (frame == NULL || frameLength != STATUS_FRAME_SIZE ||
      frame[2] != STATUS_RESPONSE_LENGTH) {
    return false;
  }

  status.targetPosition = readInt16LE(&frame[7]);
  status.currentPosition = readInt16LE(&frame[9]);
  status.currentMilliAmps = readUint16LE(&frame[11]);
  status.forceGrams = readInt16LE(&frame[13]);
  status.forceAdc = readUint16LE(&frame[15]);
  status.temperatureC = static_cast<int8_t>(frame[17]);
  status.errorCode = frame[18];
  return true;
}

uint16_t MicroServoController::asUint16(int16_t value) {
  return static_cast<uint16_t>(value);
}

int16_t MicroServoController::readInt16LE(const uint8_t *data) {
  return static_cast<int16_t>(readUint16LE(data));
}

uint16_t MicroServoController::readUint16LE(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

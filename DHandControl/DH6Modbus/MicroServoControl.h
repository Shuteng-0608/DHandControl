/*
  MicroServoControl.h - LA/LAS/LAF/LASF/LAXC micro servo cylinder control
  Protocol: PRJ-01-TS-U-007, V2.0.4
*/

#ifndef MICROSERVOCONTROL_H
#define MICROSERVOCONTROL_H

#include <Arduino.h>

#define GET_LOW_BYTE(A)  ((uint8_t)((uint16_t)(A) & 0xFF))
#define GET_HIGH_BYTE(A) ((uint8_t)(((uint16_t)(A) >> 8) & 0xFF))

#define DEFAULT_BAUDRATE 921600
#define TX_PIN 35
#define RX_PIN 34

// V2.0.4 command types.
#define CMD_RD_STATUS   0x30
#define CMD_RD_REGISTER 0x31
#define CMD_WR_REGISTER 0x32

// V2.0.4 register table.
#define SERVO_REG_DEVICE_ID        0x0016
#define SERVO_REG_BAUDRATE         0x0017
#define SERVO_REG_CLEAR_FAULT      0x0018
#define SERVO_REG_EMERGENCY_STOP   0x0019
#define SERVO_REG_PAUSE            0x001A
#define SERVO_REG_RESTORE          0x001B
#define SERVO_REG_SAVE             0x001C
#define SERVO_REG_PERMISSION       0x001D
#define SERVO_REG_CONTROL_MODE     0x0025
#define SERVO_REG_MOTOR_VOLTAGE    0x0026
#define SERVO_REG_FORCE_TARGET     0x0027
#define SERVO_REG_TARGET_SPEED     0x0028
#define SERVO_REG_TARGET_POSITION  0x0029
#define SERVO_REG_CURRENT_POSITION 0x002A
#define SERVO_REG_CURRENT_MA       0x002B
#define SERVO_REG_FORCE_G          0x002C
#define SERVO_REG_FORCE_RAW        0x002D
#define SERVO_REG_TEMPERATURE      0x002E
#define SERVO_REG_ERROR_FLAGS      0x002F

#define MICRO_SERVO_MAX_REGISTERS 8
#define MICRO_SERVO_RESPONSE_TIMEOUT_MS 20

enum MicroServoResult : uint8_t {
  MICRO_SERVO_OK = 0,
  MICRO_SERVO_TIMEOUT = 1,
  MICRO_SERVO_FORMAT_ERROR = 2,
  MICRO_SERVO_CHECKSUM_ERROR = 3,
  MICRO_SERVO_INVALID_ARGUMENT = 4
};

struct MicroServoStatus {
  uint8_t actuatorId;
  int16_t targetPosition;
  int16_t currentPosition;
  uint16_t currentMa;
  int16_t forceG;
  uint16_t forceRaw;
  int8_t temperatureC;
  uint8_t errorFlags;
};

class MicroServoController {
 private:
  HardwareSerial *_serial;
  uint32_t _baudRate;
  uint32_t _lastCommandMicros;
  MicroServoResult _lastResult;

  uint8_t calculateChecksum(const uint8_t *data, size_t firstIndex, size_t lastIndex) const;
  void clearInput();
  void waitForCommandInterval();
  bool readResponseFrame(uint8_t *frame, size_t capacity, size_t *frameLength,
                         uint16_t timeoutMs);
  bool validateCommonResponse(const uint8_t *frame, size_t frameLength,
                              uint8_t expectedId, uint8_t expectedCommand,
                              uint16_t expectedRegister);
  bool parseStatusResponse(const uint8_t *frame, size_t frameLength,
                           uint8_t expectedId, uint8_t expectedCommand,
                           uint16_t expectedRegister, MicroServoStatus *status);

 public:
  MicroServoController(HardwareSerial &serial, uint32_t baud = DEFAULT_BAUDRATE);
  void InitServo(int8_t rxPin = RX_PIN, int8_t txPin = TX_PIN);

  MicroServoResult lastResult() const;
  bool readStatus(uint8_t id, MicroServoStatus *status,
                  uint16_t timeoutMs = MICRO_SERVO_RESPONSE_TIMEOUT_MS);
  bool readRegisters(uint8_t id, uint16_t registerAddress, uint8_t registerCount,
                     uint16_t *values,
                     uint16_t timeoutMs = MICRO_SERVO_RESPONSE_TIMEOUT_MS);
  bool writeRegisters(uint8_t id, uint16_t registerAddress, uint8_t registerCount,
                      const uint16_t *values, MicroServoStatus *status = nullptr,
                      uint16_t timeoutMs = MICRO_SERVO_RESPONSE_TIMEOUT_MS);
  bool writeRegister(uint8_t id, uint16_t registerAddress, uint16_t value,
                     MicroServoStatus *status = nullptr,
                     uint16_t timeoutMs = MICRO_SERVO_RESPONSE_TIMEOUT_MS);

  bool ParameterSave(uint8_t id);
  int readDeviceID(uint8_t id);
  bool setDeviceID(uint8_t id, uint8_t newID);
  bool setPosition(uint8_t id, int16_t position);
  bool clearError(uint8_t id);
  bool moveFingers(uint8_t num, uint8_t idList[], int16_t positionList[]);
};

#endif

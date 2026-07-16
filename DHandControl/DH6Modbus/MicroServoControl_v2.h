/*
  MicroServoControl_v2.h

  Arduino driver for the LA/LAS/LAF/LASF/LAXC micro servo cylinders using
  MicroServoControlProtocal v2.0.4.

  The original public API is kept for drop-in migration.  New code can use
  the bool-returning methods to detect communication failures.
*/

#ifndef MICROSERVOCONTROL_V2_H
#define MICROSERVOCONTROL_V2_H

#include <Arduino.h>

#ifndef GET_LOW_BYTE
#define GET_LOW_BYTE(A) (static_cast<uint8_t>((A) & 0xFFU))
#endif

#ifndef GET_HIGH_BYTE
#define GET_HIGH_BYTE(A) (static_cast<uint8_t>(((A) >> 8) & 0xFFU))
#endif

#ifndef DEFAULT_BAUDRATE
#define DEFAULT_BAUDRATE 921600UL
#endif

#ifndef TX_PIN
#define TX_PIN 35
#endif

#ifndef RX_PIN
#define RX_PIN 34
#endif

#ifndef DEFAULT_RESPONSE_TIMEOUT_MS
#define DEFAULT_RESPONSE_TIMEOUT_MS 20UL
#endif

class MicroServoController {
 public:
  static const uint8_t BROADCAST_ID = 0xFF;

  enum Command : uint8_t {
    CMD_READ_STATUS = 0x30,
    CMD_READ_REGISTER = 0x31,
    CMD_WRITE_REGISTER = 0x32
  };

  enum Register : uint16_t {
    REG_DEVICE_ID = 0x0016,
    REG_BAUD_RATE = 0x0017,
    REG_CLEAR_FAULT = 0x0018,
    REG_EMERGENCY_STOP = 0x0019,
    REG_PAUSE_MOTION = 0x001A,
    REG_RESTORE_PARAMETERS = 0x001B,
    REG_SAVE_PARAMETERS = 0x001C,
    REG_PERMISSION_CODE = 0x001D,
    REG_OVER_TEMPERATURE = 0x001E,
    REG_RECOVERY_TEMPERATURE = 0x001F,
    REG_OVER_CURRENT = 0x0020,
    REG_FORWARD_OUTPUT_LIMIT = 0x0021,
    REG_REVERSE_OUTPUT_LIMIT = 0x0022,
    REG_MAX_POSITION = 0x0023,
    REG_MIN_POSITION = 0x0024,
    REG_CONTROL_MODE = 0x0025,
    REG_OUTPUT_VOLTAGE = 0x0026,
    REG_TARGET_FORCE = 0x0027,
    REG_TARGET_SPEED = 0x0028,
    REG_TARGET_POSITION = 0x0029,
    REG_CURRENT_POSITION = 0x002A,
    REG_CURRENT_CURRENT = 0x002B,
    REG_CURRENT_FORCE = 0x002C,
    REG_FORCE_ADC = 0x002D,
    REG_TEMPERATURE = 0x002E,
    REG_ERROR_CODE = 0x002F
  };

  enum ControlMode : uint16_t {
    MODE_POSITION = 0,
    MODE_SERVO = 1,
    MODE_SPEED = 2,
    MODE_FORCE = 3,
    MODE_VOLTAGE = 4,
    MODE_SPEED_FORCE = 5
  };

  enum BaudRateCode : uint16_t {
    BAUD_19200 = 0,
    BAUD_57600 = 1,
    BAUD_115200 = 2,
    BAUD_921600 = 3
  };

  enum ErrorBits : uint8_t {
    ERROR_STALL = 1U << 0,
    ERROR_OVER_TEMPERATURE = 1U << 1,
    ERROR_OVER_CURRENT = 1U << 2,
    ERROR_MOTOR = 1U << 3,
    ERROR_FLASH = 1U << 4
  };

  struct ServoStatus {
    int16_t targetPosition;
    int16_t currentPosition;
    uint16_t currentMilliAmps;
    int16_t forceGrams;
    uint16_t forceAdc;
    int8_t temperatureC;
    uint8_t errorCode;
  };

  MicroServoController(HardwareSerial &serial,
                       uint32_t baud = DEFAULT_BAUDRATE);

  void InitServo(int8_t rxPin = RX_PIN, int8_t txPin = TX_PIN);

  // Original API retained for source compatibility.
  void ParameterSave(uint8_t id);
  int readDeviceID(uint8_t id);
  void setDeviceID(uint8_t id, uint8_t newID);
  void setPosition(uint8_t id, int16_t position);
  void moveFingers(uint8_t num, uint8_t id_list[], int16_t pos_list[]);
  void clearError(uint8_t id);

  // Protocol primitives.
  bool readStatus(uint8_t id, ServoStatus &status,
                  uint32_t timeoutMs = DEFAULT_RESPONSE_TIMEOUT_MS);
  bool readRegister(uint8_t id, uint16_t address, uint16_t &value,
                    uint32_t timeoutMs = DEFAULT_RESPONSE_TIMEOUT_MS);
  bool readRegisters(uint8_t id, uint16_t startAddress, uint16_t *values,
                     uint8_t count,
                     uint32_t timeoutMs = DEFAULT_RESPONSE_TIMEOUT_MS);
  bool writeRegister(uint8_t id, uint16_t address, uint16_t value,
                     ServoStatus *status = NULL,
                     uint32_t timeoutMs = DEFAULT_RESPONSE_TIMEOUT_MS);
  bool writeRegisters(uint8_t id, uint16_t startAddress,
                      const uint16_t *values, uint8_t count,
                      ServoStatus *status = NULL,
                      uint32_t timeoutMs = DEFAULT_RESPONSE_TIMEOUT_MS);

  // Motion modes introduced by protocol v2.0.4.
  bool setControlMode(uint8_t id, ControlMode mode,
                      ServoStatus *status = NULL);
  bool moveToPosition(uint8_t id, int16_t position,
                      ServoStatus *status = NULL);
  bool setServoPosition(uint8_t id, int16_t position,
                        ServoStatus *status = NULL);
  bool moveAtSpeed(uint8_t id, uint16_t speed, int16_t position,
                   ServoStatus *status = NULL);
  bool holdForce(uint8_t id, int16_t forceGrams,
                 ServoStatus *status = NULL);
  bool driveVoltage(uint8_t id, int16_t voltage,
                    ServoStatus *status = NULL);
  bool moveWithSpeedForce(uint8_t id, uint16_t speed, int16_t position,
                          int16_t forceGrams,
                          ServoStatus *status = NULL);
  bool setPositions(uint8_t count, const uint8_t *ids,
                    const int16_t *positions);

  // Device management commands represented by v2.0.4 registers.
  bool clearFault(uint8_t id, ServoStatus *status = NULL);
  bool emergencyStop(uint8_t id, ServoStatus *status = NULL);
  bool pauseMotion(uint8_t id, ServoStatus *status = NULL);
  bool restoreParameters(uint8_t id, ServoStatus *status = NULL);
  bool saveParameters(uint8_t id, ServoStatus *status = NULL);
  bool changeDeviceID(uint8_t id, uint8_t newID,
                      ServoStatus *status = NULL);
  bool setBaudRate(uint8_t id, uint32_t baudRate,
                   ServoStatus *status = NULL);
  bool readBaudRate(uint8_t id, uint32_t &baudRate);

  bool lastOperationSucceeded() const;

 private:
  static const uint8_t MAX_REGISTER_COUNT = 32;
  static const size_t MAX_FRAME_SIZE = 8 + MAX_REGISTER_COUNT * 2;

  HardwareSerial *_serial;
  uint32_t _baudRate;
  bool _lastOperationSucceeded;

  uint8_t calculateChecksum(const uint8_t *frame,
                            size_t checksumIndex) const;
  void flushInput();
  bool sendCommand(uint8_t id, uint8_t command, uint16_t address,
                   const uint8_t *data, size_t dataLength);
  bool readFrame(uint8_t *frame, size_t capacity, size_t &frameLength,
                 uint32_t timeoutMs);
  bool validateResponse(const uint8_t *frame, size_t frameLength,
                        uint8_t id, uint8_t command,
                        uint16_t address) const;
  bool parseStatus(const uint8_t *frame, size_t frameLength,
                   ServoStatus &status) const;

  static uint16_t asUint16(int16_t value);
  static int16_t readInt16LE(const uint8_t *data);
  static uint16_t readUint16LE(const uint8_t *data);
};

#endif  // MICROSERVOCONTROL_V2_H

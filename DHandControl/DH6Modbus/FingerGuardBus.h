#pragma once
#include <Arduino.h>
#include "FingerGuard.h"

namespace mh6 {
// V2.0.4 only. One owner and one outstanding transaction on the finger UART.
class FingerGuardBus {
public:
    explicit FingerGuardBus(HardwareSerial &port): serial(port) {}
    enum class Result { None, Good, Timeout };
    bool busy = false;
    uint8_t id = 0, command = 0;
    uint16_t address = 0;
    FingerFeedback feedback;
    Result poll(uint32_t now, uint16_t timeoutMs);
    bool query(uint8_t deviceId, uint32_t now);
    bool write(uint8_t deviceId, uint16_t reg, const uint16_t *values, uint8_t count, uint32_t now);
    void drain();
    static uint8_t checksum(const uint8_t *data, uint8_t count);
private:
    HardwareSerial &serial;
    uint8_t rx[20] = {}, used = 0;
    uint32_t started = 0;
    bool send(uint8_t *data, uint8_t bytes, uint32_t now);
};
}

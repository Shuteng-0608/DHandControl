#pragma once
#include "FingerGuardBus.h"
#include "FingerGuardConfig.h"

namespace mh6 {
class FingerGuardRuntime {
public:
    explicit FingerGuardRuntime(HardwareSerial &port): bus(port) {}
    FingerGuard fingers[5];
    bool armed = false;
    void begin();
    void begin(const FingerLimits *limits, bool enable, uint8_t enabledMask=0x1F);
    bool configured() const;
    bool palmsAllowed(uint32_t now) const;
    bool accept(uint8_t count, const uint8_t *ids, const int16_t *positions, uint32_t now);
    bool clearError(uint8_t id, uint32_t now); // explicit, executed by UART owner
    uint16_t clearResult(uint8_t id) const; // 0=pending, 1=verified, 2=rejected
    void tick(uint32_t now);
    void registers(uint16_t *destination, uint16_t count, uint32_t now) const;
private:
    FingerGuardBus bus;
    bool clearPending[5] = {};
    uint16_t stopSent[5] = {};
    uint16_t clearResults[5] = {};
    uint32_t lastSlot = 0, quarantineAt = 0;
    bool sentSlot = false, quarantine = false, clearing = false;
    uint8_t next = 0;
    uint32_t validReplies[5] = {};
    uint8_t enabledMask = 0x1F;
};
}

#pragma once
#include <stdint.h>

// Pure state machine: no UART, delay, allocation, or dependence on host rate.
namespace mh6 {
enum class FingerState : uint16_t { Waiting, Following, Holding, Opening, Stale, Fault };
enum class FingerReason : uint16_t { None, Unconfigured, FeedbackMissing, CommandExpired,
    OverForce, OverCurrent, OverTemperature, DriverFault, Communication, PositionInvalid };
struct FingerLimits {
    // Feature switches, not claims of physical validation.
    bool forceControlEnabled = false;
    bool nativeControlEnabled = false;
    int8_t forceSign = 0;
    int16_t minimum = 20, maximum = 1950;
    uint16_t feedbackPositionToleranceSteps = 20;
    uint16_t releaseForceG = 0, contactForceG = 0, holdForceG = 0,
             limitForceG = 0, dangerForceG = 0;
    uint16_t continuousCurrentMa = 0, tripCurrentMa = 0, maximumTemperatureC = 0;
    uint16_t outputLimit = 0, speedStepsPerSecond = 300, maximumLeadSteps = 10;
    uint16_t contactMs = 30, releaseMs = 60, overCurrentMs = 100;
    uint16_t feedbackTimeoutMs = 100;
    uint16_t commandTimeoutMs = 0; // 0 disables expiry; latest target stays active.
    bool valid() const;
    bool feedbackPositionValid(int16_t position) const;
};
struct FingerFeedback {
    int16_t target = 0, position = 0, forceG = 0;
    uint16_t currentMa = 0, forceAdc = 0;
    int8_t temperatureC = 0;
    uint8_t errors = 0;
};
struct FingerAction {
    // 0xFF=query, 0xFE=pause, 0xFD=emergency stop; 2/3/5 are V2 modes.
    uint8_t mode = 0xFF;
    int16_t target = 0, upper = 0, forceTargetG = 0;
};
class FingerGuard {
public:
    FingerLimits limits;
    FingerState state = FingerState::Waiting;
    FingerReason reason = FingerReason::Unconfigured;
    FingerFeedback feedback;
    bool hasFeedback = false, hasTarget = false, urgentOpening = false;
    int16_t desired = 20, applied = 20;
    uint32_t feedbackAt = 0, targetAt = 0, targetGeneration = 0;
    uint8_t nativeMode = 0xFF;
    int16_t nativeUpper = 20;
    void setTarget(int16_t target, uint32_t now);
    void observe(const FingerFeedback &sample, uint32_t now);
    void communicationFault();
    bool clearLatch(); // Explicit only; never periodic or automatic.
    FingerAction decide(uint32_t now, bool armed);
private:
    bool contactPending = false, releasePending = false, currentPending = false;
    bool commandExpired = false, openingIntent = false;
    int16_t holdingCeiling = 20;
    uint32_t contactAt = 0, releaseAt = 0, currentAt = 0, decisionAt = 0;
    uint32_t expiredGeneration = 0;
    double continuousApplied = 20.;
    bool timer(bool condition, bool &pending, uint32_t &since, uint32_t now, uint16_t duration);
    void fault(FingerReason cause);
};
}

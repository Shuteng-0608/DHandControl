#pragma once
#include "FingerGuard.h"

// Deliberately NOT armed: actual sensor validity and safe loads are not known.
// Fill every finger independently, then set true after low-load bench checks.
// This header is RAM-only configuration; no actuator Flash writes are issued.
constexpr bool FINGER_GUARD_ARMED = false;
// Commission one finger with e.g. 0x01; full teleoperation requires all five.
constexpr uint8_t FINGER_GUARD_MASK = 0x1F;
constexpr uint16_t FINGER_GUARD_SLOT_MS = 4; // nominal <=50 Hz per finger (5 fingers)
constexpr uint16_t FINGER_GUARD_UART_TIMEOUT_MS = 5;
constexpr uint16_t FINGER_GUARD_UART_QUARANTINE_MS = 25;
inline mh6::FingerLimits fingerGuardLimits(uint8_t id) {
    mh6::FingerLimits limits;
    // Existing physical travel: 20=open, 1950=closed. Increasing means closing.
    // Use separate rows here when measured force/current parameters differ.
    (void)id;
    // REQUIRED: forceVerified, nativeModesAndLimitsVerified, forceSign,
    // releaseForceG, contactForceG,
    // holdForceG, limitForceG, dangerForceG, continuousCurrentMa, tripCurrentMa,
    // maximumTemperatureC, outputLimit. Zero/default values keep motion locked.
    return limits;
}

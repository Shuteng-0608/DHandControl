#pragma once
#include "FingerGuard.h"

// Operator-confirmed series: LASF (force-sensor hardware; V2.0.4 manual pp. 1-2).
// Operator confirms positive force feedback while grasping: all five use +1.
// Sensor baseline, installed firmware behavior and safe loads still need checks.
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
    limits.minimum = 20;
    limits.maximum = 1950;
    // Initial test settings only; they are not measured actuator safety ratings.
    limits.speedStepsPerSecond = 300;
    limits.maximumLeadSteps = 10;
    limits.contactMs = 30;
    limits.releaseMs = 60;
    limits.overCurrentMs = 100;
    limits.feedbackTimeoutMs = 100;
    limits.commandTimeoutMs = 500;

    // Fill each F1-F5 block from actual checks/specifications. Zero means UNKNOWN,
    // not zero allowable load. Unknown values keep the whole-hand motion locked.
    // Force: g; current: mA; temperature: deg C; output: 1..1000.
    // Required: 0 < release < contact <= hold < limit < danger;
    //           0 < continuousCurrent < tripCurrent; forceSign is +1 or -1.
    switch (id) {
    case 1: // F1
        limits.forceVerified = false;
        limits.nativeModesAndLimitsVerified = false;
        limits.forceSign = 1;
        limits.releaseForceG = 0;
        limits.contactForceG = 0;
        limits.holdForceG = 0;
        limits.limitForceG = 0;
        limits.dangerForceG = 0;
        limits.continuousCurrentMa = 0;
        limits.tripCurrentMa = 0;
        limits.maximumTemperatureC = 0;
        limits.outputLimit = 0;
        break;
    case 2: // F2
        limits.forceVerified = false;
        limits.nativeModesAndLimitsVerified = false;
        limits.forceSign = 1;
        limits.releaseForceG = 0;
        limits.contactForceG = 0;
        limits.holdForceG = 0;
        limits.limitForceG = 0;
        limits.dangerForceG = 0;
        limits.continuousCurrentMa = 0;
        limits.tripCurrentMa = 0;
        limits.maximumTemperatureC = 0;
        limits.outputLimit = 0;
        break;
    case 3: // F3
        limits.forceVerified = false;
        limits.nativeModesAndLimitsVerified = false;
        limits.forceSign = 1;
        limits.releaseForceG = 0;
        limits.contactForceG = 0;
        limits.holdForceG = 0;
        limits.limitForceG = 0;
        limits.dangerForceG = 0;
        limits.continuousCurrentMa = 0;
        limits.tripCurrentMa = 0;
        limits.maximumTemperatureC = 0;
        limits.outputLimit = 0;
        break;
    case 4: // F4
        limits.forceVerified = false;
        limits.nativeModesAndLimitsVerified = false;
        limits.forceSign = 1;
        limits.releaseForceG = 0;
        limits.contactForceG = 0;
        limits.holdForceG = 0;
        limits.limitForceG = 0;
        limits.dangerForceG = 0;
        limits.continuousCurrentMa = 0;
        limits.tripCurrentMa = 0;
        limits.maximumTemperatureC = 0;
        limits.outputLimit = 0;
        break;
    case 5: // F5
        limits.forceVerified = false;
        limits.nativeModesAndLimitsVerified = false;
        limits.forceSign = 1;
        limits.releaseForceG = 0;
        limits.contactForceG = 0;
        limits.holdForceG = 0;
        limits.limitForceG = 0;
        limits.dangerForceG = 0;
        limits.continuousCurrentMa = 0;
        limits.tripCurrentMa = 0;
        limits.maximumTemperatureC = 0;
        limits.outputLimit = 0;
        break;
    default:
        return mh6::FingerLimits(); // Invalid IDs never inherit another finger.
    }
    return limits;
}

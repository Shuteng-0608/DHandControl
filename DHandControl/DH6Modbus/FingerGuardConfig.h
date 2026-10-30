#pragma once
#include "FingerGuard.h"

// Operator-confirmed series: LASF (force-sensor hardware; V2.0.4 manual pp. 1-2).
// Operator confirms positive force feedback while grasping: all five use +1.
// Operator-approved LOW-FORCE TRIAL values below are not validated load ratings.
// All five fingers are enabled for the operator-requested low-force trial.
// Fresh feedback and all runtime fault/timeout protections remain mandatory.
// This header is RAM-only configuration; no actuator Flash writes are issued.
constexpr bool FINGER_GUARD_ARMED = true;
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

    // F1-F5 use the approved trial profile; each block remains independently editable.
    // 800/1200 mA are ESP32 software thresholds. Native register 0x20 is left
    // unchanged, so the actuator's own overcurrent protection remains active.
    // Feature switches authorize the trial; they do not certify physical tests.
    // Force: g; current: mA; temperature: deg C; output: 1..1000.
    // Required: 0 < release < contact <= hold < limit < danger;
    //           0 < continuousCurrent < tripCurrent; forceSign is +1 or -1.
    switch (id) {
    case 1: // F1
        limits.forceControlEnabled = true;
        limits.nativeControlEnabled = true;
        limits.forceSign = 1;
        limits.releaseForceG = 100;
        limits.contactForceG = 300;
        limits.holdForceG = 400;
        limits.limitForceG = 600;
        limits.dangerForceG = 1000;
        limits.continuousCurrentMa = 800;
        limits.tripCurrentMa = 1200;
        limits.maximumTemperatureC = 45;
        limits.outputLimit = 200;
        break;
    case 2: // F2
        limits.forceControlEnabled = true;
        limits.nativeControlEnabled = true;
        limits.forceSign = 1;
        limits.releaseForceG = 100;
        limits.contactForceG = 300;
        limits.holdForceG = 400;
        limits.limitForceG = 600;
        limits.dangerForceG = 1000;
        limits.continuousCurrentMa = 800;
        limits.tripCurrentMa = 1200;
        limits.maximumTemperatureC = 45;
        limits.outputLimit = 200;
        break;
    case 3: // F3
        limits.forceControlEnabled = true;
        limits.nativeControlEnabled = true;
        limits.forceSign = 1;
        limits.releaseForceG = 100;
        limits.contactForceG = 300;
        limits.holdForceG = 400;
        limits.limitForceG = 600;
        limits.dangerForceG = 1000;
        limits.continuousCurrentMa = 800;
        limits.tripCurrentMa = 1200;
        limits.maximumTemperatureC = 45;
        limits.outputLimit = 200;
        break;
    case 4: // F4
        limits.forceControlEnabled = true;
        limits.nativeControlEnabled = true;
        limits.forceSign = 1;
        limits.releaseForceG = 100;
        limits.contactForceG = 300;
        limits.holdForceG = 400;
        limits.limitForceG = 600;
        limits.dangerForceG = 1000;
        limits.continuousCurrentMa = 800;
        limits.tripCurrentMa = 1200;
        limits.maximumTemperatureC = 45;
        limits.outputLimit = 200;
        break;
    case 5: // F5
        limits.forceControlEnabled = true;
        limits.nativeControlEnabled = true;
        limits.forceSign = 1;
        limits.releaseForceG = 100;
        limits.contactForceG = 300;
        limits.holdForceG = 400;
        limits.limitForceG = 600;
        limits.dangerForceG = 1000;
        limits.continuousCurrentMa = 800;
        limits.tripCurrentMa = 1200;
        limits.maximumTemperatureC = 45;
        limits.outputLimit = 200;
        break;
    default:
        return mh6::FingerLimits(); // Invalid IDs never inherit another finger.
    }
    return limits;
}

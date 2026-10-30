#include "FingerGuard.h"
#include <algorithm>
#include <cmath>

namespace mh6 {
bool FingerLimits::valid() const {
    return forceControlEnabled && nativeControlEnabled &&
        (forceSign == 1 || forceSign == -1) && minimum >= 0 &&
        maximum <= 2000 && maximum > minimum && feedbackPositionToleranceSteps <= 100 &&
        releaseForceG > 0 && releaseForceG < contactForceG &&
        contactForceG <= holdForceG && holdForceG < limitForceG &&
        limitForceG < dangerForceG && dangerForceG <= 30000 &&
        continuousCurrentMa > 0 && continuousCurrentMa < tripCurrentMa &&
        maximumTemperatureC > 0 && maximumTemperatureC <= 80 &&
        outputLimit > 0 && outputLimit <= 1000 && speedStepsPerSecond > 0 &&
        maximumLeadSteps > 0 && maximumLeadSteps <= 100 && contactMs > 0 && releaseMs > 0 &&
        overCurrentMs > 0 && feedbackTimeoutMs >= 20 && feedbackTimeoutMs <= 500 &&
        (commandTimeoutMs == 0 || (commandTimeoutMs >= 100 && commandTimeoutMs <= 2000));
}
bool FingerLimits::feedbackPositionValid(int16_t position) const {
    // Feedback is measured encoder position, not a requested motion target.
    // Permit a small endpoint offset while retaining the protocol's hard bounds.
    const int32_t measured=position;
    return measured>=0 && measured<=2000 &&
        measured>=int32_t(minimum)-feedbackPositionToleranceSteps &&
        measured<=int32_t(maximum)+feedbackPositionToleranceSteps;
}
bool FingerGuard::timer(bool c, bool &pending, uint32_t &since, uint32_t now, uint16_t duration) {
    if (!c) { pending = false; return false; }
    if (!pending) { pending = true; since = now; }
    return uint32_t(now - since) >= duration;
}
void FingerGuard::fault(FingerReason cause) {
    state = FingerState::Fault; reason = cause;
}
void FingerGuard::setTarget(int16_t target, uint32_t now) {
    // Always replace the target; no queue and no latched pre-contact destination.
    const int16_t old = desired;
    desired = std::max(limits.minimum, std::min(limits.maximum, target));
    targetAt = now; ++targetGeneration; hasTarget = true;
    if ((desired < old && desired <= applied) || (hasFeedback && desired < feedback.position)) {
        openingIntent = true; urgentOpening = true;
    } else if (desired > old) {
        openingIntent = false;
    }
}
void FingerGuard::observe(const FingerFeedback &sample, uint32_t now) {
    feedback = sample; feedbackAt = now;
    if (!hasFeedback) { applied = sample.position; continuousApplied = applied; }
    hasFeedback = true;
    if (!limits.valid()) return;
    if (!limits.feedbackPositionValid(sample.position)) {
        fault(FingerReason::PositionInvalid); return;
    }
    if (sample.errors) { fault(FingerReason::DriverFault); return; }
    const int32_t force = int32_t(sample.forceG) * limits.forceSign;
    if (std::abs(int32_t(sample.forceG)) >= limits.dangerForceG) {
        fault(FingerReason::OverForce); return;
    }
    if (sample.temperatureC >= limits.maximumTemperatureC) {
        fault(FingerReason::OverTemperature); return;
    }
    if (sample.currentMa >= limits.tripCurrentMa ||
        timer(sample.currentMa > limits.continuousCurrentMa, currentPending, currentAt,
              now, limits.overCurrentMs)) {
        fault(FingerReason::OverCurrent); return;
    }
    if (state == FingerState::Fault || !hasTarget) return;
    if (openingIntent) { contactPending = releasePending = false; return; }
    if (timer(force >= limits.contactForceG && desired > sample.position,
              contactPending, contactAt, now, limits.contactMs)) {
        if (state != FingerState::Holding) {
            holdingCeiling=std::min(limits.maximum,int16_t(sample.position+limits.maximumLeadSteps));
        }
        state = FingerState::Holding; reason = FingerReason::None;
        // Remove the old position error on contact. Native force control takes over.
        applied = sample.position; continuousApplied = applied;
    }
    if (state == FingerState::Holding &&
        timer(force < limits.releaseForceG, releasePending, releaseAt, now, limits.releaseMs)) {
        state = FingerState::Following; contactPending = false;
        applied = sample.position; continuousApplied = applied; decisionAt = now;
    }
}
void FingerGuard::communicationFault() { fault(FingerReason::Communication); }
bool FingerGuard::clearLatch() {
    if (!hasFeedback || feedback.errors || !limits.valid() ||
        !limits.feedbackPositionValid(feedback.position) ||
        std::abs(int32_t(feedback.forceG)) >= limits.contactForceG ||
        feedback.currentMa > limits.continuousCurrentMa ||
        feedback.temperatureC >= limits.maximumTemperatureC) return false;
    state = FingerState::Waiting; reason = FingerReason::None;
    hasTarget = false; urgentOpening = openingIntent = false;
    contactPending = releasePending = currentPending = false;
    applied = feedback.position; continuousApplied = applied;
    return true;
}
FingerAction FingerGuard::decide(uint32_t now, bool armed) {
    FingerAction action;
    if (!armed || !limits.valid()) { reason = FingerReason::Unconfigured; return action; }
    if (state == FingerState::Fault) { action.mode = 0xFD; return action; }
    if (!hasFeedback || uint32_t(now-feedbackAt) > limits.feedbackTimeoutMs) {
        state = FingerState::Stale; reason = FingerReason::FeedbackMissing;
        // Do not leave an autonomous force target active when observation is lost.
        action.mode = hasTarget ? 0xFE : 0xFF; return action;
    }
    if (!hasTarget) return action;
    if (limits.commandTimeoutMs && uint32_t(now-targetAt) > limits.commandTimeoutMs) {
        // Optional command expiry. Zero keeps a one-shot target active.
        state = FingerState::Stale; reason = FingerReason::CommandExpired;
        commandExpired = true; expiredGeneration = targetGeneration; action.mode = 0xFE;
        return action;
    }
    if (state == FingerState::Stale || commandExpired) {
        if (commandExpired && limits.commandTimeoutMs && targetGeneration == expiredGeneration) {
            action.mode=0xFE; return action;
        }
        applied = feedback.position; continuousApplied = applied;
        state = FingerState::Following; commandExpired = false; decisionAt = now;
    }
    reason = FingerReason::None;
    uint32_t elapsed = decisionAt ? std::min(uint32_t(now-decisionAt),uint32_t(20)) : 20;
    decisionAt = now;
    const double step = double(limits.speedStepsPerSecond)*elapsed/1000.;
    // Rebase the position ramp after contact; it never winds up behind an obstacle.
    continuousApplied = std::min(continuousApplied, double(feedback.position+limits.maximumLeadSteps));
    if (openingIntent) {
        // A shallower new goal cancels force hold immediately. If it is still
        // beyond contact, stop at contact instead of autonomously re-closing.
        const int16_t goal = std::min(desired, feedback.position);
        // Native velocity mode enforces speed without waiting for a software
        // ramp to remove a previously deeper force/position destination.
        continuousApplied = goal;
        state = FingerState::Opening;
        action.mode = 2;
    } else if (state == FingerState::Holding) {
        action.mode = 3; continuousApplied = feedback.position;
    } else {
        continuousApplied += std::max(-step, std::min(step, double(desired)-continuousApplied));
        state = FingerState::Following;
        action.mode = desired < feedback.position ? 2 : 5;
    }
    applied = std::max(limits.minimum,std::min(limits.maximum,int16_t(std::lround(continuousApplied))));
    action.target = applied;
    action.upper = std::min(limits.maximum,
        std::max(feedback.position,std::min(desired,int16_t(feedback.position+limits.maximumLeadSteps))));
    if (action.mode==3) {
        // Freeze the force-mode envelope at contact. If the object disappears,
        // repeated observations must NOT walk this bound forward before release
        // detection switches back to explicitly speed-limited position tracking.
        action.upper=std::min(limits.maximum,std::max(feedback.position,
            std::min(desired,holdingCeiling)));
    }
    // Example: measured 9 + lead 10 = 19, below the command lower bound 20.
    // Recover toward a legal target without sending upper < lower or target > upper.
    action.upper=std::max(limits.minimum,std::max(action.target,action.upper));
    action.forceTargetG = action.mode == 3 ? int16_t(limits.holdForceG*limits.forceSign)
                                         : int16_t(limits.limitForceG);
    nativeMode = action.mode; nativeUpper = action.upper;
    urgentOpening = false;
    return action;
}
}

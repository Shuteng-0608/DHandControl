#include "FingerGuardRuntime.h"
#include <algorithm>
#include <cstdlib>

namespace mh6 {
void FingerGuardRuntime::begin() {
    FingerLimits cfg[5]; for (uint8_t i=0;i<5;++i) cfg[i]=fingerGuardLimits(i+1);
    begin(cfg,FINGER_GUARD_ARMED,FINGER_GUARD_MASK);
}
void FingerGuardRuntime::begin(const FingerLimits *cfg,bool enable,uint8_t mask) {
    armed=enable; enabledMask=mask;
    bus.busy=false; sentSlot=false; quarantine=false; clearing=false; next=0;
    for (uint8_t i=0;i<5;++i) {
        clearPending[i]=false; stopSent[i]=0; clearResults[i]=0; validReplies[i]=0;
    }
    for (uint8_t i=0;i<5;++i) { fingers[i]=FingerGuard(); fingers[i].limits=cfg[i]; }
}
bool FingerGuardRuntime::configured() const {
    if (!armed || !enabledMask || (enabledMask & ~0x1F)) return false;
    for (uint8_t i=0;i<5;++i) if ((enabledMask & (1<<i)) && !fingers[i].limits.valid()) return false;
    return true;
}
bool FingerGuardRuntime::palmsAllowed(uint32_t now) const {
    if (!configured() || enabledMask!=0x1F) return false;
    for (const auto &f:fingers) {
        if (!f.hasFeedback || uint32_t(now-f.feedbackAt)>f.limits.feedbackTimeoutMs ||
            f.state==FingerState::Fault || f.state==FingerState::Stale) return false;
    }
    return true;
}
bool FingerGuardRuntime::accept(uint8_t count,const uint8_t *ids,const int16_t *positions,uint32_t now) {
    if (!configured() || !count || count>5) return false;
    uint8_t seen=0;
    for (uint8_t i=0;i<count;++i) {
        if (ids[i]<1 || ids[i]>5 || !(enabledMask & (1<<(ids[i]-1))) ||
            (seen & (1<<(ids[i]-1)))) return false;
        seen|=1<<(ids[i]-1);
        const auto &cfg=fingers[ids[i]-1].limits;
        if (positions[i]<cfg.minimum || positions[i]>cfg.maximum) return false;
    }
    for (uint8_t i=0;i<count;++i) fingers[ids[i]-1].setTarget(positions[i],now);
    return true;
}
bool FingerGuardRuntime::clearError(uint8_t id,uint32_t now) {
    if (id<1 || id>5 || !(enabledMask & (1<<(id-1)))) return false;
    auto &f=fingers[id-1];
    // Clearing under load would repeatedly re-enable a damaged/blocked motor.
    if (!f.limits.valid() || !f.hasFeedback || uint32_t(now-f.feedbackAt)>f.limits.feedbackTimeoutMs ||
        !f.limits.feedbackPositionValid(f.feedback.position) ||
        std::abs(int32_t(f.feedback.forceG))>=f.limits.contactForceG ||
        f.feedback.currentMa>f.limits.continuousCurrentMa ||
        f.feedback.temperatureC>=f.limits.maximumTemperatureC) return false;
    clearPending[id-1]=true; clearResults[id-1]=0; return true;
}
uint16_t FingerGuardRuntime::clearResult(uint8_t id) const {
    return id>=1 && id<=5 ? clearResults[id-1]:2;
}
void FingerGuardRuntime::tick(uint32_t now) {
    const auto result=bus.poll(now,FINGER_GUARD_UART_TIMEOUT_MS);
    if (result!=FingerGuardBus::Result::None) {
        const uint8_t i=bus.id-1;
        if (result==FingerGuardBus::Result::Good) {
            fingers[i].observe(bus.feedback,now); ++validReplies[i];
            if (clearing) {
                clearResults[i]=fingers[i].clearLatch()?1:2;
                stopSent[i]=0;
            }
        } else {
            if (configured() && (enabledMask & (1<<i))) fingers[i].communicationFault();
            if (bus.command==0x32 && (bus.address==0x19 || bus.address==0x1A)) stopSent[i]=0;
            if (clearing) clearResults[i]=2;
            quarantine=true; quarantineAt=now;
        }
        clearing=false;
    }
    if (bus.busy) return;
    if (quarantine) {
        bus.drain();
        if (uint32_t(now-quarantineAt)<FINGER_GUARD_UART_QUARANTINE_MS) return;
        quarantine=false;
    }
    if (sentSlot && uint32_t(now-lastSlot)<FINGER_GUARD_SLOT_MS) return;
    // Round-robin bounds the update time of every finger. A new opening target
    // may take the next slot, provided another finger is not already overdue.
    uint8_t index=next; bool overdue=false;
    for (uint8_t k=0;k<5;++k) {
        const uint8_t i=(next+k)%5;
        if (fingers[i].hasFeedback && uint32_t(now-fingers[i].feedbackAt)>=25) {
            index=i; overdue=true; break;
        }
    }
    if (!overdue) for (uint8_t k=0;k<5;++k) {
        const uint8_t i=(next+k)%5;
        if (fingers[i].urgentOpening) { index=i; break; }
    }
    next=(index+1)%5;
    auto &f=fingers[index];
    bool sent=false;
    if (clearPending[index]) {
        const uint16_t one=1;
        sent=bus.write(index+1,0x18,&one,1,now);
        if (sent) { clearPending[index]=false; clearing=true; }
    } else {
        const auto action=f.decide(now,configured() && (enabledMask & (1<<index)));
        const uint16_t stopRegister=action.mode==0xFD ? 0x19:0x1A;
        if ((action.mode==0xFD || action.mode==0xFE) && stopSent[index]!=stopRegister) {
            const uint16_t one=1;
            sent=bus.write(index+1,stopRegister,&one,1,now);
            if (sent) { stopSent[index]=stopRegister; f.nativeMode=action.mode; }
        } else if (action.mode==0xFF || action.mode==0xFE || action.mode==0xFD) {
            sent=bus.query(index+1,now);
        } else {
            // Write 0x21..0x29 only: output bounds, stroke, mode, force, speed,
            // position. Preserve native overcurrent register 0x20 unchanged.
            // tripCurrentMa remains an ESP32 software-feedback trip threshold.
            // Force mode ignores 0x29: stroke upper bound is therefore essential.
            const uint16_t words[9]={f.limits.outputLimit,
                f.limits.outputLimit,uint16_t(action.upper),uint16_t(f.limits.minimum),
                action.mode,0,uint16_t(action.forceTargetG),f.limits.speedStepsPerSecond,
                uint16_t(action.target)};
            sent=bus.write(index+1,0x21,words,9,now);
            if (sent) stopSent[index]=0;
        }
    }
    if (!sent) f.communicationFault();
    lastSlot=now; sentSlot=true;
}
void FingerGuardRuntime::registers(uint16_t *out,uint16_t count,uint32_t now) const {
    if (count<86) return;
    for (uint16_t i=0;i<count;++i) out[i]=0;
    out[0]=0x4647; out[1]=1;
    out[2]=(uint16_t(enabledMask)<<8)|(armed?1:0)|(configured()?2:0); out[3]=0x0204;
    for (uint8_t i=0;i<5;++i) {
        const auto &f=fingers[i];
        if (f.state==FingerState::Holding) ++out[4];
        if (f.state==FingerState::Fault || f.state==FingerState::Stale) out[5]|=1<<i;
        auto *r=out+6+16*i;
        r[0]=uint16_t(f.state); r[1]=uint16_t(f.reason); r[2]=f.desired; r[3]=f.applied;
        r[4]=f.feedback.position; r[5]=f.feedback.forceG; r[6]=f.feedback.currentMa;
        r[7]=int16_t(f.feedback.temperatureC);
        r[8]=f.hasFeedback?std::min(uint32_t(65534),uint32_t(now-f.feedbackAt)):65535;
        r[9]=f.feedback.errors; r[10]=validReplies[i]>>16; r[11]=validReplies[i];
        r[12]=f.nativeMode; r[13]=f.nativeUpper;
        r[14]=f.hasTarget?std::min(uint32_t(65534),uint32_t(now-f.targetAt)):65535;
        r[15]=f.hasFeedback && uint32_t(now-f.feedbackAt)<=f.limits.feedbackTimeoutMs;
    }
}
}

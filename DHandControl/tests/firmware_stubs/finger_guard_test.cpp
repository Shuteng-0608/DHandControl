#include "FingerGuard.h"
#include "FingerGuardBus.h"
#include "FingerGuardRuntime.h"
#include <cassert>
#include <iostream>
#include <vector>
using namespace mh6;

FingerLimits benchLimits() {
    FingerLimits c;
    // Synthetic units for tests only. These are not suggested hardware loads.
    c.forceControlEnabled=true; c.nativeControlEnabled=true; c.forceSign=1;
    c.releaseForceG=50; c.contactForceG=100; c.holdForceG=150;
    c.limitForceG=250; c.dangerForceG=400; c.continuousCurrentMa=200;
    c.tripCurrentMa=500; c.maximumTemperatureC=60; c.outputLimit=100;
    c.speedStepsPerSecond=200; c.contactMs=10; c.releaseMs=20;
    c.overCurrentMs=20; c.commandTimeoutMs=500; return c;
}
FingerFeedback sample(int pos,int force=0,int current=50) {
    FingerFeedback f; f.position=pos; f.forceG=force; f.currentMa=current; f.temperatureC=25;
    return f;
}
std::vector<uint8_t> v2reply(uint8_t id,uint8_t cmd,uint16_t reg,int pos=100,int force=0) {
    std::vector<uint8_t> p={0xAA,0x55,15,id,cmd,uint8_t(reg),uint8_t(reg>>8),
        0x34,0x12,uint8_t(pos),uint8_t(pos>>8),0x56,0x04,uint8_t(force),uint8_t(force>>8),
        0x89,0x07,32,0,0};
    p.back()=FingerGuardBus::checksum(p.data(),19); return p;
}
void stateTests() {
    FingerGuard g; assert(!g.limits.valid());
    g.limits=benchLimits(); assert(g.limits.valid());
    g.observe(sample(100),100); g.setTarget(300,100);
    auto a=g.decide(100,false); assert(a.mode==0xFF);
    a=g.decide(100,true); assert(a.mode==5 && a.target<=104 && a.upper<=110);
    g.observe(sample(104,120),110); g.observe(sample(104,120),121);
    a=g.decide(121,true); assert(g.state==FingerState::Holding && a.mode==3);
    assert(a.forceTargetG==150 && a.upper<=114 && g.desired==300);
    // Deep new target is remembered, never applied raw while holding.
    g.setTarget(400,122); a=g.decide(122,true);
    assert(a.mode==3 && a.upper<=114 && g.desired==400);
    // A replacement goal still beyond contact replaces 400, it is not a queue.
    g.setTarget(200,123); a=g.decide(123,true); assert(a.mode==3 && g.desired==200);
    auto envelope=g;
    envelope.observe(sample(110,0),124); a=envelope.decide(124,true); assert(a.upper==114);
    envelope.observe(sample(113,0),130); a=envelope.decide(130,true); assert(a.upper==114);
    // Object removed: no release command is sent, resume smoothly toward 200.
    g.observe(sample(104,0),125); g.observe(sample(104,0),146);
    g.setTarget(200,146); a=g.decide(146,true);
    assert(a.mode==5 && a.target<=108 && a.target<=200);
    // New opening overrides force hold even with nonzero contact force.
    g.observe(sample(110,120),150); g.observe(sample(110,120),161);
    assert(g.decide(161,true).mode==3);
    g.setTarget(20,162); a=g.decide(162,true);
    assert(a.mode==2 && a.target<110 && g.state==FingerState::Opening);
    for (uint32_t t=170;t<300;t+=10) {
        g.observe(sample(100,120),t); g.setTarget(20,t);
        a=g.decide(t,true); assert(a.mode==2 && a.target<=100 && g.desired==20);
    }
    // A 20 -> 150 replacement ends opening immediately, never returns to 400.
    g.setTarget(150,300); g.observe(sample(100,0),300);
    a=g.decide(300,true); assert(a.mode==5 && g.desired==150 && a.target<=110);
    // Lost feedback cannot leave force mode working autonomously.
    a=g.decide(401,true); assert(a.mode==0xFE && g.reason==FingerReason::FeedbackMissing);
    g.observe(sample(100),410); g.setTarget(150,410); assert(g.decide(410,true).mode==5);
    g.observe(sample(100),1000); a=g.decide(1000,true);
    assert(a.mode==0xFE && g.reason==FingerReason::CommandExpired);
    g.observe(sample(100),1001); assert(g.decide(1001,true).mode==0xFE);
    // Disabling an optional timeout also releases an existing expiry latch.
    auto disabled=g; disabled.limits.commandTimeoutMs=0;
    assert(disabled.decide(1001,true).mode==5 && disabled.desired==150);
    g.setTarget(20,1002); assert(g.decide(1002,true).mode==2);
    // Hard fault cannot be bypassed by latest target or by fresh feedback.
    g.observe(sample(100,450),1010); g.setTarget(20,1011);
    assert(g.decide(1011,true).mode==0xFD && !g.clearLatch());
    g.observe(sample(100,0),1020); assert(g.decide(1020,true).mode==0xFD);
    assert(g.clearLatch()); assert(!g.hasTarget); assert(g.decide(1020,true).mode==0xFF);
    g.setTarget(200,1021); g.observe(sample(100,0,250),1021);
    g.observe(sample(100,0,250),1042); assert(g.reason==FingerReason::OverCurrent);
    // Negative sensor polarity uses a negative force-hold target.
    FingerGuard negative; negative.limits=benchLimits(); negative.limits.forceSign=-1;
    negative.observe(sample(100,-120),100); negative.setTarget(300,100);
    negative.observe(sample(100,-120),101); negative.observe(sample(100,-120),112);
    assert(negative.decide(112,true).forceTargetG==-150);
    // Timestamp wrap does not bypass deadline checks.
    FingerGuard wrap; wrap.limits=benchLimits(); wrap.observe(sample(100),0xFFFFFFF0);
    wrap.setTarget(200,0xFFFFFFF0); assert(wrap.decide(0x20,true).mode==5);
    assert(wrap.decide(0x100,true).mode==0xFE);
}
void busTests() {
    HardwareSerial uart; FingerGuardBus bus(uart);
    assert(bus.query(2,100));
    assert(uart.writes.back()==std::vector<uint8_t>({0x55,0xAA,3,2,0x30,0,0,0x35}));
    auto good=v2reply(2,0x30,0,0x345,-1234);
    // Noise, wrong ID/command/address, CRC failure and old 22-byte replies cannot pass.
    for (int kind=0;kind<4;++kind) {
        auto bad=good;
        if(kind==0)bad[3]=3; if(kind==1)bad[4]=0x32;
        if(kind==2)bad[5]=1;
        if(kind!=3)bad.back()=FingerGuardBus::checksum(bad.data(),19);
        else bad.back()^=1;
        for(auto b:bad)uart.input.push_back(b);
        assert(bus.poll(101,5)==FingerGuardBus::Result::None);
    }
    for(size_t i=0;i<good.size();++i) {
        uart.input.push_back(good[i]);
        auto result=bus.poll(101,5);
        assert(result==(i+1==good.size()?FingerGuardBus::Result::Good:FingerGuardBus::Result::None));
    }
    assert(bus.feedback.position==0x345 && bus.feedback.forceG==-1234);
    assert(bus.feedback.currentMa==0x456 && bus.feedback.forceAdc==0x789);
    assert(bus.feedback.temperatureC==32 && bus.feedback.errors==0);
    uint16_t words[5]={5,0,250,200,300};
    assert(bus.write(2,0x25,words,5,102));
    auto frame=uart.writes.back();
    assert(frame.size()==18 && frame[2]==13 && frame[4]==0x32 && frame[5]==0x25);
    assert(frame[7]==5 && frame[11]==250 && frame[15]==44 && frame[16]==1);
    assert(frame.back()==FingerGuardBus::checksum(frame.data(),17));
    assert(!bus.query(3,103)); assert(bus.poll(108,5)==FingerGuardBus::Result::Timeout);
}
void deployedTrialTests() {
    for(uint8_t id=1;id<=5;++id) {
        FingerGuard g; g.limits=fingerGuardLimits(id); assert(g.limits.valid());
        g.observe(sample(100),100);
        assert(g.decide(100,true).mode==0xFF); // Enable never creates a target.
        g.setTarget(500,100);
        auto action=g.decide(100,true);
        assert(action.mode==5 && action.forceTargetG==600);
        auto oneShot=g;
        int position=100;
        for(uint32_t t=120;t<=2300;t+=20) {
            oneShot.observe(sample(position),t);
            action=oneShot.decide(t,true);
            assert(action.mode==5);
            position=action.target;
        }
        assert(position==500 && oneShot.targetGeneration==1 && oneShot.targetAt==100);
        g.observe(sample(100,350),110); g.observe(sample(100,350),141);
        action=g.decide(141,true);
        assert(action.mode==3 && action.forceTargetG==400 && action.upper<=110);
        // One command keeps force holding beyond the old 500 ms expiry.
        auto held=g;
        for(uint32_t t=160;t<=2200;t+=20) {
            held.observe(sample(100,350),t);
            action=held.decide(t,true);
            assert(action.mode==3 && action.forceTargetG==400);
        }
        assert(held.targetGeneration==1 && held.desired==500);
        // Removing the object resumes the original target without a new command.
        held.observe(sample(100,0),2210); held.observe(sample(100,0),2271);
        action=held.decide(2271,true);
        assert(action.mode==5 && action.target<=106 && held.desired==500);
        // Disabling command expiry does not disable feedback freshness checks.
        assert(held.decide(2372,true).mode==0xFE);
        assert(held.reason==FingerReason::FeedbackMissing);
        auto currentFault=g;
        currentFault.observe(sample(100,350,1200),142);
        assert(currentFault.decide(142,true).mode==0xFD);
        assert(currentFault.reason==FingerReason::OverCurrent);
        auto forceFault=g;
        forceFault.observe(sample(100,1000),142);
        assert(forceFault.decide(142,true).mode==0xFD);
        assert(forceFault.reason==FingerReason::OverForce);
        g.setTarget(20,142);
        assert(g.decide(142,true).mode==2); // Latest opening cancels holding.
    }
}
void runtimeTests() {
    HardwareSerial uart; FingerGuardRuntime rt(uart);
    rt.begin(); uint8_t ids[5]={1,2,3,4,5}; int16_t goals[5]={300,300,300,300,300};
    // The deployed profile is enabled, but boot alone cannot send motion.
    assert(rt.configured() && !rt.palmsAllowed(100));
    rt.tick(100);
    assert(uart.writes.size()==1 && uart.writes[0][4]==0x30);
    for(const auto &f:rt.fingers) assert(!f.hasTarget);
    FingerLimits cfg[5]; for(auto &c:cfg)c=benchLimits();
    rt.begin(cfg,false);
    assert(!rt.configured() && !rt.accept(5,ids,goals,100));
    rt.begin(cfg,true);
    assert(rt.accept(5,ids,goals,100));
    uint32_t fakeNow=100;
    uart.onWrite=[&](const std::vector<uint8_t>& q) {
        auto p=v2reply(q[3],q[4],q[5]|(q[6]<<8),100,0);
        // current must remain below synthetic limits.
        p[11]=50;p[12]=0;p.back()=FingerGuardBus::checksum(p.data(),19);
        for(auto b:p)uart.input.push_back(b);
    };
    for (;fakeNow<180;++fakeNow) rt.tick(fakeNow);
    for(const auto &f:rt.fingers)assert(f.hasFeedback && f.state!=FingerState::Fault);
    // Scheduler writes only V2.0.4 commands, serialized with >=4ms slot spacing.
    for(const auto &q:uart.writes) assert(q[4]==0x30 || q[4]==0x32);
    bool motion=false;
    for(const auto &q:uart.writes) if(q[4]==0x32) {
        assert(q[5]!=0x20); // Preserve the actuator's native overcurrent setting.
        if(q[5]!=0x21) continue;
        motion=true; assert(q.size()==26); assert(q[15]==5); // reg 0x25 mode
        assert(q[19]==250); // reg 0x27 force limit
    }
    assert(motion);
    uint16_t regs[86]; rt.registers(regs,86,180);
    assert(regs[0]==0x4647 && regs[2]==0x1F03 && regs[3]==0x0204);
    assert(regs[8]==300 && regs[6+15]==1);
    const auto before=uart.writes.size();
    goals[0]=2000; assert(!rt.accept(5,ids,goals,180));
    assert(uart.writes.size()==before && rt.fingers[0].desired==300);
    // Valid other fingers do not overwrite the replaced F1 opening destination.
    int16_t open=20; assert(rt.accept(1,ids,&open,181));
    rt.tick(181); rt.tick(182); rt.tick(184); rt.tick(185);
    assert(rt.fingers[0].desired==20);
    // Offline diagnostics/targets never enqueue historical destination values.
    int16_t latest=40; assert(rt.accept(1,ids,&latest,186));
    latest=60; assert(rt.accept(1,ids,&latest,187));
    assert(rt.fingers[0].desired==60);
    // No replies: bounded timeout, fault latch and quarantined late RX.
    uart.onWrite=nullptr;
    for(uint32_t t=190;t<400;++t) rt.tick(t);
    bool fault=false; for(const auto &f:rt.fingers) fault|=f.state==FingerState::Fault;
    assert(fault);
    // One-finger commissioning cannot enable the other fingers or the palms.
    HardwareSerial singleUart; FingerGuardRuntime single(singleUart);
    FingerLimits isolated[5]; isolated[0]=benchLimits(); single.begin(isolated,true,1);
    assert(single.configured() && !single.palmsAllowed(100));
    int16_t goal=100; uint8_t first=1,second=2;
    assert(single.accept(1,&first,&goal,100)); assert(!single.accept(1,&second,&goal,100));
    single.registers(regs,86,100); assert(regs[2]==0x0103);
}
void oneShotRuntimeTests() {
    HardwareSerial uart; FingerGuardRuntime rt(uart); rt.begin();
    uint8_t ids[5]={1,2,3,4,5}; int16_t goals[5]={500,500,500,500,500};
    uart.onWrite=[&](const std::vector<uint8_t>& q) {
        auto reply=v2reply(q[3],q[4],q[5]|(q[6]<<8),100,350);
        reply[11]=50; reply[12]=0;
        reply.back()=FingerGuardBus::checksum(reply.data(),19);
        for(auto byte:reply) uart.input.push_back(byte);
    };
    // Let the normal boot polling obtain feedback before issuing the target;
    // otherwise an initial feedback-missing pause would be expected protection.
    for(uint32_t t=100;t<140;++t) rt.tick(t);
    assert(rt.accept(5,ids,goals,140)); // The only host command in this test.
    for(uint32_t t=140;t<=2300;++t) rt.tick(t);
    for(const auto &f:rt.fingers) {
        assert(f.state==FingerState::Holding && f.nativeMode==3);
        assert(f.desired==500 && f.targetAt==140 && f.targetGeneration==1);
    }
    for(const auto &q:uart.writes) {
        if(q[4]==0x32) assert(q[5]==0x21); // No command-age pause or stop.
    }
}
int main() {
    stateTests(); busTests(); deployedTrialTests(); runtimeTests(); oneShotRuntimeTests();
    std::cout<<"finger guard scenarios: OK\n";
}

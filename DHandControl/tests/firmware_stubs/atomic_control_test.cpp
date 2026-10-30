// Actual sketch + UART stubs. Python creates the protocol request arrays.
#include <cassert>
#include <iostream>
#include <iomanip>
#include "atomic_packets.h"

std::vector<uint8_t> responseBytes() {
    std::vector<uint8_t> result;
    for (const auto& write:RS485Serial.writes) result.insert(result.end(),write.begin(),write.end());
    return result;
}

void inject(const std::vector<uint8_t>& data) {
    for (auto value:data) RS485Serial.input.push_back(value);
    handleIncomingData();
}

void repairCRC(std::vector<uint8_t>& data) {
    uint16_t crc=calculateCRC(data.data(),data.size()-2);
    data[data.size()-2]=crc&0xff; data.back()=crc>>8;
}

void configureGuardForMemoryTest() {
    mh6::FingerLimits cfg[5];
    for (auto &c:cfg) {
        c.forceVerified=true; c.nativeModesAndLimitsVerified=true; c.forceSign=1; c.releaseForceG=50;
        c.contactForceG=100;c.holdForceG=150;c.limitForceG=250;c.dangerForceG=400;
        c.continuousCurrentMa=200;c.tripCurrentMa=500;c.maximumTemperatureC=60;c.outputLimit=100;
    }
    fingerProtection.begin(cfg,true);
    for(auto &f:fingerProtection.fingers) {
        mh6::FingerFeedback sample;sample.position=20;sample.temperatureC=25;
        f.observe(sample,millis());
    }
}
int main() {
    initializeHoldingRegisters();
    assert(holdingRegisters[50]==0x4D36 && holdingRegisters[51]==1 && holdingRegisters[52]==7);
    std::vector<uint8_t> ack(request_ack,request_ack+sizeof(request_ack));
    std::vector<uint8_t> one(request_oneway,request_oneway+sizeof(request_oneway));
    // Shipped firmware is motion-locked until real loads are configured.
    inject(ack);
    assert(holdingRegisters[REG_STATUS]==STATUS_ERR_GUARD_LOCKED);
    assert(Serial.writes.empty() && BusServoSerial.writes.empty());
    configureGuardForMemoryTest(); RS485Serial.writes.clear();
    // Fragment every byte. No target is published until all bytes and CRC arrive.
    for (size_t i=0;i+1<ack.size();++i) {
        inject({ack[i]});
        assert(Serial.writes.empty() && BusServoSerial.writes.empty());
    }
    inject({ack.back()});
    assert(Serial.writes.empty() && BusServoSerial.writes.size()==4);
    auto reply=responseBytes();
    assert(reply.size()==12 && reply[8]==0 && reply[9]==0x95);
    assert(calculateCRC(reply.data(),10)==uint16_t(reply[10]|reply[11]<<8));
    std::cout<<"ACK_HEX ";
    for (auto b:reply) std::cout<<std::hex<<std::setfill('0')<<std::setw(2)<<unsigned(b);
    std::cout<<std::dec<<"\n";
    assert(holdingRegisters[53]==0x1020 && holdingRegisters[54]==0x3040);
    assert(holdingRegisters[56]==1);
    RS485Serial.writes.clear();
    inject(one);
    assert(RS485Serial.writes.empty());
    assert(Serial.writes.empty() && BusServoSerial.writes.size()==8);

    // Reject any malformed final axis before any finger or palm can move.
    for (int kind=0;kind<7;++kind) {
        auto bad=ack;
        if (kind==0) bad[2]=2; // wrong protocol version
        if (kind==1) bad[9]=4; // wrong finger count
        if (kind==2) bad[8+2*21+1]=1; // duplicate last palm ID
        if (kind==3) { bad[8+2*22]=3; bad[8+2*22+1]=233; } // M4=1001
        if (kind==4) { bad[8+2*23]=0; bad[8+2*23+1]=0; } // zero move time
        if (kind==5) bad[3]=2; // unsupported flag, no reply
        repairCRC(bad);
        if (kind==6) bad.back()^=1; // wrong CRC, no reply
        auto fingerWrites=Serial.writes.size(), palmWrites=BusServoSerial.writes.size();
        std::vector<uint16_t> previousTargets(holdingRegisters+20,holdingRegisters+47);
        RS485Serial.writes.clear(); inject(bad);
        assert(Serial.writes.size()==fingerWrites && BusServoSerial.writes.size()==palmWrites);
        assert(std::vector<uint16_t>(holdingRegisters+20,holdingRegisters+47)==previousTargets);
        if (kind<5) assert(responseBytes().size()==12 && holdingRegisters[REG_STATUS]!=STATUS_GUARDED_ACCEPTED);
        else assert(RS485Serial.writes.empty());
    }
    // Even errors in one-way mode cannot leave an unread response behind.
    auto badOne=one; badOne[9]=4; repairCRC(badOne);
    RS485Serial.writes.clear(); inject(badOne);
    assert(RS485Serial.writes.empty() && holdingRegisters[55]==ATOMIC_ERR_COUNTS);
    // Consecutive complete frames are consumed separately, not concatenated.
    auto twice=one; twice.insert(twice.end(),one.begin(),one.end());
    auto before=fingerProtection.fingers[0].targetGeneration; inject(twice);
    assert(fingerProtection.fingers[0].targetGeneration==before+2 && RS485Serial.writes.empty());
    assert(fingerProtection.fingers[0].desired==100);
    // Legacy read remains compatible with startup capability probing.
    std::vector<uint8_t> query={1,3,0,50,0,3,0,0}; repairCRC(query);
    inject(query); assert(bufferIndex==8); processCompletePacket();
    reply=responseBytes();
    assert(reply.size()==11 && reply[1]==3 && reply[3]==0x4D && reply[4]==0x36);
    // Legacy single/group entries also replace targets, never send raw F2/21.
    auto fingerWrites=Serial.writes.size();
    holdingRegisters[REG_DEVICE_TYPE]=0; holdingRegisters[REG_DEVICE_ID]=1;
    holdingRegisters[REG_POSITION]=333; executeSingleControl();
    assert(fingerProtection.fingers[0].desired==333 && Serial.writes.size()==fingerWrites);
    holdingRegisters[REG_GROUP_COUNT]=2;
    holdingRegisters[10]=1; holdingRegisters[11]=444;
    holdingRegisters[12]=2; holdingRegisters[13]=555; executeGroupControl();
    assert(fingerProtection.fingers[0].desired==444 && fingerProtection.fingers[1].desired==555);
    assert(Serial.writes.size()==fingerWrites);
    holdingRegisters[REG_DEVICE_ID]=257;executeSingleControl();
    assert(holdingRegisters[REG_STATUS]==STATUS_ERR_INVALID_DEVICE_ID);
    assert(fingerProtection.fingers[0].desired==444);
    // A fault cannot be aggravated by continuing to close any of the palms.
    fingerProtection.fingers[0].communicationFault();
    auto palmsBefore=BusServoSerial.writes.size();
    inject(one);
    assert(BusServoSerial.writes.size()==palmsBefore);
    assert(fingerProtection.fingers[0].state==mh6::FingerState::Fault);
    // Diagnostic flags are generated from the controller, never client-writable.
    handleWriteSingleRegister(1,82,0x1F03);
    assert(holdingRegisters[REG_STATUS]==STATUS_ERR_INVALID_REGISTER);
    std::cout<<"atomic bridge: OK\n";
}

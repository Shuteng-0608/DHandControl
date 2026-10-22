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

int main() {
    initializeHoldingRegisters();
    assert(holdingRegisters[50]==0x4D36 && holdingRegisters[51]==1 && holdingRegisters[52]==3);
    std::vector<uint8_t> ack(request_ack,request_ack+sizeof(request_ack));
    std::vector<uint8_t> one(request_oneway,request_oneway+sizeof(request_oneway));
    // Fragment every byte. No target is published until all bytes and CRC arrive.
    for (size_t i=0;i+1<ack.size();++i) {
        inject({ack[i]});
        assert(Serial.writes.empty() && BusServoSerial.writes.empty());
    }
    inject({ack.back()});
    assert(Serial.writes.size()==1 && BusServoSerial.writes.size()==4);
    auto reply=responseBytes();
    assert(reply.size()==12 && reply[8]==0 && reply[9]==0x90);
    assert(calculateCRC(reply.data(),10)==uint16_t(reply[10]|reply[11]<<8));
    std::cout<<"ACK_HEX ";
    for (auto b:reply) std::cout<<std::hex<<std::setfill('0')<<std::setw(2)<<unsigned(b);
    std::cout<<std::dec<<"\n";
    assert(holdingRegisters[53]==0x1020 && holdingRegisters[54]==0x3040);
    assert(holdingRegisters[56]==1);
    RS485Serial.writes.clear();
    inject(one);
    assert(RS485Serial.writes.empty());
    assert(Serial.writes.size()==2 && BusServoSerial.writes.size()==8);

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
        if (kind<5) assert(responseBytes().size()==12 && holdingRegisters[REG_STATUS]!=0x90);
        else assert(RS485Serial.writes.empty());
    }
    // Even errors in one-way mode cannot leave an unread response behind.
    auto badOne=one; badOne[9]=4; repairCRC(badOne);
    RS485Serial.writes.clear(); inject(badOne);
    assert(RS485Serial.writes.empty() && holdingRegisters[55]==ATOMIC_ERR_COUNTS);
    // Consecutive complete frames are consumed separately, not concatenated.
    auto twice=one; twice.insert(twice.end(),one.begin(),one.end());
    auto before=Serial.writes.size(); inject(twice);
    assert(Serial.writes.size()==before+2 && RS485Serial.writes.empty());
    // Legacy read remains compatible with startup capability probing.
    std::vector<uint8_t> query={1,3,0,50,0,3,0,0}; repairCRC(query);
    inject(query); assert(bufferIndex==8); processCompletePacket();
    reply=responseBytes();
    assert(reply.size()==11 && reply[1]==3 && reply[3]==0x4D && reply[4]==0x36);
    std::cout<<"atomic bridge: OK\n";
}

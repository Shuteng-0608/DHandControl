#include "FingerGuardBus.h"

namespace mh6 {
uint8_t FingerGuardBus::checksum(const uint8_t *data, uint8_t count) {
    uint8_t sum=0; for (uint8_t i=2;i<count;++i) sum+=data[i]; return sum;
}
void FingerGuardBus::drain() {
    for (uint16_t i=0;i<128 && serial.available();++i) serial.read();
}
bool FingerGuardBus::send(uint8_t *data, uint8_t bytes, uint32_t now) {
    if (busy) return false;
    drain(); used=0;
    id=data[3]; command=data[4]; address=uint16_t(data[5])|(uint16_t(data[6])<<8);
    started=now; busy=true;
    data[bytes-1]=checksum(data,bytes-1);
    if (serial.write(data,bytes)!=bytes) { busy=false; return false; }
    return true;
}
bool FingerGuardBus::query(uint8_t deviceId, uint32_t now) {
    // Generic status frame in section 3.3.1: command plus 16-bit reserved address.
    uint8_t data[8]={0x55,0xAA,3,deviceId,0x30,0,0,0};
    return send(data,sizeof(data),now);
}
bool FingerGuardBus::write(uint8_t deviceId,uint16_t reg,const uint16_t *values,uint8_t count,uint32_t now) {
    if (!count || count>10) return false;
    uint8_t data[28]={0x55,0xAA,uint8_t(3+2*count),deviceId,0x32,uint8_t(reg),uint8_t(reg>>8)};
    for (uint8_t i=0;i<count;++i) { data[7+2*i]=values[i]; data[8+2*i]=values[i]>>8; }
    return send(data,uint8_t(8+2*count),now);
}
FingerGuardBus::Result FingerGuardBus::poll(uint32_t now,uint16_t timeoutMs) {
    if (!busy) return Result::None;
    uint16_t budget=128;
    while (budget-- && serial.available()) {
        const uint8_t value=serial.read();
        if (!used) { if (value==0xAA) rx[used++]=value; continue; }
        if (used==1 && value!=0x55) { used=value==0xAA ? 1:0; continue; }
        rx[used++]=value;
        if (used==3 && rx[2]!=15) { used=0; continue; }
        if (used<sizeof(rx)) continue;
        used=0;
        if (rx[3]!=id || rx[4]!=command || rx[5]!=uint8_t(address) ||
            rx[6]!=uint8_t(address>>8) || rx[19]!=checksum(rx,19)) continue;
        auto word=[&](uint8_t i){ return uint16_t(rx[i])|(uint16_t(rx[i+1])<<8); };
        feedback.target=int16_t(word(7)); feedback.position=int16_t(word(9));
        feedback.currentMa=word(11); feedback.forceG=int16_t(word(13));
        feedback.forceAdc=word(15); feedback.temperatureC=int8_t(rx[17]);
        feedback.errors=rx[18];
        // 15 data bytes = command(1), address(2), status(12); checksum is B19.
        busy=false; return Result::Good;
    }
    if (uint32_t(now-started)>=timeoutMs) { busy=false; used=0; return Result::Timeout; }
    return Result::None;
}
}

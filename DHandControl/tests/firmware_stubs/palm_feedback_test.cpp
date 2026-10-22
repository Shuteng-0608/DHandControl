// Included after the Arduino sketch by test_mh6_firmware_feedback.py.
#include <cassert>
#include <iostream>

std::vector<byte> response(byte id, byte cmd, uint16_t value) {
    byte width = cmd == LOBOT_SERVO_TEMP_READ ? 1 : 2;
    std::vector<byte> p = {0x55,0x55,id,(byte)(width+3),cmd,(byte)value};
    if (width == 2) p.push_back(value >> 8);
    unsigned sum = 0;
    for (size_t i=2;i<p.size();++i) sum += p[i];
    p.push_back((byte)~sum);
    return p;
}

void installResponder(HardwareSerial& uart,int mode=0) {
    uart.input.clear(); uart.readyAt = 0; uart.byteSpacing = 150;
    uart.onWrite = [&uart,mode](const std::vector<byte>& request) {
        if (mode == 5) return; // no response
        auto p = response(request[2],request[4],request[4] == 26 ? 32 : request[4] == 27 ? 7400 : 614);
        if (mode == 1) p[2] = 3;          // wrong ID, even with correct checksum
        if (mode == 2) p[4] = 14;         // wrong command
        if (mode == 3) p.back() ^= 1;     // corrupted checksum
        if (mode == 4) p[3] = 255;        // malicious length
        if (mode == 1 || mode == 2) {
            unsigned sum = 0;
            for (size_t i=2;i+1<p.size();++i) sum += p[i];
            p.back() = (byte)~sum;
        }
        for (byte b:p) uart.input.push_back(b);
        uart.readyAt = hostMicros+100;
    };
}

int main() {
    HardwareSerial uart;
    LobotSerialServoControl driver(uart,13,14);
    uint16_t value = 0;
    installResponder(uart);
    assert(driver.readFeedbackValue(4,LOBOT_SERVO_POS_READ,&value));
    assert(value == 614);
    assert(driver.readFeedbackValue(4,LOBOT_SERVO_TEMP_READ,&value) && value == 32);
    assert(driver.readFeedbackValue(4,LOBOT_SERVO_VIN_READ,&value) && value == 7400);
    for (int mode=1;mode<=5;++mode) {
        installResponder(uart,mode);
        uint32_t started = millis();
        assert(!driver.readFeedbackValue(4,LOBOT_SERVO_POS_READ,&value));
        assert((uint32_t)(millis()-started) <= 11);
    }
    // Stale buffered data cannot satisfy a new query.
    installResponder(uart,5); uart.byteSpacing = 0;
    auto stale = response(4,28,222);
    for (byte b:stale) uart.input.push_back(b);
    assert(!driver.readFeedbackValue(4,28,&value));
    auto before = uart.writes.size();
    assert(!driver.readFeedbackValue(4,LOBOT_SERVO_MOVE_TIME_WRITE,&value));
    assert(uart.writes.size() == before);

    // Exercise the actual sketch command/register bridge with the real parser.
    installResponder(BusServoSerial);
    holdingRegisters[REG_DEVICE_TYPE] = 1;
    holdingRegisters[REG_DEVICE_ID] = 4;
    handleCommandExecution(CMD_READ_PALM_STATUS);
    assert(holdingRegisters[REG_STATUS] == 0x94);
    assert(holdingRegisters[72] == 4 && holdingRegisters[73] == 1);
    assert(holdingRegisters[74] == 614 && holdingRegisters[75] == 32);
    assert(holdingRegisters[76] == 7400 && holdingRegisters[77] == 0);
    assert(holdingRegisters[78] == 1);
    auto sequence = holdingRegisters[79];
    installResponder(BusServoSerial,5);
    handleCommandExecution(CMD_READ_PALM_STATUS);
    assert(holdingRegisters[REG_STATUS] == 0xF6);
    assert(holdingRegisters[73] == 0 && holdingRegisters[74] == 0);
    assert(holdingRegisters[78] == 0 && holdingRegisters[79] != sequence);
    holdingRegisters[REG_DEVICE_TYPE] = 0;
    auto writesBefore = BusServoSerial.writes.size();
    handleCommandExecution(CMD_READ_PALM_STATUS);
    assert(holdingRegisters[REG_STATUS] == STATUS_ERR_INVALID_DEVICE_TYPE);
    assert(BusServoSerial.writes.size() == writesBefore);
    std::cout << "fragmented UART, checksum/ID/command/length rejection, bounded timeout, bridge registers: OK\n";
}

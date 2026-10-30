#include <HardwareSerial.h>
#include "FingerGuardRuntime.h"
#include "LobotSerialServoControl.h"

// ==== 硬件配置 - 根据原理图定义 ====
#define RS485_SERIAL_RX 16   // GPIO16 - RS485接收
#define RS485_SERIAL_TX 17   // GPIO17 - RS485发送
#define RS485_CTRL_PIN 4     // RS485方向控制引脚（GPIO4）
#define MODBUS_BAUDRATE 115200
#define MODBUS_SLAVE_ID 1
// Atomic streaming protocol v1: 64-byte command; optional 12-byte result.
// Both experiment branches use this same firmware. Legacy Modbus is retained.
#define ATOMIC_FUNCTION 0x41
#define ATOMIC_VERSION 1
#define ATOMIC_REQUEST_BYTES 64
#define REG_ATOMIC_CAPABILITIES 50
#define ATOMIC_ERR_HEADER 0x0101
#define ATOMIC_ERR_COUNTS 0x0102
#define ATOMIC_ERR_IDS 0x0103
#define ATOMIC_ERR_POSITION 0x0104
#define ATOMIC_ERR_TIME 0x0105
// Detect Modbus RTU inter-frame silence. 100 ms is too slow for teleoperation;
// 2 ms is conservative for 115200 baud and still separates complete frames.
#define MODBUS_FRAME_TIMEOUT_MS 5
#define MAX_GROUP_DEVICES 5

#define FINGER_UART_BAUDRATE 921600
#define FINGER_UART_RX_PIN 3
// Project wiring confirms GPIO35 as finger UART TX. On standard ESP32 modules,
// GPIO35 is input-only, so verify the actual board variant if TX does not work.
#define FINGER_UART_TX_PIN 1

#define SERVO_SERIAL_RX   18
#define SERVO_SERIAL_TX   19
#define receiveEnablePin  13
#define transmitEnablePin 14

#define CMD_COMBINED_CONTROL 0x04
#define CMD_READ_DEVICE_ID 0x05
#define CMD_SET_DEVICE_ID  0x06
#define CMD_READ_FINGER_STATUS 0x07
#define CMD_READ_PALM_STATUS 0x08


#define FINGER_QUERY_ERROR_NONE       0
#define FINGER_QUERY_ERROR_TIMEOUT    1
#define FINGER_QUERY_ERROR_FORMAT     2
#define FINGER_QUERY_ERROR_CHECKSUM   3

// 调试串口使用Serial0（USB）
// #define  DEBUG_SERIAL Serial
// 电缸串口
mh6::FingerGuardRuntime fingerProtection(Serial);

// 舵机串口
HardwareSerial BusServoSerial(1);
LobotSerialServoControl BusServo(BusServoSerial,receiveEnablePin,transmitEnablePin);


// RS485串口使用Serial2
HardwareSerial RS485Serial(2);

// 数据接收缓冲区
uint8_t receiveBuffer[256];
uint16_t bufferIndex = 0;
uint32_t lastReceiveTime = 0;

// 保持寄存器数组 - 扩大范围以覆盖所有可能的寄存器地址
#define HOLDING_REGISTERS_SIZE 166
#define REG_FINGER_GUARD_BASE 80
#define STATUS_GUARDED_ACCEPTED 0x95
#define STATUS_ERR_GUARD_LOCKED 0xF7
uint16_t holdingRegisters[HOLDING_REGISTERS_SIZE] = {0};

// 寄存器地址映射（根据你的Modbus协议定义）
#define REG_COMMAND       0   // 命令寄存器
#define REG_DEVICE_TYPE   1   // 设备类型
#define REG_DEVICE_ID     2   // 设备ID
#define REG_POSITION      3   // 目标位置
#define REG_EXEC_TIME     4   // 执行时间
#define REG_STATUS        5   // 状态寄存器
#define REG_GROUP_COUNT   6   // 组控数量
#define REG_NEW_DEVICE_ID 7
#define REG_ID_RESULT     8
#define REG_ID_SAVE       9
#define REG_GROUP_START   10  // 组控数据起始地址
#define REG_HAND_FINGER_COUNT 20
#define REG_HAND_FINGER_START 21
#define REG_HAND_PALM_COUNT   31
#define REG_HAND_PALM_START   32
#define REG_FINGER_STATUS_BASE  60
#define REG_FINGER_STATUS_COUNT 12
#define REG_PALM_STATUS_BASE 72
#define REG_PALM_STATUS_COUNT 8

#define STATUS_COMBINED_ISSUED 0x90
#define STATUS_ID_READ_OK 0x91
#define STATUS_ID_SET_OK  0x92
#define STATUS_FINGER_STATUS_OK 0x93
#define STATUS_PALM_STATUS_OK 0x94
#define STATUS_ERR_PALM_STATUS 0xF6
#define STATUS_CLEAR_ERROR_OK 0xF0
#define STATUS_ERR_INVALID_DEVICE_TYPE 0xE1
#define STATUS_ERR_CLEAR_ERROR_UNSUPPORTED 0xE5
#define STATUS_ERR_INVALID_GROUP_COUNT 0xE3
#define STATUS_ERR_INVALID_REGISTER    0xE6
#define STATUS_ERR_UNSUPPORTED_FUNCTION 0xE7
#define STATUS_ERR_INVALID_COMBINED_FINGER_COUNT 0xE8
#define STATUS_ERR_INVALID_COMBINED_PALM_COUNT   0xE9
#define STATUS_ERR_COMBINED_REGISTER_RANGE       0xEA
#define STATUS_ERR_COMBINED_EMPTY                0xEB
#define STATUS_ERR_ID_READ_FAILED 0xEC
#define STATUS_ERR_ID_SET_FAILED  0xED
#define STATUS_ERR_INVALID_DEVICE_ID 0xEE
#define STATUS_ERR_ID_UNSUPPORTED    0xEF
#define STATUS_ERR_FINGER_STATUS_TIMEOUT  0xF1
#define STATUS_ERR_FINGER_STATUS_FORMAT   0xF2
#define STATUS_ERR_FINGER_STATUS_CHECKSUM 0xF3
#define STATUS_ERR_CLEAR_ERROR_VERIFY_FAILED    0xF4
#define STATUS_ERR_CLEAR_ERROR_REMAINING_FAULT  0xF5

#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION 0x01
#define MODBUS_EXCEPTION_ILLEGAL_ADDRESS  0x02
#define MODBUS_EXCEPTION_ILLEGAL_VALUE    0x03

struct FingerActuatorStatus {
    uint8_t actuatorId;
    uint16_t targetPosition;
    int16_t currentPosition;
    int8_t temperatureC;
    uint16_t currentMa;
    int16_t forceG;
    uint8_t errorFlags;
    uint16_t internal1;
    uint16_t internal2;
    uint8_t responseErrorCode;
    bool checksumOk;
};

bool isRegisterAddressValid(uint16_t regAddress) {
    return regAddress < HOLDING_REGISTERS_SIZE;
}

bool isRegisterRangeValid(uint16_t startAddr, uint16_t quantity) {
    return quantity > 0 &&
           startAddr < HOLDING_REGISTERS_SIZE &&
           quantity <= (HOLDING_REGISTERS_SIZE - startAddr);
}

bool isCombinedSectionRangeValid(uint16_t startAddr, uint16_t count, uint8_t registersPerDevice) {
    if (count == 0) {
        return true;
    }

    uint16_t lastAddr = startAddr + count * registersPerDevice - 1;
    return lastAddr < HOLDING_REGISTERS_SIZE;
}

bool isConfigDeviceIdValid(uint16_t deviceId) {
    return deviceId >= 1 && deviceId <= 253;
}

// The UART owner samples V2.0.4 continuously; host queries only read its cache.
bool queryFingerActuatorStatus(uint8_t id, FingerActuatorStatus *out, uint16_t timeoutMs) {
    (void)timeoutMs;
    out->actuatorId=id; out->checksumOk=false;
    out->responseErrorCode=FINGER_QUERY_ERROR_TIMEOUT;
    if (id<1 || id>5) return false;
    const auto &f=fingerProtection.fingers[id-1];
    if (!f.hasFeedback || uint32_t(millis()-f.feedbackAt)>f.limits.feedbackTimeoutMs) return false;
    out->targetPosition=f.feedback.target; out->currentPosition=f.feedback.position;
    out->temperatureC=f.feedback.temperatureC; out->currentMa=f.feedback.currentMa;
    out->forceG=f.feedback.forceG; out->errorFlags=f.feedback.errors;
    out->internal1=f.feedback.forceAdc; out->internal2=0;
    out->responseErrorCode=FINGER_QUERY_ERROR_NONE; out->checksumOk=true;
    return true;
}

void clearFingerStatusRegisters(uint8_t actuatorId) {
    holdingRegisters[REG_FINGER_STATUS_BASE] = actuatorId;
    for (uint16_t i = 1; i < REG_FINGER_STATUS_COUNT; i++) {
        holdingRegisters[REG_FINGER_STATUS_BASE + i] = 0;
    }
}

void writeFingerStatusRegisters(const FingerActuatorStatus &status) {
    holdingRegisters[REG_FINGER_STATUS_BASE + 0] = status.actuatorId;
    holdingRegisters[REG_FINGER_STATUS_BASE + 1] =
        (status.responseErrorCode == FINGER_QUERY_ERROR_NONE && status.checksumOk) ? 1 : 0;
    holdingRegisters[REG_FINGER_STATUS_BASE + 2] = status.targetPosition;
    holdingRegisters[REG_FINGER_STATUS_BASE + 3] = (uint16_t)status.currentPosition;
    holdingRegisters[REG_FINGER_STATUS_BASE + 4] = (uint16_t)(int16_t)status.temperatureC;
    holdingRegisters[REG_FINGER_STATUS_BASE + 5] = status.currentMa;
    holdingRegisters[REG_FINGER_STATUS_BASE + 6] = (uint16_t)status.forceG;
    holdingRegisters[REG_FINGER_STATUS_BASE + 7] = status.errorFlags;
    holdingRegisters[REG_FINGER_STATUS_BASE + 8] = status.internal1;
    holdingRegisters[REG_FINGER_STATUS_BASE + 9] = status.internal2;
    holdingRegisters[REG_FINGER_STATUS_BASE + 10] = status.responseErrorCode;
    holdingRegisters[REG_FINGER_STATUS_BASE + 11] = status.checksumOk ? 1 : 0;
}

void debugInvalidGroupCount(uint16_t groupCount) {
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("Invalid group_count: ");
    DEBUG_SERIAL.println(groupCount);
#endif
}

void debugInvalidRegister(uint16_t regAddress) {
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("Invalid register address: ");
    DEBUG_SERIAL.println(regAddress);
#endif
}

void debugInvalidQuantity(uint16_t quantity) {
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("Invalid register quantity: ");
    DEBUG_SERIAL.println(quantity);
#endif
}

void debugInvalidByteCount(uint8_t byteCount, uint16_t quantity) {
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("Invalid byte count: ");
    DEBUG_SERIAL.print(byteCount);
    DEBUG_SERIAL.print(", quantity=");
    DEBUG_SERIAL.println(quantity);
#endif
}

void debugUnsupportedFunction(uint8_t functionCode) {
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("Unsupported Modbus function: 0x");
    DEBUG_SERIAL.println(functionCode, HEX);
#endif
}

void debugInvalidCombinedCount(const char *sectionName, uint16_t count) {
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("Invalid combined ");
    DEBUG_SERIAL.print(sectionName);
    DEBUG_SERIAL.print(" count: ");
    DEBUG_SERIAL.println(count);
#endif
}

void debugCombinedEmpty() {
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Invalid combined command: empty finger and palm sections");
#endif
}

void debugCombinedRegisterRange() {
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Invalid combined command: payload register range");
#endif
}

void debugInvalidDeviceId(const char *label, uint16_t deviceId) {
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("Invalid ");
    DEBUG_SERIAL.print(label);
    DEBUG_SERIAL.print(" ID: ");
    DEBUG_SERIAL.println(deviceId);
#endif
}

void debugUnsupportedIdOperation(uint8_t devType) {
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("Unsupported ID operation for device_type: ");
    DEBUG_SERIAL.println(devType);
#endif
}

void debugInvalidDeviceType(uint8_t devType) {
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("Invalid device_type: ");
    DEBUG_SERIAL.println(devType);
#endif
}

void debugUnsupportedClearError(uint8_t devType) {
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("Unsupported clear error for device_type: ");
    DEBUG_SERIAL.println(devType);
#endif
}

void debugIdReadFailed(uint8_t devType, uint16_t queryId) {
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("ID read failed, device_type=");
    DEBUG_SERIAL.print(devType);
    DEBUG_SERIAL.print(", query_id=");
    DEBUG_SERIAL.println(queryId);
#endif
}

void debugIdSetFailed(uint8_t devType, uint16_t oldId, uint16_t newId, int readbackId) {
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("ID set failed, device_type=");
    DEBUG_SERIAL.print(devType);
    DEBUG_SERIAL.print(", old_id=");
    DEBUG_SERIAL.print(oldId);
    DEBUG_SERIAL.print(", new_id=");
    DEBUG_SERIAL.print(newId);
    DEBUG_SERIAL.print(", readback=");
    DEBUG_SERIAL.println(readbackId);
#endif
}

void setup() {
    // 初始化调试串口（USB）
    // DEBUG_SERIAL.begin(115200);
    // DEBUG_SERIAL.println("=== ESP32 RS485 Modbus从站 ===");
    // DEBUG_SERIAL.println("系统启动中...");

    // 初始化舵机串口
    BusServoSerial.begin(115200, SERIAL_8N1, SERVO_SERIAL_RX, SERVO_SERIAL_TX);
    BusServo.OnInit();           // 初始化总线舵机库

    // 初始化电缸
    Serial.begin(FINGER_UART_BAUDRATE,SERIAL_8N1,FINGER_UART_RX_PIN,FINGER_UART_TX_PIN);
    
    // 初始化RS485方向控制引脚
    pinMode(RS485_CTRL_PIN, OUTPUT);
    digitalWrite(RS485_CTRL_PIN, LOW); // 设置为接收模式
    // DEBUG_SERIAL.println("RS485方向控制引脚初始化完成");
    
    // 初始化RS485串口
    RS485Serial.begin(MODBUS_BAUDRATE, SERIAL_8E1, RS485_SERIAL_RX, RS485_SERIAL_TX);
    // DEBUG_SERIAL.println("RS485串口初始化完成");
    // DEBUG_SERIAL.print("波特率: "); 
    // DEBUG_SERIAL.println(MODBUS_BAUDRATE);
    
    fingerProtection.begin();
    // 初始化保持寄存器默认值
    initializeHoldingRegisters();
    
    // 清空接收缓冲区
    while (RS485Serial.available()) {
        RS485Serial.read();
    }
    
    // DEBUG_SERIAL.println("=== 开始监听Modbus命令 ===");
    // DEBUG_SERIAL.println("等待接收数据...");
    // DEBUG_SERIAL.println("=====================================");
}

void loop() {
    fingerProtection.tick(millis());
    if (RS485Serial.available()) {
        handleIncomingData();
    }
    
    if (bufferIndex > 0 && (millis() - lastReceiveTime > MODBUS_FRAME_TIMEOUT_MS)) {
        processCompletePacket();
    }
    
    static uint32_t lastStatusTime = 0;
    if (millis() - lastStatusTime > 5000) {
        lastStatusTime = millis();
        // DEBUG_SERIAL.print("状态: 监听中... 最后状态: 0x");
        // DEBUG_SERIAL.println(holdingRegisters[REG_STATUS], HEX);
    }
    
    fingerProtection.tick(millis());
    delay(1);
}

// 初始化保持寄存器默认值
void initializeHoldingRegisters() {
    // 设置默认状态
    holdingRegisters[REG_STATUS] = 0xA0; // 默认空闲状态
    holdingRegisters[REG_ATOMIC_CAPABILITIES] = 0x4D36;
    holdingRegisters[REG_ATOMIC_CAPABILITIES+1] = ATOMIC_VERSION;
    holdingRegisters[REG_ATOMIC_CAPABILITIES+2] = 7; // ACK, one-way, V2 finger guard
    
    // DEBUG_SERIAL.println("保持寄存器初始化完成");
    printRegisterMap();
}

// 打印当前寄存器映射表
void printRegisterMap() {
    // DEBUG_SERIAL.println("当前寄存器状态:");
    for (int i = 0; i < HOLDING_REGISTERS_SIZE; i++) {
        if (holdingRegisters[i] != 0) { // 只显示非零寄存器
            // DEBUG_SERIAL.print("寄存器[");
            // DEBUG_SERIAL.print(i);
            // DEBUG_SERIAL.print("] = 0x");
            // DEBUG_SERIAL.print(holdingRegisters[i], HEX);
            // DEBUG_SERIAL.print(" (");
            // DEBUG_SERIAL.print(holdingRegisters[i]);
            // DEBUG_SERIAL.println(")");
        }
    }
}

// 处理写单个寄存器请求
void handleWriteSingleRegister(uint8_t slaveAddress, uint16_t regAddress, uint16_t regValue) {
    // DEBUG_SERIAL.print("处理写单个寄存器: 地址=");
    // DEBUG_SERIAL.print(regAddress);
    // DEBUG_SERIAL.print(", 值=0x");
    // DEBUG_SERIAL.print(regValue, HEX);
    // DEBUG_SERIAL.print(" (");
    // DEBUG_SERIAL.print(regValue);
    // DEBUG_SERIAL.println(")");
    
    // 验证寄存器地址范围
    if (!isRegisterAddressValid(regAddress) || regAddress>=50) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_INVALID_REGISTER;
        debugInvalidRegister(regAddress);
        sendErrorResponse(slaveAddress, 0x06, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
        return;
    }
    
    // 更新保持寄存器
    holdingRegisters[regAddress] = regValue;
    
    // 特殊处理命令寄存器（地址0）
    if (regAddress == REG_COMMAND) {
        handleCommandExecution(regValue);
    }
    
    // 发送成功响应
    sendWriteRegisterResponse(slaveAddress, regAddress, regValue);
    
    // 打印更新后的寄存器状态
    printRegisterMap();
}

// 处理写多个寄存器请求
void handleWriteMultipleRegisters(uint8_t slaveAddress, uint16_t startAddr, uint16_t quantity, uint8_t byteCount) {
    if (quantity == 0 || quantity > HOLDING_REGISTERS_SIZE) {
        debugInvalidQuantity(quantity);
        sendErrorResponse(slaveAddress, 0x10, MODBUS_EXCEPTION_ILLEGAL_VALUE);
        return;
    }

    if (byteCount != quantity * 2) {
        debugInvalidByteCount(byteCount, quantity);
        sendErrorResponse(slaveAddress, 0x10, MODBUS_EXCEPTION_ILLEGAL_VALUE);
        return;
    }

    if (!isRegisterRangeValid(startAddr, quantity) || startAddr+quantity>50) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_INVALID_REGISTER;
        debugInvalidRegister(startAddr);
        sendErrorResponse(slaveAddress, 0x10, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
        return;
    }

    uint16_t expectedLength = 7 + byteCount + 2;
    if (bufferIndex < expectedLength) {
        debugInvalidByteCount(byteCount, quantity);
        sendErrorResponse(slaveAddress, 0x10, MODBUS_EXCEPTION_ILLEGAL_VALUE);
        return;
    }

    bool commandWritten = false;
    uint16_t commandValue = 0;

    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t regAddress = startAddr + i;
        uint16_t regValue = (receiveBuffer[7 + i * 2] << 8) | receiveBuffer[8 + i * 2];
        holdingRegisters[regAddress] = regValue;

        if (regAddress == REG_COMMAND && regValue != 0) {
            commandWritten = true;
            commandValue = regValue;
        }
    }

    if (commandWritten) {
        handleCommandExecution(commandValue);
    }

    sendWriteMultipleRegistersResponse(slaveAddress, startAddr, quantity);
    printRegisterMap();
}

// 处理命令执行
void handleCommandExecution(uint16_t command) {
    // DEBUG_SERIAL.print("执行命令: 0x");
    // DEBUG_SERIAL.println(command, HEX);
    
    // 根据命令类型设置状态
    switch (command) {
        case 0x01: // 单设备控制
            holdingRegisters[REG_STATUS] = (holdingRegisters[REG_DEVICE_TYPE] == 0) ? 0xA0 : 0xB0;
            executeSingleControl();
            break;
        case 0x02: // 组控
            holdingRegisters[REG_STATUS] = (holdingRegisters[REG_DEVICE_TYPE] == 0) ? 0xC0 : 0xD0;
            executeGroupControl();
            break;
        case 0x03: // 清除错误
            executeClearError();
            break;
        case CMD_COMBINED_CONTROL:
            executeCombinedControl();
            break;
        case CMD_READ_DEVICE_ID:
            executeReadDeviceID();
            break;
        case CMD_SET_DEVICE_ID:
            executeSetDeviceID();
            break;
        case CMD_READ_FINGER_STATUS:
            handleReadFingerStatusCommand();
            break;
        case CMD_READ_PALM_STATUS:
            handleReadPalmStatusCommand();
            break;
        default:
            holdingRegisters[REG_STATUS] = 0xE0; // 无效命令
            // DEBUG_SERIAL.println("错误: 无效命令");
            break;
    }
}

// 执行单设备控制
void executeSingleControl() {
    uint8_t devType = holdingRegisters[REG_DEVICE_TYPE];
    uint8_t devId = holdingRegisters[REG_DEVICE_ID];
    uint16_t position = holdingRegisters[REG_POSITION];
    uint16_t execTime = holdingRegisters[REG_EXEC_TIME];
    const uint16_t rawId=holdingRegisters[REG_DEVICE_ID];
    if (devType>1 || rawId<1 || rawId>(devType==0 ? 5:4)) {
        holdingRegisters[REG_STATUS]=STATUS_ERR_INVALID_DEVICE_ID; return;
    }
    
    // DEBUG_SERIAL.println("=== 执行单设备控制 ===");
    // DEBUG_SERIAL.print("设备类型: ");
    // DEBUG_SERIAL.print(devType);
    // DEBUG_SERIAL.println(devType == 0 ? "电缸" : "舵机");
    // DEBUG_SERIAL.print("设备ID: ");
    // DEBUG_SERIAL.println(devId);
    // DEBUG_SERIAL.print("目标位置: ");
    // DEBUG_SERIAL.println(position);
    // DEBUG_SERIAL.print("执行时间: ");
    // DEBUG_SERIAL.println(execTime);
    // DEBUG_SERIAL.println("=========================");
    
    if (!fingerProtection.configured()) { holdingRegisters[REG_STATUS]=STATUS_ERR_GUARD_LOCKED; return; }
    if (devType == 1) { // 舵机单控
        if (!fingerProtection.palmsAllowed(millis())) { holdingRegisters[REG_STATUS]=STATUS_ERR_GUARD_LOCKED; return; }
        BusServo.LobotSerialServoMove(devId, position, execTime);
    } 
    else if (devType == 0) { // 电缸单控
        const int16_t target=position;
        if (!fingerProtection.accept(1,&devId,&target,millis())) holdingRegisters[REG_STATUS]=STATUS_ERR_GUARD_LOCKED;
    }
}

// 执行组控
void executeGroupControl() {
    uint8_t devType = holdingRegisters[REG_DEVICE_TYPE];
    uint16_t groupCount = holdingRegisters[REG_GROUP_COUNT];

    if (groupCount == 0 || groupCount > MAX_GROUP_DEVICES) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_INVALID_GROUP_COUNT;
        debugInvalidGroupCount(groupCount);
        return;
    }
    if (devType>1 || (devType==1 && groupCount>4)) {
        holdingRegisters[REG_STATUS]=STATUS_ERR_INVALID_GROUP_COUNT; return;
    }

    // 准备数组存储组控参数
    uint8_t idArray[MAX_GROUP_DEVICES] = {0};
    int16_t posArray[MAX_GROUP_DEVICES] = {0};
    uint16_t timeArray[MAX_GROUP_DEVICES] = {0};
    
    // DEBUG_SERIAL.println("=== 执行组控 ===");
    // DEBUG_SERIAL.print("设备类型: ");
    // DEBUG_SERIAL.println(devType == 0 ? "电缸" : "舵机");
    // DEBUG_SERIAL.print("设备数量: ");
    // DEBUG_SERIAL.println(groupCount);
    
    for (int i = 0; i < groupCount; i++) {
        uint16_t baseAddr = REG_GROUP_START + i * (devType == 0 ? 2 : 3);
        const uint16_t rawId=holdingRegisters[baseAddr];
        if (rawId<1 || rawId>(devType==0 ? 5:4)) {
            holdingRegisters[REG_STATUS]=STATUS_ERR_INVALID_DEVICE_ID; return;
        }
        uint8_t devId = holdingRegisters[baseAddr];
        uint16_t position = holdingRegisters[baseAddr + 1];
        idArray[i] = holdingRegisters[baseAddr];
        posArray[i] = holdingRegisters[baseAddr + 1];
        
        // DEBUG_SERIAL.print("设备");
        // DEBUG_SERIAL.print(i + 1);
        // DEBUG_SERIAL.print(": ID=");
        // DEBUG_SERIAL.print(devId);
        // DEBUG_SERIAL.print(", 位置=");
        // DEBUG_SERIAL.println(position);
        
        if (holdingRegisters[REG_DEVICE_TYPE] == 1) { // 舵机有时间参数
            uint16_t execTime = holdingRegisters[baseAddr + 2];
            timeArray[i] = holdingRegisters[baseAddr + 2];
            // DEBUG_SERIAL.print("执行时间=");
            // DEBUG_SERIAL.println(execTime);
        }
    }
    // DEBUG_SERIAL.println("===================");
    if (!fingerProtection.configured()) { holdingRegisters[REG_STATUS]=STATUS_ERR_GUARD_LOCKED; return; }
    if (devType == 0) { // 电缸组控
        if (!fingerProtection.accept(groupCount,idArray,posArray,millis())) holdingRegisters[REG_STATUS]=STATUS_ERR_GUARD_LOCKED;
    } 
    else if (devType == 1) { // 舵机组控
        if (!fingerProtection.palmsAllowed(millis())) { holdingRegisters[REG_STATUS]=STATUS_ERR_GUARD_LOCKED; return; }
        for (int j = 0; j < groupCount; j++) {
            BusServo.LobotSerialServoMove(idArray[j], posArray[j], timeArray[j]);
        }
        
    }
}

// 执行手指 + 手掌组合控制
void executeCombinedControl() {
    uint16_t fingerCount = holdingRegisters[REG_HAND_FINGER_COUNT];
    uint16_t palmCount = holdingRegisters[REG_HAND_PALM_COUNT];

    if (fingerCount > MAX_GROUP_DEVICES) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_INVALID_COMBINED_FINGER_COUNT;
        debugInvalidCombinedCount("finger", fingerCount);
        return;
    }

    if (palmCount > MAX_GROUP_DEVICES) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_INVALID_COMBINED_PALM_COUNT;
        debugInvalidCombinedCount("palm", palmCount);
        return;
    }

    if (fingerCount == 0 && palmCount == 0) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_COMBINED_EMPTY;
        debugCombinedEmpty();
        return;
    }

    if (!isCombinedSectionRangeValid(REG_HAND_FINGER_START, fingerCount, 2) ||
        !isCombinedSectionRangeValid(REG_HAND_PALM_START, palmCount, 3)) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_COMBINED_REGISTER_RANGE;
        debugCombinedRegisterRange();
        return;
    }

    uint8_t fingerIds[MAX_GROUP_DEVICES] = {0};
    int16_t fingerPositions[MAX_GROUP_DEVICES] = {0};
    uint8_t palmIds[MAX_GROUP_DEVICES] = {0};
    int16_t palmPositions[MAX_GROUP_DEVICES] = {0};
    uint16_t palmTimes[MAX_GROUP_DEVICES] = {0};

    for (int i = 0; i < fingerCount; i++) {
        uint16_t baseAddr = REG_HAND_FINGER_START + i * 2;
        if (holdingRegisters[baseAddr]<1 || holdingRegisters[baseAddr]>5) {
            holdingRegisters[REG_STATUS]=STATUS_ERR_INVALID_DEVICE_ID; return;
        }
        fingerIds[i] = holdingRegisters[baseAddr];
        fingerPositions[i] = holdingRegisters[baseAddr + 1];
    }

    for (int i = 0; i < palmCount; i++) {
        uint16_t baseAddr = REG_HAND_PALM_START + i * 3;
        if (holdingRegisters[baseAddr]<1 || holdingRegisters[baseAddr]>4) {
            holdingRegisters[REG_STATUS]=STATUS_ERR_INVALID_DEVICE_ID; return;
        }
        palmIds[i] = holdingRegisters[baseAddr];
        palmPositions[i] = holdingRegisters[baseAddr + 1];
        palmTimes[i] = holdingRegisters[baseAddr + 2];
    }

    if (!fingerProtection.configured() ||
        (fingerCount && !fingerProtection.accept(fingerCount,fingerIds,fingerPositions,millis()))) {
        holdingRegisters[REG_STATUS]=STATUS_ERR_GUARD_LOCKED; return;
    }

    // A finger fault/absent feedback must not be aggravated by continued palm
    // closure. Freeze all palm axes together; finger opening targets still enter
    // the guard. This is an interlock, not a palm force controller.
    if (palmCount > 0 && fingerProtection.palmsAllowed(millis())) {
        for (int i = 0; i < palmCount; i++) {
            BusServo.LobotSerialServoMove(palmIds[i], palmPositions[i], palmTimes[i]);
        }
    }

    holdingRegisters[REG_STATUS] = STATUS_GUARDED_ACCEPTED;
}

void handleReadFingerStatusCommand() {
    uint8_t devType = holdingRegisters[REG_DEVICE_TYPE];
    uint16_t actuatorId = holdingRegisters[REG_DEVICE_ID];

    clearFingerStatusRegisters(actuatorId);

    if (devType != 0) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_INVALID_DEVICE_TYPE;
        return;
    }

    if (!isConfigDeviceIdValid(actuatorId)) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_INVALID_DEVICE_ID;
        return;
    }

    FingerActuatorStatus status = {};
    status.actuatorId = actuatorId;

    if (queryFingerActuatorStatus(actuatorId, &status, 0)) {
        writeFingerStatusRegisters(status);
        holdingRegisters[REG_STATUS] = STATUS_FINGER_STATUS_OK;
        return;
    }

    writeFingerStatusRegisters(status);

    switch (status.responseErrorCode) {
        case FINGER_QUERY_ERROR_TIMEOUT:
            holdingRegisters[REG_STATUS] = STATUS_ERR_FINGER_STATUS_TIMEOUT;
            break;
        case FINGER_QUERY_ERROR_CHECKSUM:
            holdingRegisters[REG_STATUS] = STATUS_ERR_FINGER_STATUS_CHECKSUM;
            break;
        default:
            holdingRegisters[REG_STATUS] = STATUS_ERR_FINGER_STATUS_FORMAT;
            break;
    }
}

// Read-only palm query. Registers: ID, valid, signed position, temperature C,
// voltage mV, error (0=OK/1=query failed), all checks passed, query sequence.
void handleReadPalmStatusCommand() {
    static uint16_t sequence = 0;
    const uint16_t id = holdingRegisters[REG_DEVICE_ID];
    for (uint16_t i = 0; i < REG_PALM_STATUS_COUNT; ++i)
        holdingRegisters[REG_PALM_STATUS_BASE+i] = 0;
    holdingRegisters[REG_PALM_STATUS_BASE] = id;
    holdingRegisters[REG_PALM_STATUS_BASE+5] = 1;
    holdingRegisters[REG_PALM_STATUS_BASE+7] = ++sequence;
    if (holdingRegisters[REG_DEVICE_TYPE] != 1) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_INVALID_DEVICE_TYPE;
        return;
    }
    if (!isConfigDeviceIdValid(id)) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_INVALID_DEVICE_ID;
        return;
    }
    uint16_t position = 0, temperature = 0, voltage = 0;
    const bool positionOk = BusServo.readFeedbackValue(id, LOBOT_SERVO_POS_READ, &position);
    const bool temperatureOk = BusServo.readFeedbackValue(id, LOBOT_SERVO_TEMP_READ, &temperature);
    const bool voltageOk = BusServo.readFeedbackValue(id, LOBOT_SERVO_VIN_READ, &voltage);
    if (!positionOk || !temperatureOk || !voltageOk) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_PALM_STATUS;
        return;
    }
    holdingRegisters[REG_PALM_STATUS_BASE+1] = 1;
    holdingRegisters[REG_PALM_STATUS_BASE+2] = position;
    holdingRegisters[REG_PALM_STATUS_BASE+3] = temperature;
    holdingRegisters[REG_PALM_STATUS_BASE+4] = voltage;
    holdingRegisters[REG_PALM_STATUS_BASE+5] = 0;
    holdingRegisters[REG_PALM_STATUS_BASE+6] = 1;
    holdingRegisters[REG_STATUS] = STATUS_PALM_STATUS_OK;
}

// 读取设备ID
void executeReadDeviceID() {
    uint8_t devType = holdingRegisters[REG_DEVICE_TYPE];
    uint16_t queryId = holdingRegisters[REG_DEVICE_ID];

    if (!isConfigDeviceIdValid(queryId)) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_INVALID_DEVICE_ID;
        debugInvalidDeviceId("query", queryId);
        return;
    }

    if (devType == 0) {
        // Finger UART belongs exclusively to the V2 guard scheduler.
        holdingRegisters[REG_STATUS]=STATUS_ERR_ID_UNSUPPORTED; return;
    }

    if (devType != 1) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_ID_UNSUPPORTED;
        debugUnsupportedIdOperation(devType);
        return;
    }

    int readId = BusServo.LobotSerialServoReadID(queryId);
    if (isConfigDeviceIdValid(readId)) {
        holdingRegisters[REG_ID_RESULT] = readId;
        holdingRegisters[REG_STATUS] = STATUS_ID_READ_OK;
        return;
    }

    holdingRegisters[REG_STATUS] = STATUS_ERR_ID_READ_FAILED;
    debugIdReadFailed(devType, queryId);
}

// 修改设备ID
void executeSetDeviceID() {
    uint8_t devType = holdingRegisters[REG_DEVICE_TYPE];
    uint16_t oldId = holdingRegisters[REG_DEVICE_ID];
    uint16_t newId = holdingRegisters[REG_NEW_DEVICE_ID];

    if (!isConfigDeviceIdValid(oldId)) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_INVALID_DEVICE_ID;
        debugInvalidDeviceId("old", oldId);
        return;
    }

    if (!isConfigDeviceIdValid(newId)) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_INVALID_DEVICE_ID;
        debugInvalidDeviceId("new", newId);
        return;
    }

    // Treat same-ID writes as invalid to avoid issuing unnecessary configuration commands.
    if (oldId == newId) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_INVALID_DEVICE_ID;
        debugInvalidDeviceId("unchanged", newId);
        return;
    }

    if (devType == 0) {
        holdingRegisters[REG_STATUS]=STATUS_ERR_ID_UNSUPPORTED; return;
    }

    if (devType != 1) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_ID_UNSUPPORTED;
        debugUnsupportedIdOperation(devType);
        return;
    }

    BusServo.LobotSerialServoSetID(oldId, newId);
    delay(100);

    int readbackId = BusServo.LobotSerialServoReadID(newId);
    if (readbackId == newId) {
        holdingRegisters[REG_ID_RESULT] = newId;
        holdingRegisters[REG_STATUS] = STATUS_ID_SET_OK;
        return;
    }

    holdingRegisters[REG_STATUS] = STATUS_ERR_ID_SET_FAILED;
    debugIdSetFailed(devType, oldId, newId, readbackId);
}

// 执行清除错误
void executeClearError() {
    // DEBUG_SERIAL.println("=== 清除设备错误 ===");
    uint16_t devType = holdingRegisters[REG_DEVICE_TYPE];
    uint16_t devId = holdingRegisters[REG_DEVICE_ID];
    
    // DEBUG_SERIAL.print("设备类型: ");
    // DEBUG_SERIAL.println(devType == 0 ? "电缸" : "舵机");
    // DEBUG_SERIAL.print("设备ID: ");
    // DEBUG_SERIAL.println(devId);
    // DEBUG_SERIAL.println("===================");
    if (devType == 1) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_CLEAR_ERROR_UNSUPPORTED;
        debugUnsupportedClearError((uint8_t)devType);
        return;
    }

    if (devType != 0) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_INVALID_DEVICE_TYPE;
        debugInvalidDeviceType((uint8_t)devType);
        return;
    }

    if (!isConfigDeviceIdValid(devId)) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_INVALID_DEVICE_ID;
        return;
    }

    if (!fingerProtection.clearError(devId,millis())) {
        holdingRegisters[REG_STATUS]=STATUS_ERR_CLEAR_ERROR_REMAINING_FAULT; return;
    }
    // Startup/manual clear only. Keep the same UART owner running while awaiting
    // its correlated V2.0.4 reply; never clear periodically during grasping.
    const uint32_t started=millis();
    while (fingerProtection.clearResult(devId)==0 && uint32_t(millis()-started)<100) {
        fingerProtection.tick(millis()); delay(1);
    }
    FingerActuatorStatus status={};
    queryFingerActuatorStatus(devId,&status,0); writeFingerStatusRegisters(status);
    holdingRegisters[REG_STATUS]=fingerProtection.clearResult(devId)==1
        ? STATUS_CLEAR_ERROR_OK:STATUS_ERR_CLEAR_ERROR_VERIFY_FAILED;
}

// 处理读保持寄存器请求
void handleReadHoldingRegisters(uint8_t slaveAddress, uint16_t startAddr, uint16_t quantity) {
    // DEBUG_SERIAL.print("处理读保持寄存器: 起始地址=");
    // DEBUG_SERIAL.print(startAddr);
    // DEBUG_SERIAL.print(", 数量=");
    // DEBUG_SERIAL.println(quantity);
    
    // 验证地址范围
    if (quantity == 0) {
        sendErrorResponse(slaveAddress, 0x03, MODBUS_EXCEPTION_ILLEGAL_VALUE);
        return;
    }

    if (!isRegisterRangeValid(startAddr, quantity)) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_INVALID_REGISTER;
        debugInvalidRegister(startAddr);
        sendErrorResponse(slaveAddress, 0x03, MODBUS_EXCEPTION_ILLEGAL_ADDRESS);
        return;
    }
    
    if (quantity>125) { sendErrorResponse(slaveAddress,0x03,MODBUS_EXCEPTION_ILLEGAL_VALUE); return; }
    fingerProtection.registers(holdingRegisters+REG_FINGER_GUARD_BASE,86,millis());
    // 准备响应数据
    uint8_t response[3 + quantity * 2 + 2]; // 地址+功能码+字节数+数据+CRC
    response[0] = slaveAddress;
    response[1] = 0x03;
    response[2] = quantity * 2; // 字节数
    
    // 填充寄存器数据
    for (int i = 0; i < quantity; i++) {
        uint16_t regValue = holdingRegisters[startAddr + i];
        response[3 + i * 2] = (regValue >> 8) & 0xFF;
        response[4 + i * 2] = regValue & 0xFF;
    }
    
    // 计算CRC
    uint16_t crc = calculateCRC(response, 3 + quantity * 2);
    response[3 + quantity * 2] = crc & 0xFF;
    response[4 + quantity * 2] = crc >> 8;
    
    // 发送响应
    sendModbusResponse(response, 3 + quantity * 2 + 2);
}

// 发送写寄存器成功响应
void sendWriteRegisterResponse(uint8_t slaveAddress, uint16_t regAddress, uint16_t regValue) {
    uint8_t response[8];
    response[0] = slaveAddress;
    response[1] = 0x06;
    response[2] = (regAddress >> 8) & 0xFF;
    response[3] = regAddress & 0xFF;
    response[4] = (regValue >> 8) & 0xFF;
    response[5] = regValue & 0xFF;
    
    uint16_t crc = calculateCRC(response, 6);
    response[6] = crc & 0xFF;
    response[7] = crc >> 8;
    
    sendModbusResponse(response, sizeof(response));
}

// 发送写多个寄存器成功响应
void sendWriteMultipleRegistersResponse(uint8_t slaveAddress, uint16_t startAddr, uint16_t quantity) {
    uint8_t response[8];
    response[0] = slaveAddress;
    response[1] = 0x10;
    response[2] = (startAddr >> 8) & 0xFF;
    response[3] = startAddr & 0xFF;
    response[4] = (quantity >> 8) & 0xFF;
    response[5] = quantity & 0xFF;

    uint16_t crc = calculateCRC(response, 6);
    response[6] = crc & 0xFF;
    response[7] = crc >> 8;

    sendModbusResponse(response, sizeof(response));
}

// 发送错误响应
void sendErrorResponse(uint8_t slaveAddress, uint8_t functionCode, uint8_t exceptionCode) {
    uint8_t response[5];
    response[0] = slaveAddress;
    response[1] = functionCode | 0x80; // 设置错误标志
    response[2] = exceptionCode;
    
    uint16_t crc = calculateCRC(response, 3);
    response[3] = crc & 0xFF;
    response[4] = crc >> 8;
    
    sendModbusResponse(response, sizeof(response));
}

// 发送Modbus响应
void sendModbusResponse(uint8_t *data, uint16_t length) {
    digitalWrite(RS485_CTRL_PIN, HIGH);
    delayMicroseconds(100);
    
    // DEBUG_SERIAL.print("发送响应: ");
    for (int i = 0; i < length; i++) {
        if (data[i] < 16) {
            // DEBUG_SERIAL.print("0");
        }
        // DEBUG_SERIAL.print(data[i], HEX);
        // DEBUG_SERIAL.print(" ");
        RS485Serial.write(data[i]);
    }
    // DEBUG_SERIAL.println();
    
    RS485Serial.flush();
    delayMicroseconds(100);
    digitalWrite(RS485_CTRL_PIN, LOW);
}



void handleIncomingData() {
    lastReceiveTime = millis();
    
    uint16_t byteBudget=128;
    while (byteBudget-- && RS485Serial.available() && bufferIndex < sizeof(receiveBuffer)) {
        uint8_t data = RS485Serial.read();
        receiveBuffer[bufferIndex++] = data;
        // Only the fixed-size atomic protocol has an immediate completion path.
        // All 64 bytes including CRC must be present before it can execute.
        if (bufferIndex == ATOMIC_REQUEST_BYTES && receiveBuffer[1] == ATOMIC_FUNCTION) {
            processCompletePacket();
        }
    }
}

void processCompletePacket() {
    if (bufferIndex == 0) return;
    
    // DEBUG_SERIAL.println();
    // DEBUG_SERIAL.println("=== 完整数据包分析 ===");
    
    // 显示原始数据
    // DEBUG_SERIAL.print("原始数据 (");
    // DEBUG_SERIAL.print(bufferIndex);
    // DEBUG_SERIAL.print("字节): ");
    for (int i = 0; i < bufferIndex; i++) {
        if (receiveBuffer[i] < 16) {
            // DEBUG_SERIAL.print("0");
        }
        // DEBUG_SERIAL.print(receiveBuffer[i], HEX);
        // DEBUG_SERIAL.print(" ");
    }
    // DEBUG_SERIAL.println();
    
    // 解析Modbus帧
    if (bufferIndex >= 6) {
        analyzeModbusFrame();
    }
    
    bufferIndex = 0;
    // DEBUG_SERIAL.println("=== 数据包处理完成 ===");
}

void analyzeModbusFrame() {
    uint8_t slaveAddress = receiveBuffer[0];
    uint8_t functionCode = receiveBuffer[1];

    if (slaveAddress != MODBUS_SLAVE_ID) {
        return;
    }
    
    // DEBUG_SERIAL.print("从站地址: ");
    // DEBUG_SERIAL.println(slaveAddress);
    // DEBUG_SERIAL.print("功能码: 0x");
    // DEBUG_SERIAL.println(functionCode, HEX);
    
    // CRC校验
    uint16_t receivedCRC = (receiveBuffer[bufferIndex-1] << 8) | receiveBuffer[bufferIndex-2];
    uint16_t calculatedCRC = calculateCRC(receiveBuffer, bufferIndex - 2);
    
    // DEBUG_SERIAL.print("CRC校验: 接收=0x");
    // DEBUG_SERIAL.print(receivedCRC, HEX);
    // DEBUG_SERIAL.print(", 计算=0x");
    // DEBUG_SERIAL.print(calculatedCRC, HEX);
    
    if (receivedCRC != calculatedCRC) {
        // DEBUG_SERIAL.println(" (错误)");
        bufferIndex = 0;
        return;
    }

    if (functionCode == ATOMIC_FUNCTION) {
        handleAtomicControl();
        return;
    }
    
    // DEBUG_SERIAL.println(" (正确)");
    
    // 根据功能码处理请求
    uint16_t address = (receiveBuffer[2] << 8) | receiveBuffer[3];
    uint16_t quantity = (receiveBuffer[4] << 8) | receiveBuffer[5];
    uint8_t byteCount = (bufferIndex > 6) ? receiveBuffer[6] : 0;
    
    switch (functionCode) {
        case 0x03: // 读保持寄存器
            handleReadHoldingRegisters(slaveAddress, address, quantity);
            break;
        case 0x06: // 写单个寄存器
            handleWriteSingleRegister(slaveAddress, address, quantity);
            break;
        case 0x10: // 写多个寄存器
            handleWriteMultipleRegisters(slaveAddress, address, quantity, byteCount);
            break;
        default:
            holdingRegisters[REG_STATUS] = STATUS_ERR_UNSUPPORTED_FUNCTION;
            debugUnsupportedFunction(functionCode);
            sendErrorResponse(slaveAddress, functionCode, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
            break;
    }
}

uint16_t validateAtomicWords(const uint16_t *words) {
    if (words[0] != 5 || words[11] != 4 || words[24] || words[25] || words[26]) {
        return ATOMIC_ERR_COUNTS;
    }
    uint8_t seenFingers = 0, seenPalms = 0;
    for (int i=0; i<5; ++i) {
        uint16_t id = words[1+2*i], position = words[2+2*i];
        if (id < 1 || id > 5 || (seenFingers & (1 << (id-1)))) return ATOMIC_ERR_IDS;
        seenFingers |= 1 << (id-1);
        if (position > 2000) return ATOMIC_ERR_POSITION;
    }
    for (int i=0; i<4; ++i) {
        uint16_t id = words[12+3*i], position = words[13+3*i], duration = words[14+3*i];
        if (id < 1 || id > 4 || (seenPalms & (1 << (id-1)))) return ATOMIC_ERR_IDS;
        seenPalms |= 1 << (id-1);
        if (position > 1000) return ATOMIC_ERR_POSITION;
        if (!duration) return ATOMIC_ERR_TIME;
    }
    return 0;
}

void handleAtomicControl() {
    // CRC and slave address were checked by analyzeModbusFrame(). Never emit
    // even an error response for a one-way packet, so no stale RX accumulates.
    if (bufferIndex != ATOMIC_REQUEST_BYTES) return;
    uint8_t flags = receiveBuffer[3];
    uint16_t status = ATOMIC_ERR_HEADER;
    uint16_t words[27];
    if (receiveBuffer[2] == ATOMIC_VERSION && flags <= 1) {
        for (int i=0; i<27; ++i) {
            words[i] = (receiveBuffer[8+2*i] << 8) | receiveBuffer[9+2*i];
        }
        status = validateAtomicWords(words);
        if (!status) {
            // Validate the entire packet before publishing any target/motion.
            for (int i=0; i<27; ++i) holdingRegisters[20+i] = words[i];
            executeCombinedControl();
            status = holdingRegisters[REG_STATUS];
        }
    }
    holdingRegisters[REG_STATUS] = status;
    holdingRegisters[53] = (receiveBuffer[4] << 8) | receiveBuffer[5];
    holdingRegisters[54] = (receiveBuffer[6] << 8) | receiveBuffer[7];
    holdingRegisters[55] = status;
    // Diagnostic counters only, modulo 65536. One-way clients do not poll them.
    ++holdingRegisters[status == STATUS_GUARDED_ACCEPTED ? 56 : 57];
    if (flags == 1) {
        uint8_t response[12] = {MODBUS_SLAVE_ID, ATOMIC_FUNCTION, ATOMIC_VERSION, 1};
        for (int i=4; i<8; ++i) response[i] = receiveBuffer[i];
        response[8] = status >> 8;
        response[9] = status & 0xFF;
        uint16_t crc = calculateCRC(response,10);
        response[10] = crc & 0xFF;
        response[11] = crc >> 8;
        sendModbusResponse(response,sizeof(response));
    }
}

// CRC计算函数
uint16_t calculateCRC(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)data[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

#include <HardwareSerial.h>
#include <MicroServoControl.h>
#include <LobotSerialServoControl.h>

// ==== 硬件配置 - 根据原理图定义 ====
#define RS485_SERIAL_RX 16   // GPIO16 - RS485接收
#define RS485_SERIAL_TX 17   // GPIO17 - RS485发送
#define RS485_CTRL_PIN 4     // RS485方向控制引脚（GPIO4）
#define MODBUS_BAUDRATE 115200
#define MODBUS_SLAVE_ID 1
// Detect Modbus RTU inter-frame silence. 100 ms is too slow for teleoperation;
// 2 ms is conservative for 115200 baud and still separates complete frames.
#define MODBUS_FRAME_TIMEOUT_MS 2
#define MAX_GROUP_DEVICES 5

#define FINGER_UART_BAUDRATE 921600
#define FINGER_UART_RX_PIN 34
// Project wiring confirms GPIO35 as finger UART TX. On standard ESP32 modules,
// GPIO35 is input-only, so verify the actual board variant if TX does not work.
#define FINGER_UART_TX_PIN 35

#define SERVO_SERIAL_RX   18
#define SERVO_SERIAL_TX   19
#define receiveEnablePin  13
#define transmitEnablePin 14

#define CMD_COMBINED_CONTROL 0x04
#define CMD_READ_DEVICE_ID 0x05
#define CMD_SET_DEVICE_ID  0x06
#define CMD_READ_FINGER_STATUS 0x07

#define FINGER_STATUS_RESPONSE_LENGTH 22
#define FINGER_STATUS_QUERY_TIMEOUT_MS 20
#define CLEAR_ERROR_VERIFY_DELAY_MS 20

#define FINGER_QUERY_ERROR_NONE       0
#define FINGER_QUERY_ERROR_TIMEOUT    1
#define FINGER_QUERY_ERROR_FORMAT     2
#define FINGER_QUERY_ERROR_CHECKSUM   3

// 调试串口使用Serial0（USB）
// #define  DEBUG_SERIAL Serial
// 电缸串口
MicroServoController servo(Serial, FINGER_UART_BAUDRATE);

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
#define HOLDING_REGISTERS_SIZE 72
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

#define STATUS_COMBINED_ISSUED 0x90
#define STATUS_ID_READ_OK 0x91
#define STATUS_ID_SET_OK  0x92
#define STATUS_FINGER_STATUS_OK 0x93
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

uint8_t calculateActuatorChecksum(const uint8_t *frame, uint8_t firstIndex, uint8_t lastIndex) {
    uint8_t checksum = 0;
    for (uint8_t i = firstIndex; i <= lastIndex; i++) {
        checksum += frame[i];
    }
    return checksum;
}

void buildFingerStatusQueryFrame(uint8_t actuatorId, uint8_t frame[8]) {
    frame[0] = 0x55;
    frame[1] = 0xAA;
    frame[2] = 0x03;
    frame[3] = actuatorId;
    frame[4] = 0x04;
    frame[5] = 0x00;
    frame[6] = 0x22;
    frame[7] = calculateActuatorChecksum(frame, 2, 6);
}

uint8_t readFingerStatusResponse(uint8_t actuatorId, uint8_t response[22], uint16_t timeoutMs) {
    uint8_t responseIndex = 0;
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeoutMs && responseIndex < FINGER_STATUS_RESPONSE_LENGTH) {
        while (Serial.available() > 0 && responseIndex < FINGER_STATUS_RESPONSE_LENGTH) {
            uint8_t value = Serial.read();

            if (responseIndex == 0) {
                if (value == 0xAA) {
                    response[responseIndex++] = value;
                }
                continue;
            }

            if (responseIndex == 1) {
                if (value == 0x55) {
                    response[responseIndex++] = value;
                } else if (value == 0xAA) {
                    response[0] = value;
                    responseIndex = 1;
                } else {
                    responseIndex = 0;
                }
                continue;
            }

            response[responseIndex++] = value;
        }
    }

    if (responseIndex < FINGER_STATUS_RESPONSE_LENGTH) {
        return FINGER_QUERY_ERROR_TIMEOUT;
    }

    return FINGER_QUERY_ERROR_NONE;
}

uint8_t validateFingerStatusResponse(uint8_t actuatorId, const uint8_t response[22]) {
    if (response[0] != 0xAA || response[1] != 0x55 ||
        response[2] != 0x11 || response[3] != actuatorId ||
        response[4] != 0x04 || response[5] != 0x00 || response[6] != 0x22) {
        return FINGER_QUERY_ERROR_FORMAT;
    }

    if (response[21] != calculateActuatorChecksum(response, 2, 20)) {
        return FINGER_QUERY_ERROR_CHECKSUM;
    }

    return FINGER_QUERY_ERROR_NONE;
}

void parseFingerStatusResponse(const uint8_t response[22], FingerActuatorStatus *out) {
    out->actuatorId = response[3];
    out->targetPosition = (uint16_t)response[7] | ((uint16_t)response[8] << 8);
    out->currentPosition = (int16_t)((uint16_t)response[9] | ((uint16_t)response[10] << 8));
    out->temperatureC = (int8_t)response[11];
    out->currentMa = (uint16_t)response[12] | ((uint16_t)response[13] << 8);
    out->forceG = (int16_t)((uint16_t)response[14] | ((uint16_t)response[16] << 8));
    out->errorFlags = response[15];
    out->internal1 = (uint16_t)response[17] | ((uint16_t)response[18] << 8);
    out->internal2 = (uint16_t)response[19] | ((uint16_t)response[20] << 8);
    out->responseErrorCode = FINGER_QUERY_ERROR_NONE;
    out->checksumOk = true;
}

bool queryFingerActuatorStatus(uint8_t actuatorId, FingerActuatorStatus *out, uint16_t timeoutMs) {
    uint8_t queryFrame[8] = {0};
    uint8_t response[FINGER_STATUS_RESPONSE_LENGTH] = {0};

    out->actuatorId = actuatorId;
    out->responseErrorCode = FINGER_QUERY_ERROR_NONE;
    out->checksumOk = false;

    while (Serial.available() > 0) {
        Serial.read();
    }

    buildFingerStatusQueryFrame(actuatorId, queryFrame);
    Serial.write(queryFrame, sizeof(queryFrame));
    Serial.flush();
    // No finger-actuator DE/RE direction pin is controlled in this firmware.
    // If the physical finger bus uses a half-duplex transceiver that requires
    // explicit direction switching, RX will not work until that hardware pin is handled.

    uint8_t errorCode = readFingerStatusResponse(actuatorId, response, timeoutMs);
    if (errorCode != FINGER_QUERY_ERROR_NONE) {
        out->responseErrorCode = errorCode;
        return false;
    }

    errorCode = validateFingerStatusResponse(actuatorId, response);
    if (errorCode != FINGER_QUERY_ERROR_NONE) {
        out->responseErrorCode = errorCode;
        return false;
    }

    parseFingerStatusResponse(response, out);
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
    servo.InitServo(FINGER_UART_RX_PIN, FINGER_UART_TX_PIN);
    
    // 初始化RS485方向控制引脚
    pinMode(RS485_CTRL_PIN, OUTPUT);
    digitalWrite(RS485_CTRL_PIN, LOW); // 设置为接收模式
    // DEBUG_SERIAL.println("RS485方向控制引脚初始化完成");
    
    // 初始化RS485串口
    RS485Serial.begin(MODBUS_BAUDRATE, SERIAL_8E1, RS485_SERIAL_RX, RS485_SERIAL_TX);
    // DEBUG_SERIAL.println("RS485串口初始化完成");
    // DEBUG_SERIAL.print("波特率: "); 
    // DEBUG_SERIAL.println(MODBUS_BAUDRATE);
    
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
    
    delay(1);
}

// 初始化保持寄存器默认值
void initializeHoldingRegisters() {
    // 设置默认状态
    holdingRegisters[REG_STATUS] = 0xA0; // 默认空闲状态
    
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
    if (!isRegisterAddressValid(regAddress)) {
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

    if (!isRegisterRangeValid(startAddr, quantity)) {
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
    
    if (devType == 1) { // 舵机单控
        BusServo.LobotSerialServoMove(devId, position, execTime);
    } 
    else if (devType == 0) { // 电缸单控
        servo.setPosition(devId, position);
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
    if (devType == 0) { // 电缸组控
        servo.moveFingers(groupCount, idArray, posArray);
    } 
    else if (devType == 1) { // 舵机组控
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
        fingerIds[i] = holdingRegisters[baseAddr];
        fingerPositions[i] = holdingRegisters[baseAddr + 1];
    }

    for (int i = 0; i < palmCount; i++) {
        uint16_t baseAddr = REG_HAND_PALM_START + i * 3;
        palmIds[i] = holdingRegisters[baseAddr];
        palmPositions[i] = holdingRegisters[baseAddr + 1];
        palmTimes[i] = holdingRegisters[baseAddr + 2];
    }

    if (fingerCount > 0) {
        servo.moveFingers(fingerCount, fingerIds, fingerPositions);
    }

    if (palmCount > 0) {
        for (int i = 0; i < palmCount; i++) {
            BusServo.LobotSerialServoMove(palmIds[i], palmPositions[i], palmTimes[i]);
        }
    }

    holdingRegisters[REG_STATUS] = STATUS_COMBINED_ISSUED;
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

    if (queryFingerActuatorStatus(actuatorId, &status, FINGER_STATUS_QUERY_TIMEOUT_MS)) {
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
        int readId = servo.readDeviceID(queryId);
        if (isConfigDeviceIdValid(readId)) {
            holdingRegisters[REG_ID_RESULT] = readId;
            holdingRegisters[REG_STATUS] = STATUS_ID_READ_OK;
            return;
        }

        holdingRegisters[REG_STATUS] = STATUS_ERR_ID_READ_FAILED;
        debugIdReadFailed(devType, queryId);
        return;
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
        servo.setDeviceID(oldId, newId);
        delay(100);

        int readbackId = servo.readDeviceID(newId);
        if (readbackId != newId) {
            holdingRegisters[REG_STATUS] = STATUS_ERR_ID_SET_FAILED;
            debugIdSetFailed(devType, oldId, newId, readbackId);
            return;
        }

        if (holdingRegisters[REG_ID_SAVE] != 0) {
            servo.ParameterSave(newId);
            delay(100);

            readbackId = servo.readDeviceID(newId);
            if (readbackId != newId) {
                holdingRegisters[REG_STATUS] = STATUS_ERR_ID_SET_FAILED;
                debugIdSetFailed(devType, oldId, newId, readbackId);
                return;
            }
        }

        holdingRegisters[REG_ID_RESULT] = newId;
        holdingRegisters[REG_STATUS] = STATUS_ID_SET_OK;
        return;
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

    clearFingerStatusRegisters((uint8_t)devId);
    servo.clearError((uint8_t)devId);

    delay(CLEAR_ERROR_VERIFY_DELAY_MS);

    FingerActuatorStatus status = {};
    status.actuatorId = (uint8_t)devId;

    if (!queryFingerActuatorStatus((uint8_t)devId, &status, FINGER_STATUS_QUERY_TIMEOUT_MS)) {
        writeFingerStatusRegisters(status);
        holdingRegisters[REG_STATUS] = STATUS_ERR_CLEAR_ERROR_VERIFY_FAILED;
        return;
    }

    writeFingerStatusRegisters(status);

    if (!status.checksumOk || status.errorFlags != 0) {
        holdingRegisters[REG_STATUS] = STATUS_ERR_CLEAR_ERROR_REMAINING_FAULT;
        return;
    }

    holdingRegisters[REG_STATUS] = STATUS_CLEAR_ERROR_OK;
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
    
    while (RS485Serial.available() && bufferIndex < sizeof(receiveBuffer)) {
        uint8_t data = RS485Serial.read();
        receiveBuffer[bufferIndex++] = data;
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

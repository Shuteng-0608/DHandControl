#include <HardwareSerial.h>
#include <MicroServoControl.h>
#include <LobotSerialServoControl.h>

// ==== 硬件配置 - 根据原理图定义 ====
#define RS485_SERIAL_RX 16   // GPIO16 - RS485接收
#define RS485_SERIAL_TX 17   // GPIO17 - RS485发送
#define RS485_CTRL_PIN 4     // RS485方向控制引脚（GPIO4）

#define SERVO_SERIAL_RX   18
#define SERVO_SERIAL_TX   19
#define receiveEnablePin  13
#define transmitEnablePin 14


// 调试串口使用Serial0（USB）
// #define  DEBUG_SERIAL Serial
// 电缸串口
MicroServoController servo(Serial);

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
#define HOLDING_REGISTERS_SIZE 50
uint16_t holdingRegisters[HOLDING_REGISTERS_SIZE] = {0};

// 寄存器地址映射（根据你的Modbus协议定义）
#define REG_COMMAND       0   // 命令寄存器
#define REG_DEVICE_TYPE   1   // 设备类型
#define REG_DEVICE_ID     2   // 设备ID
#define REG_POSITION      3   // 目标位置
#define REG_EXEC_TIME     4   // 执行时间
#define REG_STATUS        5   // 状态寄存器
#define REG_GROUP_COUNT   6   // 组控数量
#define REG_GROUP_START   10  // 组控数据起始地址

void setup() {
    // 初始化调试串口（USB）
    // DEBUG_SERIAL.begin(115200);
    // DEBUG_SERIAL.println("=== ESP32 RS485 Modbus从站 ===");
    // DEBUG_SERIAL.println("系统启动中...");

    // 初始化舵机串口
    BusServoSerial.begin(115200, SERIAL_8N1, SERVO_SERIAL_RX, SERVO_SERIAL_TX);
    BusServo.OnInit();           // 初始化总线舵机库

    // 初始化电缸
    servo.InitServo();
    
    // 初始化RS485方向控制引脚
    pinMode(RS485_CTRL_PIN, OUTPUT);
    digitalWrite(RS485_CTRL_PIN, LOW); // 设置为接收模式
    // DEBUG_SERIAL.println("RS485方向控制引脚初始化完成");
    
    // 初始化RS485串口
    RS485Serial.begin(921600, SERIAL_8E1, RS485_SERIAL_RX, RS485_SERIAL_TX);
    // DEBUG_SERIAL.println("RS485串口初始化完成");
    // DEBUG_SERIAL.print("波特率: "); 
    // DEBUG_SERIAL.println(921600);
    
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
    
    if (bufferIndex > 0 && (millis() - lastReceiveTime > 100)) {
        processCompletePacket();
    }
    
    static uint32_t lastStatusTime = 0;
    if (millis() - lastStatusTime > 5000) {
        lastStatusTime = millis();
        // DEBUG_SERIAL.print("状态: 监听中... 最后状态: 0x");
        // DEBUG_SERIAL.println(holdingRegisters[REG_STATUS], HEX);
    }
    
    delay(10);
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
    if (regAddress >= HOLDING_REGISTERS_SIZE) {
        // DEBUG_SERIAL.println("错误: 寄存器地址超出范围");
        sendErrorResponse(slaveAddress, 0x06, 0x02); // 非法数据地址
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
            holdingRegisters[REG_STATUS] = 0xF0;
            executeClearError();
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
    uint8_t groupCount = holdingRegisters[REG_GROUP_COUNT];

    // 准备数组存储组控参数
    uint8_t idArray[5] = {0};
    int16_t posArray[5] = {0};
    uint16_t timeArray[5] = {0};
    
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

// 执行清除错误
void executeClearError() {
    // DEBUG_SERIAL.println("=== 清除设备错误 ===");
    uint8_t devType = holdingRegisters[REG_DEVICE_TYPE];
    uint8_t devId = holdingRegisters[REG_DEVICE_ID];
    
    // DEBUG_SERIAL.print("设备类型: ");
    // DEBUG_SERIAL.println(devType == 0 ? "电缸" : "舵机");
    // DEBUG_SERIAL.print("设备ID: ");
    // DEBUG_SERIAL.println(devId);
    // DEBUG_SERIAL.println("错误已清除");
    // DEBUG_SERIAL.println("===================");
    servo.clearError(devId);
}

// 处理读保持寄存器请求
void handleReadHoldingRegisters(uint8_t slaveAddress, uint16_t startAddr, uint16_t quantity) {
    // DEBUG_SERIAL.print("处理读保持寄存器: 起始地址=");
    // DEBUG_SERIAL.print(startAddr);
    // DEBUG_SERIAL.print(", 数量=");
    // DEBUG_SERIAL.println(quantity);
    
    // 验证地址范围
    if (startAddr + quantity > HOLDING_REGISTERS_SIZE) {
        // DEBUG_SERIAL.println("错误: 寄存器地址超出范围");
        sendErrorResponse(slaveAddress, 0x03, 0x02); // 非法数据地址
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
    
    switch (functionCode) {
        case 0x03: // 读保持寄存器
            handleReadHoldingRegisters(slaveAddress, address, quantity);
            break;
        case 0x06: // 写单个寄存器
            handleWriteSingleRegister(slaveAddress, address, quantity);
            break;
        default:
            // DEBUG_SERIAL.println("不支持的功能码");
            sendErrorResponse(slaveAddress, functionCode, 0x01); // 非法功能
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
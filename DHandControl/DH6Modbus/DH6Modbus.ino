#include <ModbusRTU.h>
#include "LobotSerialServoControl.h"
#include "MicroServoControl.h"

// ==== 硬件配置 - 根据实际连接修改 ====
#define SLAVE_ID 1       // Modbus从站地址
#define RS485_CTRL_PIN 4 // RS485控制引脚

// 手掌舵机串口引脚
#define SERVO_SERIAL_RX   18
#define SERVO_SERIAL_TX   19
#define RECEIVE_ENABLE_PIN  13
#define TRANSMIT_ENABLE_PIN 14

// 电缸串口引脚
#define CYLINDER_SERIAL_TX 16
#define CYLINDER_SERIAL_RX 17

// ==== Modbus寄存器定义 ====
enum Registers {
    CMD_REG = 0,         // 命令寄存器
    DEV_TYPE_REG = 1,     // 设备类型 (0=电缸, 1=舵机)
    DEV_ID_REG = 2,       // 设备ID
    POSITION_REG = 3,     // 位置/角度值
    TIME_REG = 4,         // 执行时间(ms)
    STATUS_REG = 5,       // 状态反馈
    GROUP_SIZE_REG = 6,   // 组控设备数量
    GROUP_START_REG = 10  // 组控数据起始地址
};

// ==== 硬件对象 ====
// 手掌舵机控制器
LobotSerialServoControl palmServo(Serial1, RECEIVE_ENABLE_PIN, TRANSMIT_ENABLE_PIN);

// 电缸控制器
MicroServoController cylinder(Serial2);

// Modbus对象
ModbusRTU mb;

void setup() {
    Serial.begin(115200);
    
    // 初始化舵机控制器
    Serial1.begin(115200, SERIAL_8N1, SERVO_SERIAL_RX, SERVO_SERIAL_TX);
    Serial2.begin(921600, SERIAL_8N1, CYLINDER_SERIAL_TX, CYLINDER_SERIAL_RX);
    
    palmServo.OnInit();
    cylinder.InitServo();
    
    // 初始化Modbus
    mb.begin(&Serial, RS485_CTRL_PIN);
    mb.setSlaveId(SLAVE_ID);
    
    // 初始化Modbus寄存器
    initModbusRegisters();
    
    // 设置回调函数
    setupModbusCallbacks();
    
    Serial.println("系统初始化完成");
}

void loop() {
    mb.task();
    delay(10);
}

// 初始化Modbus寄存器
void initModbusRegisters() {
    mb.addHreg(CMD_REG, 0);
    mb.addHreg(DEV_TYPE_REG, 0);
    mb.addHreg(DEV_ID_REG, 1);
    mb.addHreg(POSITION_REG, 0);
    mb.addHreg(TIME_REG, 1000);
    mb.addHreg(STATUS_REG, 0);
    mb.addHreg(GROUP_SIZE_REG, 0);
    
    // 初始化组控寄存器
    for (int i = 0; i < 20; i++) {
        mb.addHreg(GROUP_START_REG + i, 0);
    }
}

// 设置Modbus回调
void setupModbusCallbacks() {
    mb.onSetHreg(CMD_REG, handleCommand);
}

// 处理Modbus命令
void handleCommand(uint16_t address, uint16_t value) {
    if (value == 0) return; // 忽略0值
    
    uint8_t cmd = value;
    mb.Hreg(CMD_REG, 0); // 重置命令寄存器
    
    switch (cmd) {
        case 1: // 单个设备控制
            handleSingleControl();
            break;
        case 2: // 组控
            handleGroupControl();
            break;
        case 3: // 清除错误
            handleClearError();
            break;
        default:
            mb.Hreg(STATUS_REG, 0xE0); // 无效命令错误
            break;
    }
}

// 单个设备控制
void handleSingleControl() {
    uint8_t devType = mb.Hreg(DEV_TYPE_REG);
    uint8_t devID = mb.Hreg(DEV_ID_REG);
    uint16_t position = mb.Hreg(POSITION_REG);
    uint16_t timeVal = mb.Hreg(TIME_REG);
    
    // 位置范围验证
    if (!validatePosition(devType, position)) {
        mb.Hreg(STATUS_REG, 0xE1); // 位置超限错误
        return;
    }
    
    bool success = false;
    
    if (devType == 0) {
        // 控制电缸
        cylinder.setPosition(devID, position);
        success = true;
    } else if (devType == 1) {
        // 控制手掌舵机
        palmServo.LobotSerialServoMove(devID, position, timeVal);
        success = true;
    }
    
    // 更新状态寄存器
    if (success) {
        mb.Hreg(STATUS_REG, 0xA0 | (devID & 0x0F)); // 状态码: A类型+ID
    } else {
        mb.Hreg(STATUS_REG, 0xE2); // 控制失败错误
    }
}

// 组控处理
void handleGroupControl() {
    uint8_t groupSize = mb.Hreg(GROUP_SIZE_REG);
    uint8_t devType = mb.Hreg(DEV_TYPE_REG);
    
    // 组控数量验证
    if (groupSize > 5) {
        mb.Hreg(STATUS_REG, 0xE3); // 组控数量超限
        return;
    }
    
    bool success = false;
    
    if (devType == 0) {
        // 电缸组控
        uint8_t idList[5];
        int16_t posList[5];
        
        for (int i = 0; i < groupSize; i++) {
            idList[i] = mb.Hreg(GROUP_START_REG + i*2);
            posList[i] = mb.Hreg(GROUP_START_REG + i*2 + 1);
            
            // 位置验证
            if (!validatePosition(devType, posList[i])) {
                mb.Hreg(STATUS_REG, 0xE1); // 位置超限错误
                return;
            }
        }
        
        cylinder.moveFingers(groupSize, idList, posList);
        success = true;
    } else {
        // 舵机组控
        uint8_t idList[5];
        int16_t posList[5];
        int16_t timeList[5];
        
        for (int i = 0; i < groupSize; i++) {
            idList[i] = mb.Hreg(GROUP_START_REG + i*3);
            posList[i] = mb.Hreg(GROUP_START_REG + i*3 + 1);
            timeList[i] = mb.Hreg(GROUP_START_REG + i*3 + 2);
            
            // 位置验证
            if (!validatePosition(devType, posList[i])) {
                mb.Hreg(STATUS_REG, 0xE1); // 位置超限错误
                return;
            }
        }
        
        // 使用广播同步控制
        palmServo.movePalms(groupSize, idList, posList, timeList);
        success = true;
    }
    
    // 更新状态寄存器
    if (success) {
        mb.Hreg(STATUS_REG, 0xC0 | (groupSize & 0x0F)); // 状态码: C类型+数量
    } else {
        mb.Hreg(STATUS_REG, 0xE4); // 组控失败错误
    }
}

// 清除错误
void handleClearError() {
    uint8_t devID = mb.Hreg(DEV_ID_REG);
    uint8_t devType = mb.Hreg(DEV_TYPE_REG);
    
    bool success = false;
    
    if (devType == 0) {
        // 清除电缸错误
        cylinder.clearError(devID);
        success = true;
    } else if (devType == 1) {
        // 清除舵机错误 - 需要根据您的库实现
        // 假设使用LobotSerialServoLoad作为清除错误
        palmServo.LobotSerialServoLoad(devID);
        success = true;
    }
    
    // 更新状态寄存器
    if (success) {
        mb.Hreg(STATUS_REG, 0xF0); // 清除错误成功
    } else {
        mb.Hreg(STATUS_REG, 0xE5); // 清除错误失败
    }
}

// 位置验证
bool validatePosition(uint8_t devType, int16_t position) {
    // 两种设备都使用0-1000的位置范围
    return (position >= 0 && position <= 1000);
}

// ==== 硬件状态监控 ====
void monitorHardwareStatus() {
    static uint32_t lastCheck = 0;
    if (millis() - lastCheck > 1000) {
        lastCheck = millis();
        
        // 检查电缸状态
        // 这里需要根据您的库添加状态检查函数
        // 例如: int status = cylinder.getStatus(1);
        
        // 检查舵机状态
        // 例如: int temp = palmServo.LobotSerialServoReadTemp(1);
    }
}
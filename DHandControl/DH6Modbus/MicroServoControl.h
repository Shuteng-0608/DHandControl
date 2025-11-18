/*
  MicroServoControl.h - Arduino Library for LA Series Micro Servo Cylinder
  Based on Protocol Document v1.0
  Created by [Your Name], [Date]
*/

#ifndef MICROSERVOCONTROL_H
#define MICROSERVOCONTROL_H

#include <Arduino.h>

#define GET_LOW_BYTE(A) (uint8_t)((A))       //宏函数 获得A的低八位

#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8) //宏函数 获得A的高八位



// 通信参数定义
#define DEFAULT_BAUDRATE   921600
#define TX_PIN             35
#define RX_PIN             34

// 指令类型定义
#define CMD_READ              0x01
#define CMD_WRITE             0x02
#define CMD_MOVE_ABSOLUTE_NF  0x03 // 定位模式（无反馈）
#define CMD_SET_WORK_MODE     0x04 // 单控指令
#define CMD_CLEAR_FAULT       0x1E
#define CMD_MOVE_ABSOLUTE     0x21 // 定位模式（反馈状态信息）
#define CMD_MOVE_RELATIVE     0x22
#define CMD_STOP              0x23
#define CMD_MOVE_ABSOLUTE_BC  0xF2 // 广播定位模式（无反馈）
#define CMD_WR_REGISTER       0x32 




class MicroServoController {
  private:
    HardwareSerial *_serial;
    uint32_t _baudRate;
    
    uint8_t calculateChecksum(uint8_t *data, uint8_t len);
    
  public:
    MicroServoController(HardwareSerial &serial, uint32_t baud = DEFAULT_BAUDRATE);
    void InitServo();
    
    void ParameterSave(uint8_t id);                           // 参数装订
    void setDeviceID(uint8_t id, uint8_t newID);              // 修改ID
    void setPosition(uint8_t id, int16_t position);           // 绝对定位
    void clearError(uint8_t id);                              // 故障清除
    void moveFingers(uint8_t num, uint8_t id_list[], int16_t pos_list[]);  // 广播定位模式
    
};

#endif // MICROSERVOCONTROL_H

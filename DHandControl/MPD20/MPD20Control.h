/*
  MicroServoControl.h - Arduino Library for LA Series Micro Servo Cylinder
  Based on Protocol Document v1.0
  Created by [Your Name], [Date]
*/

#ifndef MPD20CONTROL_H
#define MPD20CONTROL_H

#include <Arduino.h>

#define GET_LOW_BYTE(A) (uint8_t)((A))       //宏函数 获得A的低八位

#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8) //宏函数 获得A的高八位



// 通信参数定义
#define DEFAULT_BAUDRATE   115200
#define TX_PIN             35
#define RX_PIN             34

// 指令类型定义
#define CMD_READ              0x02
#define CMD_WRITE             0x03


class MPD20Controller {
  private:
    HardwareSerial *_serial;
    uint32_t _baudRate;
    
    uint8_t calculateChecksum(uint8_t *data, uint8_t len);
    
  public:
    MPD20Controller(HardwareSerial &serial, uint32_t baud = DEFAULT_BAUDRATE);
    void InitServo();
    
    void setDeviceID(uint8_t id, uint8_t newID);                    // 修改ID
    void setPosition(uint8_t id, int16_t position, int16_t speed);  // 绝对定位
    
};

#endif // MICROSERVOCONTROL_H

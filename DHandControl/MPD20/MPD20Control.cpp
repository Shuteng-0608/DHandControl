/*
  MicroServoControl.cpp - Implementation of MPD20Controller Class
*/

#include "MPD20Control.h"

MPD20Controller::MPD20Controller(HardwareSerial &serial, uint32_t baud) : 
  _serial(&serial), _baudRate(baud) {}

void MPD20Controller::InitServo() {
  _serial->begin(_baudRate);
  delay(100);
}


uint8_t MPD20Controller::calculateChecksum(uint8_t *data, uint8_t len) {
  uint8_t sum = 0;
  for (int i = 2; i <= len; i++) {
    sum += data[i];
  }
  sum = (0xFF - (sum & 0xFF)) & 0xFF;
  return sum;
}


void MPD20Controller::setDeviceID(uint8_t id, uint8_t newID){
  byte buf[8];
  
  buf[0] = buf[1] = 0xFF;                       // 帧头
  buf[2] = id;                                  // 设备地址
  buf[3] = 4;                                   // 数据长度
  buf[4] = CMD_WRITE;                           // 指令类型
  buf[5] = 0x05;                                // 内存地址 
  buf[6] = newID;                               // 数据段
  buf[7] = calculateChecksum(buf, 7);           

  _serial->write(buf, 8);
}


void MPD20Controller::setPosition(uint8_t id, int16_t position, int16_t speed) {
  byte buf[13];
  if (position <= 120) position = 120;
  if (position >= 850) position = 850;
  if (speed <= 0) speed = 0;
  if (speed >= 100) speed = 100;
  buf[0] = buf[1] = 0xFF;                       // 帧头
  buf[2] = id;                                  // 设备地址
  buf[3] = 9;                                   // 数据长度
  buf[4] = CMD_WRITE;                           // 指令类型
  buf[5] = 0x2A;                                // 内存地址 
  buf[6] = GET_HIGH_BYTE((uint16_t)position);   // 目标位置高位(120 - 850)
  buf[7] = GET_LOW_BYTE((uint16_t)position);    // 目标位置低位
  buf[8] = 0x00;                                // 运行时间高位(暂不支持)
  buf[9] = 0x00;                                // 运行时间低位
  buf[10] = GET_HIGH_BYTE((uint16_t)speed);     // 运行速度高位(0 - 100)
  buf[11] = GET_LOW_BYTE((uint16_t)speed);      // 运行速度低位
  buf[12] = calculateChecksum(buf, 12);           

  _serial->write(buf, 13);
  
}

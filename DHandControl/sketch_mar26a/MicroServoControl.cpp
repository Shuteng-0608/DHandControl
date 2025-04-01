/*
  MicroServoControl.cpp - Implementation of MicroServoController Class
*/

#include "MicroServoControl.h"

MicroServoController::MicroServoController(HardwareSerial &serial, uint32_t baud) : 
  _serial(&serial), _baudRate(baud) {}

void MicroServoController::InitServo() {
//  _serial->begin(_baudRate, SERIAL_8N1, RX_PIN, TX_PIN);
  _serial->begin(_baudRate);
  delay(100);
}


uint8_t MicroServoController::calculateChecksum(uint8_t *data, uint8_t len) {
  uint8_t sum = 0;
  for (int i = 2; i <= len; i++) {
    sum += data[i];
  }
  return sum;
}


void MicroServoController::ParameterSave(uint8_t id){
  byte buf[8];
  
  buf[0] = 0x55;                                // 帧头
  buf[1] = 0xAA;
  buf[2] = 3;                                   // 帧长度
  buf[3] = id;                                  // ID号
  buf[4] = 0x04;                                // 指令类型
  buf[5] = 0x00;                                // 控制表索引  
  buf[6] = 0x20;                               // 数据段
  buf[7] = calculateChecksum(buf, 7);           

  _serial->write(buf, 8);
}


void MicroServoController::setDeviceID(uint8_t id, uint8_t newID){
  byte buf[8];
  
  buf[0] = 0x55;                                // 帧头
  buf[1] = 0xAA;
  buf[2] = 3;                                   // 帧长度
  buf[3] = id;                                  // ID号
  buf[4] = 0x02;                                // 指令类型
  buf[5] = 0x02;                                // 控制表索引  
  buf[6] = newID;                               // 数据段
  buf[7] = calculateChecksum(buf, 7);           

  _serial->write(buf, 8);
}


void MicroServoController::setPosition(uint8_t id, int16_t position) {
  byte buf[9];
  
  buf[0] = 0x55;                                // 帧头
  buf[1] = 0xAA;
  buf[2] = 4;                                   // 帧长度
  buf[3] = id;                                  // ID号
  buf[4] = CMD_MOVE_ABSOLUTE;                   // 指令类型
  buf[5] = 0x37;                                // 控制表索引  
  buf[6] = GET_LOW_BYTE((uint16_t)position);    // 数据段低位
  buf[7] = GET_HIGH_BYTE((uint16_t)position);   // 数据段高位
  buf[8] = calculateChecksum(buf, 7);           

  _serial->write(buf, 9);
  
}

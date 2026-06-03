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


uint8_t MicroServoController::calculateChecksum(byte data[], uint8_t len) {
  uint8_t sum = 0;
  for (int i = 2; i <= len; i++) {
    sum += data[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
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

int MicroServoController::readDeviceID(uint8_t id){
  byte buf[8];
  byte response[8];
  uint8_t responseIndex = 0;
  uint32_t startTime = 0;

  while (_serial->available() > 0) {
    _serial->read();
  }

  buf[0] = 0x55;                                // 帧头
  buf[1] = 0xAA;
  buf[2] = 3;                                   // 帧长度
  buf[3] = id;                                  // ID号
  buf[4] = CMD_READ;                            // 读指令
  buf[5] = 0x02;                                // 电缸ID控制表索引
  buf[6] = 0x01;                                // 读取1字节
  buf[7] = calculateChecksum(buf, 6);

  _serial->write(buf, 8);

  startTime = millis();
  while ((millis() - startTime) < 20 && responseIndex < 8) {
    if (_serial->available() > 0) {
      response[responseIndex++] = _serial->read();
    }
  }

  if (responseIndex < 8) {
    return -1;
  }

  if (response[0] != 0xAA || response[1] != 0x55) {
    return -1;
  }

  if (response[2] != 0x03 || response[3] != id ||
      response[4] != CMD_READ || response[5] != 0x02) {
    return -1;
  }

  if (response[7] != calculateChecksum(response, 6)) {
    return -1;
  }

  return response[6];
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


void MicroServoController::moveFingers(uint8_t num, uint8_t id_list[], int16_t pos_list[]) {

  byte buf[3*num+6];

  buf[0] = 0x55;                                // 帧头
  buf[1] = 0xAA;
  buf[2] = 3*num + 1;                           // 帧长度

  buf[3] = 0xFF;                                // 广播ID
  buf[4] = 0xF2;                                // 定位标志

  for(int i = 0; i < num; i++) {
    buf[5 + 3*i] = id_list[i];
    buf[6 + 3*i] = GET_LOW_BYTE((uint16_t)pos_list[i]);
    buf[7 + 3*i] = GET_HIGH_BYTE((uint16_t)pos_list[i]);
  }

  buf[3*num+5] = calculateChecksum(buf, 3*num+4); 
  
  _serial->write(buf, 3*num+6);
  
}

void MicroServoController::clearError(uint8_t id){
  byte buf[8];
  
  buf[0] = 0x55;                                // 帧头
  buf[1] = 0xAA;                                // 帧头
  buf[2] = 3;                                   // 帧长度
  buf[3] = id;                                  // ID号
  buf[4] = CMD_SET_WORK_MODE;                   // 单控指令
  buf[5] = 0x00;                                // 控制表索引
  buf[6] = CMD_CLEAR_FAULT;                     // 故障清除
  buf[7] = calculateChecksum(buf, 6);           

  _serial->write(buf, 8);
}






































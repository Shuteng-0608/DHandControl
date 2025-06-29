#include "esp32-hal.h"
#include "HardwareSerial.h"
#include "Print.h"
//﻿
//* FileName:      LobotSerialServoControl.h
//* Company:     Hiwonder
//* Date:          2020/05/13  16:53
// *Last Modification Date: 202005131938
//* www.hiwonder.com

#include "LobotSerialServoControl.h"
#include <Stream.h>

LobotSerialServoControl::LobotSerialServoControl(HardwareSerial &A)
{
  isAutoEnableRT = true;
	isUseHardwareSerial = true;
	SerialX = (Stream*)(&A);
}
LobotSerialServoControl::LobotSerialServoControl(HardwareSerial &A,int receiveEnablePin, int transmitEnablePin)
{
  isAutoEnableRT = false;
  this->receiveEnablePin = receiveEnablePin;
  this->transmitEnablePin = transmitEnablePin;
  
  isUseHardwareSerial = true;
  SerialX = (Stream*)(&A);
}

void LobotSerialServoControl::OnInit(void)
{
  if(!isAutoEnableRT)
  {
    pinMode(receiveEnablePin, OUTPUT);
    pinMode(transmitEnablePin, OUTPUT);
    RxEnable();
  }
}

inline void LobotSerialServoControl::RxEnable(void)
{
  digitalWrite(receiveEnablePin, HIGH);
  digitalWrite(transmitEnablePin, LOW);
}
inline void LobotSerialServoControl::TxEnable(void)
{
  digitalWrite(receiveEnablePin, LOW);
  digitalWrite(transmitEnablePin, HIGH);
}

byte LobotSerialServoControl::LobotCheckSum(byte buf[])
{
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

void LobotSerialServoControl::movePalms(uint8_t num, uint8_t id_list[], int16_t pos_list[], int16_t time_list[]){
  byte buf[5*num+6];
  for(int i = 0; i < num; i++){
    if(pos_list[i] < 0) pos_list[i] = 0;
    if(pos_list[i] > 1000) pos_list[i] = 1000;
  }
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = 0xFE;                              // 广播ID
  buf[3] = 5*num+3;                             // 数据长度
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;       // 指令
  for(int i = 0; i < num; i++) {              // 参数
    buf[5 + 5*i] = id_list[i];
    buf[6 + 5*i] = GET_LOW_BYTE((uint16_t)pos_list[i]);
    buf[7 + 5*i] = GET_HIGH_BYTE((uint16_t)pos_list[i]);
    buf[8 + 5*i] = GET_LOW_BYTE((int16_t)time_list[i]);
    buf[9 + 5*i] = GET_HIGH_BYTE((int16_t)time_list[i]);
  }
  buf[5*num+5] = LobotCheckSum(buf);
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 3*num+6);

}




void LobotSerialServoControl::LobotSerialServoMove(uint8_t id, int16_t position, uint16_t time)
{
  byte buf[10];
  if(position < 0)
    position = 0;
  if(position > 1000)
    position = 1000;
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 10);
}

void LobotSerialServoControl::LobotSerialServoStopMove(uint8_t id)
{
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_MOVE_STOP;
  buf[5] = LobotCheckSum(buf);
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 6);
}

void LobotSerialServoControl::LobotSerialServoSetID(uint8_t oldID, uint8_t newID)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO ID WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}

void LobotSerialServoControl::LobotSerialServoSetMode(uint8_t id, uint8_t Mode, int16_t Speed)
{
  byte buf[10];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = Mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((uint16_t)Speed);
  buf[8] = GET_HIGH_BYTE((uint16_t)Speed);
  buf[9] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Set Mode");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 10);
}

void LobotSerialServoControl::LobotSerialServoLoad(uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = LobotCheckSum(buf);
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}

void LobotSerialServoControl::LobotSerialServoUnload(uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = LobotCheckSum(buf);
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
}

int LobotSerialServoControl::LobotSerialServoReceiveHandle(byte *ret)
{
  bool frameStarted = false;
  bool receiveFinished = false;
  byte frameCount = 0;
  byte dataCount = 0;
  byte dataLength = 2;
  byte rxBuf;
  byte recvBuf[32];
  byte i;

  while (SerialX->available()) {
    rxBuf = SerialX->read();
    delayMicroseconds(100);
    if (!frameStarted) {
      if (rxBuf == LOBOT_SERVO_FRAME_HEADER) {
        frameCount++;
        if (frameCount == 2) {
          frameCount = 0;
          frameStarted = true;
          dataCount = 1;
        }
      }
      else {
        frameStarted = false;
        dataCount = 0;
        frameCount = 0;
      }
    }
    if (frameStarted) {
      recvBuf[dataCount] = (uint8_t)rxBuf;
      if (dataCount == 3) {
        dataLength = recvBuf[dataCount];
        if (dataLength < 3 || dataCount > 7) {
          dataLength = 2;
          frameStarted = false;
        }
      }
      dataCount++;
      if (dataCount == dataLength + 3) {

        if (LobotCheckSum(recvBuf) == recvBuf[dataCount - 1]) {

          frameStarted = false;
          memcpy(ret, recvBuf + 4, dataLength);
          return 1;
        }
        return -1;
      }
    }
  }
}

int LobotSerialServoControl::LobotSerialServoReadPosition(uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_POS_READ;
  buf[5] = LobotCheckSum(buf);

  while (SerialX->available())
    SerialX->read();
    
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 6);
  if(isUseHardwareSerial)
  {
    delayMicroseconds(550);
  }
  if(isAutoEnableRT == false)
    RxEnable();
  while (!SerialX->available()) {
    count -= 1;
    if (count < 0)
      return -1;
  }

  if (LobotSerialServoReceiveHandle(buf) > 0)
    ret = BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;

  return ret;
}

int LobotSerialServoControl::LobotSerialServoReadVin(uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_VIN_READ;
  buf[5] = LobotCheckSum(buf);

  while (SerialX->available())
    SerialX->read();

  if(isAutoEnableRT == false)
    TxEnable();
    
  SerialX->write(buf, 6);

  if(isUseHardwareSerial)
  {
    delayMicroseconds(550);
  }
  if(isAutoEnableRT == false)
    RxEnable();
  
  while (!SerialX->available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;

  return ret;
}

int LobotSerialServoControl::LobotSerialServoControl::LobotSerialServoReadID(uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_ID_READ;
  buf[5] = LobotCheckSum(buf);

  while (SerialX->available())
    SerialX->read();
    
  if(isAutoEnableRT == false)
    TxEnable();
  SerialX->write(buf, 6);
  if(isUseHardwareSerial)
  {
    delayMicroseconds(550);
  }
  if(isAutoEnableRT == false)
    RxEnable();
  while (!SerialX->available()) {
    count -= 1;
    if (count < 0)
      return -1;
  }

  if (LobotSerialServoReceiveHandle(buf) > 0)
    ret = (int16_t)BYTE_TO_HW(0x00, buf[1]);
  else
    ret = -2;
  return ret;
}

int LobotSerialServoControl::LobotSerialServoReadTemp(uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_TEMP_READ;
  buf[5] = LobotCheckSum(buf);

  while (SerialX->available())
    SerialX->read();

  if(isAutoEnableRT == false)
    TxEnable();
    
  SerialX->write(buf, 6);

  if(isUseHardwareSerial)
  {
    delayMicroseconds(550);
  }
  if(isAutoEnableRT == false)
    RxEnable();
  
  while (!SerialX->available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(buf) > 0)
    ret = (int16_t)BYTE_TO_HW(0x00, buf[1]);
  else
    ret = -2048;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}

int LobotSerialServoControl::LobotSerialServoReadDev(uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_ANGLE_OFFSET_READ;
  buf[5] = LobotCheckSum(buf);

  while (SerialX->available())
    SerialX->read();

  if(isAutoEnableRT == false)
    TxEnable();
    
  SerialX->write(buf, 6);

  if(isUseHardwareSerial)
  {
    delayMicroseconds(550);
  }
  if(isAutoEnableRT == false)
    RxEnable();
  
  while (!SerialX->available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(buf) > 0)
    ret = (int16_t)BYTE_TO_HW(0x00, buf[1]);
  else
    ret = -2048;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}

int LobotSerialServoControl::LobotSerialServoReadAngleRange(uint8_t id){
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_ANGLE_LIMIT_READ;
  buf[5] = LobotCheckSum(buf);
  
  while (SerialX->available())
    SerialX->read();

  if(isAutoEnableRT == false){
    TxEnable();
  }
    
    
  SerialX->write(buf, 6);

  if(isUseHardwareSerial)
  {
    delayMicroseconds(550);
  }
  if(isAutoEnableRT == false){
    RxEnable();
  }
  
  while (!SerialX->available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(buf) > 0)
    {
      retL = (int16_t)BYTE_TO_HW(buf[2], buf[1]); 
      retH = (int16_t)BYTE_TO_HW(buf[4], buf[3]);
    }
  else
    ret = -2048;
  return ret;
}

int LobotSerialServoControl::LobotSerialServoReadVinLimit(uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_VIN_LIMIT_READ;
  buf[5] = LobotCheckSum(buf);
  
  while (SerialX->available())
    SerialX->read();

  if(isAutoEnableRT == false){
    TxEnable();
  }
    
  SerialX->write(buf, 6);

  if(isUseHardwareSerial)
  {
    delayMicroseconds(550);
  }
  if(isAutoEnableRT == false){
    RxEnable();
  }
  
  while (!SerialX->available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(buf) > 0)
    {
      vinL = (int16_t)BYTE_TO_HW(buf[2], buf[1]); 
      vinH = (int16_t)BYTE_TO_HW(buf[4], buf[3]);
    }
  else
    ret = -2048;
  return ret;
}

int LobotSerialServoControl::LobotSerialServoReadTempLimit(uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_TEMP_MAX_LIMIT_READ;
  buf[5] = LobotCheckSum(buf);
  while (SerialX->available())
    SerialX->read();

  if(isAutoEnableRT == false){
    TxEnable();
  }
    
  SerialX->write(buf, 6);

  if(isUseHardwareSerial)
  {
    delayMicroseconds(550);
  }
  if(isAutoEnableRT == false){
    RxEnable();
  }
  
  while (!SerialX->available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(buf) > 0)
    ret = (int16_t)BYTE_TO_HW(0x00, buf[1]);
  else
    ret = -2049;
    
  return ret;
}

int LobotSerialServoControl::LobotSerialServoReadLoadOrUnload(uint8_t id){
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_READ;
  buf[5] = LobotCheckSum(buf);
  while (SerialX->available())
    SerialX->read();

  if(isAutoEnableRT == false){
    TxEnable();
  }
    
  SerialX->write(buf, 6);

  if(isUseHardwareSerial)
  {
    delayMicroseconds(550);
  }
  if(isAutoEnableRT == false){
    RxEnable();
  }
  
  while (!SerialX->available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(buf) > 0)
    ret = (int16_t)BYTE_TO_HW(0x00, buf[1]);
  else
    ret = -2049;
    
  return ret;
}

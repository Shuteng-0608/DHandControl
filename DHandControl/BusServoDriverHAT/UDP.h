#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <stdint.h>
#include "LobotSerialServoControl.h"
#include "MicroServoControl.h"

// WIFI_STA settings.
const char* STA_SSID = "Shuteng";
const char* STA_PWD  = "12345678";

typedef char s8;
typedef unsigned char u8; 
typedef unsigned short u16; 
typedef short s16;
typedef unsigned long u32;  
typedef long s32;


#define SERVO_SERIAL_RX   18
#define SERVO_SERIAL_TX   19
#define receiveEnablePin  13
#define transmitEnablePin 14
HardwareSerial HardwareSerial(1);
LobotSerialServoControl BusServo(HardwareSerial,receiveEnablePin,transmitEnablePin);
MicroServoController servo(Serial);

WiFiUDP Udp;
const int udpPort = 12345; // Port number for UDP communication
IPAddress local_ip(192, 168, 4, 5); // static local IP   
IPAddress gateway(192, 168, 4, 1); // AP    
IPAddress subnet(255, 255, 255, 0);


void turn();

// Set board as STA
// Connect to know wifi (Target wifi needs to satisfy - "2.4GHz" && "WPA2")
void wifiConnect(){
  WiFi.config(local_ip, gateway, subnet);
  WiFi.begin(STA_SSID, STA_PWD);

  while(WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}


// UDP init
void udpInit(){
  Udp.begin(udpPort);
  Serial.print("UDP listening on port ");
  Serial.println(udpPort);
}


// Handle coming msg
void handleUdp(){

  int packetSize = Udp.parsePacket(); // Check is there any msg coming
  if(packetSize){
    char buffer[255];
    int len = Udp.read(buffer, 255); // Read the coming msg
    if (len > 0) {
      buffer[len] = 0; // Null-terminate the string
    }
    String message = String(buffer);
    Serial.println("Received: " + message);

    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, message);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }

    const char* cmd = doc["Cmd"];  
    
    if(strcmp(cmd, "Turn") == 0){
      turn();
    }

    else if(strcmp(cmd, "ServoMove") == 0){ 
      uint8_t id = doc["ID"];
      int16_t position = doc["Pos"];
      uint16_t time = doc["Time"];
      // uint8_t id, int16_t position, uint16_t time      
      BusServo.LobotSerialServoMove(id, position, time); // 设置1号舵机运行到500脉宽位置，运行时间为1000毫秒
      delay(2000); // 延时2000毫秒
    }
    else if(strcmp(cmd, "FingerMove") == 0){
      uint8_t id = doc["ID"];
      int16_t position = doc["Pos"];
      servo.setPosition(id, position);
      delay(2000);
    }
  }
}

void turn() {
  BusServo.LobotSerialServoMove(1,500,1000); // 设置1号舵机运行到500脉宽位置，运行时间为1000毫秒
  delay(2000); // 延时2000毫秒

  BusServo.LobotSerialServoMove(1,1000,1000); // 设置1号舵机运行到1000脉宽位置，运行时间为1000毫秒
  delay(2000); // 延时2000毫秒

  BusServo.LobotSerialServoMove(1,0,1000); // 设置1号舵机运行到0脉宽位置，运行时间为1000毫秒
  delay(2000); // 延时2000毫秒

  BusServo.LobotSerialServoMove(1,500,1000); // 设置1号舵机运行到500脉宽位置，运行时间为1000毫秒
  delay(2000); // 延时2000毫秒
}

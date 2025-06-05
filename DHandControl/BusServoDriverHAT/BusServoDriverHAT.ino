#include "UDP.h"
#include "HardwareSerial.h"



void setup() {
  
  servo.InitServo();           // 初始化电缸控制库
//   put your setup code here, to run once:
//  Serial.begin(115200);        // 设置串口波特率
//  Serial.println("start...");  // 串口打印"start..."
  BusServo.OnInit();           // 初始化总线舵机库
  HardwareSerial.begin(115200, SERIAL_8N1, SERVO_SERIAL_RX, SERVO_SERIAL_TX);
  delay(500);                  // 延时500毫秒
  
  wifiConnect();
  udpInit();
  
}


void loop() {
  handleUdp();
  
}

/*
 * esp32_s3.ino
 * author: Junyu Luo
 * Created on: 2025
 * Description:
 * 该文件作为esp32 s3项目的启动文件，包含了setup初始化和loop函数
 *
 * Copyright (c) 2025 MagicCube Robotics
 */

#include <ArduinoJson.h>
//wifi模块
//#include "wifi.h"
//文件系统
#include "nvs_user.h"
#include "usbmsc.h"

//lcd显示屏
#include "display.h"
//rgb
#include "rgb.h"
//ros
//#include "ros_esp32.h"
//gpio
#include "gpio.h"
//BLE蓝牙
#include "ble.h"

void setup() {
    // 串口初始化
    Serial.begin(115200);
    // 初始化wifi
    //init_wifi();
    // 初始化io设备
    init_gpio();
    // LED初始化
    init_rgb();
    // 初始化NVS
    // init_nvs();
    // 从nvs中读取wifi名字和密码，读取当前手柄是左右手
    // read_nvs_data(ssid, password, ros_master_ip);
    // read_nvs_hand(hand_name);
    // usb存储配置
    //init_usbmsc();
    // ros初始化
    //init_ros();

    // 初始化蓝牙设备
    init_BLE();
    // 初始化屏幕，并加载进度条，进度条是假的，只是为了好看
    init_display();
}

String MainTaskLoop(){
  // 主逻辑循环，当通信正常的情况下不断轮询执行，最终返回json字符串作为esp32向外发送的字符串
  // 绿灯代表通信正常
  set_rgb_green();
  // 读取引脚状
  int button_state_left = digitalRead(button_up);
  int button_state_right = digitalRead(button_down);

  int adcValue_bat = analogRead(Bat_sample);
  float batteryVoltage = (adcValue_bat / 4095.0) * 3.3 * 2;
  float batt_percent = ((batteryVoltage - 3.0)/(4.2-3.0))*100;
  dtostrf(batteryVoltage, 4, 2, battery_info);

  // 显示电池信息
  displayBatteryIcon(batt_percent);
  display.display();

  DynamicJsonDocument doc(1024);  // 动态分配内存
  doc["left_button"] = button_state_left;
  doc["right_button"] = button_state_right;
  doc["battery_adc"] = adcValue_bat;
  doc["battery_vol"] = batteryVoltage;
  doc["battery_per"] = batt_percent;
  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

void ErrorTaskLoop(){
  static bool ErrorLEDFlag=true;
  // 错误循环，当通信异常，程序检测错误，跳出主逻辑循环后的错误循环
  // 蓝灯代表通信失联
  if(ErrorLEDFlag==true){
    set_rgb_yellow();
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    ErrorLEDFlag=false;
    return;
  }
  else{
    set_rgb_black();
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (HIGH is the voltage level)
    ErrorLEDFlag=true;
  }
}

void loop() {

  // BLE循环
  BLEDeviceLoop(MainTaskLoop,ErrorTaskLoop,100);

  }

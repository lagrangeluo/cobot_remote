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
#include "wifi.h"
//文件系统
#include "nvs_user.h"
#include "usbmsc.h"

//lcd显示屏
#include "display.h"
//rgb
#include "rgb.h"
//ros
#include "ros_esp32.h"

// 两个按键，up和down是历史遗留名字
#define button_up   4
#define button_down 43
// #define hall_adc    7
#define Bat_sample  9

void setup() {
    // 串口初始化
    Serial.begin(115200);

    // 初始化wifi
    init_wifi();

    // 将引脚设置为上拉输入模式
    pinMode(button_down, INPUT_PULLUP);
    pinMode(button_up, INPUT_PULLUP);
    
    // LED初始化
    init_rgb();
    // 初始化NVS
    init_nvs();
    // 从nvs中读取wifi名字和密码，读取当前手柄是左右手
    read_nvs_data(ssid, password, ros_master_ip);
    read_nvs_hand(hand_name);
    // usb存储配置
    //init_usbmsc();
    // ros初始化
    init_ros();
    // 初始化屏幕，并加载进度条，进度条是假的，只是为了好看
    init_display();
}

void loop() {
    //远程ota处理
    //ArduinoOTA.handle();  //OTA处理

    // wifi连接标志位
    static bool wifiConnected = false;
    // 轮询间隔
    static unsigned long previousMillis = 0;
    const long interval = 100;  // 检查Wi-Fi状态的间隔缩短至100毫秒

    unsigned long currentMillis = millis();
    // 当wifi失去连接
    if (WiFi.status() != WL_CONNECTED) {
        if (wifiConnected) {
            wifiConnected = false;
            Serial.println("WiFi disconnected.");
        }

        // 定时轮询
        if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;
            // esp32板载led闪烁
            digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
            // IO外接rgb灯珠控制
            set_rgb_yellow();
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);  
            set_rgb_black(); 
            delay(50);
        
            display.clearDisplay();
            display.setTextSize(1.5);
            display.setCursor(0, 0);
            display.println("Waiting for WiFi: ");
            display.setCursor(0, 15);
            display.println(ssid);
            display.display();

            // 不断监测文件系统中的wifi名字和密码
            read_nvs_data(ssid,password,ros_master_ip);
            //重连wifi
            WiFi.begin(ssid, password);
        }

        // 连接需要时间，所以我们给它300毫秒的时间来尝试连接
        for (int i = 0; i < 3; i++) {
            if (WiFi.status() == WL_CONNECTED) {
                wifiConnected = true;
                break;
            }
            delay(100);  // 每次检查的间隔为100毫秒
        }
    } else {
        // 如果连接成功后，反复检测Wi-Fi状态，设置灯光标识
        wifiConnected = true;
        static bool firstConnect = true;  // 用于第一次连接的标志
        
        // 看门狗程序
        //loop_watchdog();

        if (firstConnect) {
            firstConnect = false;
            // 马达震动两次，表示wifi连接成功
            motor_start_short_twice();
        }

        if (currentMillis - previousMillis >= 40) { // 每20毫秒发送一次消息，相当于50Hz
          previousMillis = currentMillis;

          // 常亮
          digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (HIGH is the voltage level)
          // 根据ros是否连接，显示不同的灯光颜色
          if(if_ros_connected == false)
            set_rgb_blue();
          else
            set_rgb_green();

          display_wifi_status(ssid,hand_name);
          
          // 读取引脚状态，引脚上拉输入，所以进行逻辑反转
          int button_state_left = 1 - digitalRead(button_up);
          int button_state_right = 1 - digitalRead(button_down);

          // 读取adc数值
        //   int adcValue_x = analogRead(ADC_x_PIN); // 读取ADC值（0-4095）
        //   int adcValue_y = analogRead(ADC_y_PIN); // 读取ADC值（0-4095）
        //   int adcValue_hall = analogRead(hall_adc); //读取adc数值
          int adcValue_bat = analogRead(Bat_sample);
          float batteryVoltage = (adcValue_bat / 4095.0) * 3.3 * 2;
          float batt_percent = ((batteryVoltage - 3.0)/(4.2-3.0))*100;
          dtostrf(batteryVoltage, 4, 2, battery_info);

          // 显示电池信息
          displayBatteryIcon(batt_percent);
          display.display();

          if (nh.connected())
          {
            DynamicJsonDocument doc(1024);  // 动态分配内存
            doc["left_button"] = button_state_left;
            doc["right_button"] = button_state_right;
            doc["battery_adc"] = adcValue_bat;
            doc["battery_vol"] = batteryVoltage;
            doc["battery_per"] = batt_percent;
            String jsonString;
            serializeJson(doc, jsonString);
            str_msg.data = jsonString.c_str();
            chatter->publish( &str_msg );
          }
        
          nh.spinOnce();
        }

        // 如果Wi-Fi断开，立即进入未连接状态
        if (WiFi.status() != WL_CONNECTED) {
            wifiConnected = false;
            firstConnect = true;
            WiFi.disconnect();  // 断开当前连接
        }
    }
  }
/*
 * esp32_s3.ino
 *
 * Created on: 2024
 * Description:
 * 该文件作为esp32 s3项目的启动文件，包含了setup初始化和loop函数
 *
 * Copyright (c) 2024 AgileX Robotics
 */
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <AsyncWebSocket.h>
#include <ArduinoJson.h>
//文件系统
#include "nvs_user.h"
#include "usbmsc.h"
//OTA
#include <ArduinoOTA.h>
//lcd显示屏
#include "display.h"
//rgb
#include "rgb.h"
//motor振动马达
#include "motor.h"
//ros
#include "ros_esp32.h"

// 摇杆x,y轴针脚
#define ADC_x_PIN   1
#define ADC_y_PIN   2
// 摇杆按键针脚
#define js_button   3
// 两个按键，up和down是历史遗留名字
#define button_up   4
#define button_down 43
#define hall_adc    7
#define Bat_sample  9

static char* ssid = (char*)malloc(32 * sizeof(char));        // 设置Wi-Fi名称
static char* password = (char*)malloc(32 * sizeof(char));    // 设置Wi-Fi密码

void setup() {
    // 串口初始化
    Serial.begin(115200);
    // 连接wifi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    // 初始化远程OTA升级，初始化后可以支持wifi烧录
    ArduinoOTA.setPort(3232);  // OTA端口号，就这样固定
    ArduinoOTA.begin();

    // 将引脚设置为上拉输入模式
    pinMode(js_button, INPUT_PULLUP);
    pinMode(button_down, INPUT_PULLUP);
    pinMode(button_up, INPUT_PULLUP);
    
    // LED初始化
    init_rgb();
    // 振动马达初始化
    init_motor();
    // 初始化NVS
    init_nvs();
    // 从nvs中读取wifi名字和密码，读取当前手柄是左右手
    read_nvs_data(ssid, password, ros_master_ip);
    read_nvs_hand(hand_name);
    // usb存储配置
    init_usbmsc();
    // ros初始化
    init_ros();
    // 初始化屏幕，并加载进度条，进度条是假的，只是为了好看
    init_display();
    // 轻微震动马达代表初始化完成
    motor_start_short();
}

void loop() {
    //远程ota处理
    ArduinoOTA.handle();  //OTA处理

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
        loop_watchdog();

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

          // // 显示wifi连接状态
          // IPAddress localIP = WiFi.localIP();
          // String ipString = localIP.toString();
          // display.clearDisplay();
          // display.setTextSize(1.5);
          // display.setCursor(0, 0);
          // display.printf("WiFi: %s",ssid);
          // display.setCursor(0, 10);
          // display.printf("IP: %s", ipString.c_str());
          // display.setCursor(0,20);
          // display.printf("hand: %s", hand_name);
          
          // 显示wifi连接状态
          display_wifi_status(ssid,hand_name);
          
          // 读取引脚状态，引脚上拉输入，所以进行逻辑反转
          int js_state = 1 - digitalRead(js_button);
          int button_state_up = 1 - digitalRead(button_up);
          int button_state_down = 1 - digitalRead(button_down);

          // 读取adc数值
          int adcValue_x = analogRead(ADC_x_PIN); // 读取ADC值（0-4095）
          int adcValue_y = analogRead(ADC_y_PIN); // 读取ADC值（0-4095）
          int adcValue_hall = analogRead(hall_adc); //读取adc数值
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
            doc["x"] = adcValue_x;
            doc["y"] = adcValue_y;
            doc["js_button"] = js_state;
            doc["up_button"] = button_state_up;
            doc["down_button"] = button_state_down;
            doc["hall_adc"] = adcValue_hall;
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
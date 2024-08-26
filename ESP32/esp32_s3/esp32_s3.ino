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
//ros
#include "ros_esp32.h"

#define LEDS_COUNT  1
#define LEDS_PIN    8
#define ADC_x_PIN   1
#define ADC_y_PIN   2
#define js_button   3
#define button_up   4
#define button_down 43
#define hall_adc    7

static char* ssid = (char*)malloc(32 * sizeof(char));        // 设置Wi-Fi名称
static char* password = (char*)malloc(32 * sizeof(char));    // 设置Wi-Fi密码

void setup() {
    // 串口初始化
    Serial.begin(115200);
    // // 连接wifi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    // 初始化远程OTA升级
    ArduinoOTA.setPort(3232);  // OTA端口号
    ArduinoOTA.begin();

    // 将引脚设置为上拉输入模式
    pinMode(js_button, INPUT);
    pinMode(button_down, INPUT_PULLUP);
    pinMode(button_up, INPUT_PULLUP);
    // LED初始化
    pinMode(LED_BUILTIN, OUTPUT);

    // 初始化NVS
    init_nvs();
    // 从nvs中读取wifi名字和密码，读取当前手柄是左右手
    read_nvs_data(ssid, password);
    read_nvs_hand(hand_name);
    // ros初始化
    init_ros();
    // usb存储配置
    init_usbmsc();

    // 初始化屏幕，并加载进度条
    init_display();
}

void loop() {

    //远程ota处理
    ArduinoOTA.handle();  //OTA处理

    static bool wifiConnected = false;
    static unsigned long previousMillis = 0;
    const long interval = 100;  // 检查Wi-Fi状态的间隔缩短至100毫秒

    unsigned long currentMillis = millis();
    // 当wifi失去连接
    if (WiFi.status() != WL_CONNECTED) {
        if (wifiConnected) {
            wifiConnected = false;
            Serial.println("WiFi disconnected.");
        }

        if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;
            // 闪烁
            digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
            delay(100);                      
            digitalWrite(LED_BUILTIN, LOW);   
            delay(50);
        }

        display.clearDisplay();
        display.setTextSize(1.5);
        display.setCursor(0, 0);
        display.println("Waiting for WiFi: ");
        display.setCursor(0, 15);
        display.println(ssid);
        display.display();

        // 不断监测文件系统中的wifi名字和密码
        read_nvs_data(ssid,password);
        //重连wifi
        WiFi.begin(ssid, password);

        // 连接需要时间，所以我们给它300毫秒的时间来尝试连接
        for (int i = 0; i < 3; i++) {
            if (WiFi.status() == WL_CONNECTED) {
                wifiConnected = true;
                break;
            }
            delay(100);  // 每次检查的间隔为100毫秒
        }
    } else {
        // 如果连接成功后，反复检测Wi-Fi状态
        wifiConnected = true;
        static bool firstConnect = true;  // 用于第一次连接的标志

        if (firstConnect) {
            Serial.println("WiFi connected.");
            Serial.print("IP Address: ");
            Serial.println(WiFi.localIP());

            // 显示wifi连接状态
            IPAddress localIP = WiFi.localIP();
            String ipString = localIP.toString();
            display.clearDisplay();
            display.setTextSize(1.5);
            display.setCursor(0, 0);
            display.printf("WiFi: %s",ssid);
            display.setCursor(0, 10);
            display.printf("IP: %s", ipString.c_str());
            display.setCursor(0,20);
            display.printf("hand: %s", hand_name);
            display.display();
            firstConnect = false;
        }
        
        if (currentMillis - previousMillis >= 40) { // 每20毫秒发送一次消息，相当于50Hz
          previousMillis = currentMillis;

          // 读取引脚状态，引脚上拉输入，所以进行逻辑反转
          int js_state = digitalRead(js_button);  
          int button_state_up = 1 - digitalRead(button_up);
          int button_state_down = 1 - digitalRead(button_down);

          // 读取adc数值
          int adcValue_x = analogRead(ADC_x_PIN); // 读取ADC值（0-4095）
          int adcValue_y = analogRead(ADC_y_PIN); // 读取ADC值（0-4095）
          int adcValue_hall = analogRead(hall_adc); //读取adc数值

          String voltageStr_x = String(adcValue_x);
          String voltageStr_y = String(adcValue_y);
          String send_msg = "x: " + voltageStr_x + "y" + voltageStr_y;

          if (nh.connected())
          {
            DynamicJsonDocument doc(1024);  // 动态分配内存
            doc["x"] = adcValue_x;
            doc["y"] = adcValue_y;
            doc["js_button"] = 1 - js_state;
            doc["up_button"] = 1 - button_state_up;
            doc["down_button"] = 1 - button_state_down;
            doc["hall_adc"] = adcValue_hall;
            String jsonString;
            serializeJson(doc, jsonString);
            str_msg.data = jsonString.c_str();
            chatter->publish( &str_msg );
          }
        
          nh.spinOnce();
        }

        // 常亮
        digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (HIGH is the voltage level)

        // 如果Wi-Fi断开，立即进入未连接状态
        if (WiFi.status() != WL_CONNECTED) {
            wifiConnected = false;
            firstConnect = true;
            Serial.println("WiFi lost connection, attempting to reconnect...");
            WiFi.disconnect();  // 断开当前连接
        }
    }
  }
#include <WiFi.h>
#include "Freenove_WS2812_Lib_for_ESP32.h"
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <AsyncWebSocket.h>
#include <ArduinoJson.h>
#include <ros.h>
#include <std_msgs/String.h>

#define LEDS_COUNT  1
#define LEDS_PIN    8
#define ADC_x_PIN   0
#define ADC_y_PIN   1
#define js_button   2
#define button_up   4
#define button_down 3
#define CHANNEL     0

const char* ssid = "松灵";        // 设置Wi-Fi名称
const char* password = "agilex2024#";            // 设置Wi-Fi密码
// const char* ssid = "Navis3";        // 设置Wi-Fi名称
// const char* password = "12345678";            // 设置Wi-Fi密码
// const char* ssid = "ESP32_Hotspot";        // 设置Wi-Fi名称
// const char* password = "12345678";            // 设置Wi-Fi密码

Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);

// Create AsyncWebServer object on port 80
// AsyncWebServer server(81);
// AsyncWebSocket ws("/aloha");

//ros
IPAddress server(10, 10, 96, 111);
//IPAddress server(10, 10, 24, 12);

// IPAddress server(192, 168, 1, 109);
// IPAddress server(192, 168, 1, 2);

uint16_t serverPort = 11411;
char hello[13] = "hello world!";
uint16_t period = 1000;
uint32_t last_time = 0;
bool publish_flag = false;
ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("esp32_json_string", &str_msg);

// void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
//   switch (type) {
//     case WS_EVT_CONNECT:
//       Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
//       break;
//     case WS_EVT_DISCONNECT:
//       Serial.printf("WebSocket client #%u disconnected\n", client->id());
//       break;
//     case WS_EVT_DATA:
//       handleWebSocketMessage(arg, data, len);
//       break;
//     case WS_EVT_PONG:
//     case WS_EVT_ERROR:
//       break;
//   }
// }

// void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
//   AwsFrameInfo *info = (AwsFrameInfo*)arg;
//   if (info->final && info->index == 0 && info->len == len) {
//     if (info->opcode == WS_TEXT) {
//       data[len] = 0;
//       Serial.printf("Received text: %s\n", (char*)data);
//     }
//   }
// }

void setup() {
    Serial.begin(115200);
    strip.begin();
    strip.setBrightness(20);  
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    pinMode(js_button, INPUT);   // 将引脚设置为输入模式
    pinMode(button_down, INPUT);   // 将引脚设置为输入模式
    pinMode(button_up, INPUT);   // 将引脚设置为输入模式
    // ws.onEvent(onEvent);

    // server.addHandler(&ws);
    // // Start server
    // server.begin();
  
    // nh.initNode();
    // nh.advertise(chatter);
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();

    // Another way to get IP
    Serial.print("ROS IP = ");
    Serial.println(nh.getHardware()->getLocalIP());

    // Start to be polite
    nh.advertise(chatter);
}

void loop() {
    // str_msg.data = "Hello from ESP32";
    // chatter.publish(&str_msg);
    // nh.spinOnce();
    // delay(1000);

    static bool wifiConnected = false;
    static unsigned long previousMillis = 0;
    const long interval = 100;  // 检查Wi-Fi状态的间隔缩短至100毫秒

    unsigned long currentMillis = millis();
    // Serial.println(WiFi.status());
    if (WiFi.status() != WL_CONNECTED) {
        if (wifiConnected) {
            wifiConnected = false;
            Serial.println("WiFi disconnected.");
        }

        if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;
            // 闪烁红灯
            strip.setLedColorData(0, 255, 0, 0); // 红色
            strip.show();
            delay(50);  // 红灯亮50毫秒
            strip.setLedColorData(0, 0, 0, 0);  // 熄灭
            strip.show();
            delay(50);  // 红灯熄灭50毫秒
        }

        Serial.print("Attempting to connect to WiFi: ");
        Serial.println(ssid);
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
            firstConnect = false;
        }
        
        if (currentMillis - previousMillis >= 40) { // 每20毫秒发送一次消息，相当于50Hz
          previousMillis = currentMillis;

          int js_state = digitalRead(js_button);  // 读取引脚状态
          int button_state_up = digitalRead(button_up);
          int button_state_down = digitalRead(button_down);

          // if (button_state_up == HIGH) {
          //   Serial.println("up Pin is HIGH");
          // } else {
          //   Serial.println("up Pin is LOW");
          // }
          // if (button_state_down == HIGH) {
          //   Serial.println("down Pin is HIGH");
          // } else {
          //   Serial.println("down Pin is LOW");
          // }

          // 读取adc数值
          int adcValue_x = analogRead(ADC_x_PIN); // 读取ADC值（0-4095）
          int adcValue_y = analogRead(ADC_y_PIN); // 读取ADC值（0-4095）

          // ESP32的ADC分辨率为12位，所以ADC值的范围是0到4095
          // 将ADC值转换为电压（假设输入电压范围是0V到3.3V）
          // 节约计算资源，放弃float的计算
          // float voltage_x = adcValue_x * (3.3 / 4095.0);
          // float voltage_y = adcValue_y * (3.3 / 4095.0);

          String voltageStr_x = String(adcValue_x);
          String voltageStr_y = String(adcValue_y);
          String send_msg = "x: " + voltageStr_x + "y" + voltageStr_y;

          // for (AsyncWebSocketClient *client : ws.getClients())
          // {
          //   if (!client->queueIsFull()) {
          //     DynamicJsonDocument doc(1024);  // 动态分配内存
          //     doc["x"] = adcValue_x;
          //     doc["y"] = adcValue_y;

          //     String jsonString;
          //     serializeJson(doc, jsonString);
          //     client->text(jsonString);
          //   } else {
          //     client->close(); // 清空队列
          //     Serial.println("Client message queue is full, skipping message");
          // }
          if (nh.connected())
          {
            DynamicJsonDocument doc(1024);  // 动态分配内存
            doc["x"] = adcValue_x;
            doc["y"] = adcValue_y;
            doc["js_button"] = js_state;
            doc["up_button"] = button_state_up;
            doc["down_button"] = button_state_down;
            String jsonString;
            serializeJson(doc, jsonString);
            str_msg.data = jsonString.c_str();
            chatter.publish( &str_msg );
            if(publish_flag == false)
            {
              Serial.println("Start publish ros topic...");
              publish_flag = true;
            }
          } else {
            if(publish_flag == true)
            {
              Serial.println("rosserial node disconnected");
              publish_flag = false;
            }
          }
        
          nh.spinOnce();
        }
        // 绿灯常亮
        strip.setLedColorData(0, 0, 255, 0); // 绿色
        strip.show();

        // 如果Wi-Fi断开，立即进入未连接状态
        if (WiFi.status() != WL_CONNECTED) {
            wifiConnected = false;
            firstConnect = true;
            Serial.println("WiFi lost connection, attempting to reconnect...");
            // 清除上一次的IP地址打印
            Serial.print("Previous IP Address cleared: ");
            Serial.println(WiFi.localIP());
            WiFi.disconnect();  // 断开当前连接
        }
    }
  }


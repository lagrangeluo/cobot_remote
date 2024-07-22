#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <AsyncWebSocket.h>
#include <ArduinoJson.h>
#include <ros.h>
#include <std_msgs/String.h>

#define LEDS_COUNT  1
#define LEDS_PIN    8
#define ADC_x_PIN   1
#define ADC_y_PIN   2
#define js_button   3
#define button_up   5
#define button_down 4
#define CHANNEL     0

const char* ssid = "松灵";        // 设置Wi-Fi名称
const char* password = "agilex2024#";            // 设置Wi-Fi密码
// const char* ssid = "Navis3";        // 设置Wi-Fi名称
// const char* password = "12345678";            // 设置Wi-Fi密码
// const char* ssid = "ESP32_Hotspot";        // 设置Wi-Fi名称
// const char* password = "12345678";            // 设置Wi-Fi密码

//ros
// IPAddress server(10, 10, 96, 111);
IPAddress server(10, 10, 24, 12);

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

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    pinMode(js_button, INPUT);   // 将引脚设置为输入模式
    pinMode(button_down, INPUT);   // 将引脚设置为输入模式
    pinMode(button_up, INPUT);   // 将引脚设置为输入模式

    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();

    // Another way to get IP
    Serial.print("ROS IP = ");
    Serial.println(nh.getHardware()->getLocalIP());

    // Start to be polite
    nh.advertise(chatter);
}

void loop() {

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
            // 闪烁
            digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
            delay(100);                      
            digitalWrite(LED_BUILTIN, LOW);   
            delay(50);                      
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

          // 读取adc数值
          int adcValue_x = analogRead(ADC_x_PIN); // 读取ADC值（0-4095）
          int adcValue_y = analogRead(ADC_y_PIN); // 读取ADC值（0-4095）

          String voltageStr_x = String(adcValue_x);
          String voltageStr_y = String(adcValue_y);
          String send_msg = "x: " + voltageStr_x + "y" + voltageStr_y;

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

        // 常亮
        digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (HIGH is the voltage level)

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


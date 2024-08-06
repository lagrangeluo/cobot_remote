#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <AsyncWebSocket.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <ros.h>
#include <std_msgs/String.h>

#define LEDS_COUNT  1
#define LEDS_PIN    8
#define ADC_x_PIN   1
#define ADC_y_PIN   3
#define js_button   2
#define button_up   4
#define button_down 43
#define hall_adc    7
#define SDA_PIN     5
#define SCL_PIN     6

// const char* ssid = "mammotion";        // 设置Wi-Fi名称
// const char* password = "Mamo12345";            // 设置Wi-Fi密码
const char* ssid = "TP-LINK";        // 设置Wi-Fi名称
const char* password = "12345678";            // 设置Wi-Fi密码


// 定义 OLED 显示屏的宽度和高度，适用于 128x32 OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
// 定义 I2C 引脚
#define OLED_RESET -1  // 复位引脚（没有使用）
#define SCREEN_ADDRESS 0x3C  // I2C 地址（大多数 OLED 显示屏地址为 0x3C）

// 声明一个 OLED 显示屏对象
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
int progress = 0;

//ros
IPAddress server(192, 168, 85, 105);

uint16_t serverPort = 11411;

bool publish_flag = false;
ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("esp32_json_string", &str_msg);

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // 将引脚设置为上拉输入模式
    pinMode(js_button, INPUT_PULLUP);   
    pinMode(button_down, INPUT_PULLUP);
    pinMode(button_up, INPUT_PULLUP);

    // 初始化 I2C 总线
    Wire.begin(SDA_PIN, SCL_PIN);
    // 初始化显示屏
    if (!display.begin(SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for (;;);
    }

    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();

    // Another way to get IP
    Serial.print("ROS IP = ");
    Serial.println(nh.getHardware()->getLocalIP());

    // Start to be polite
    nh.advertise(chatter);

    // 清除缓冲区
    display.clearDisplay();
    while(progress < 121)
    {    
      // 显示一些文本
      display.setTextSize(1.5);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println(F("    AgileX Robotics"));
      display.setTextSize(1);
      display.setCursor(0, 20);
      display.println(F("     Spatial Teleop"));

      display.display();

      // 显示进度条边框
      display.drawRoundRect(0, 10, 128, 10, 5, SSD1306_WHITE);
      // 显示进度
      display.fillRoundRect(4, 13, progress, 4, 3, SSD1306_WHITE);


      // 刷新屏幕
      display.display();
      // delay(2); // 延迟一段时间后更新显示
      progress+=2;
    }
    display.clearDisplay();
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
        display.clearDisplay();
        display.setTextSize(1.5);
        display.setCursor(0, 0);
        display.println("Waiting for WiFi: ");
        display.setCursor(0, 15);
        display.println(ssid);
        display.display();

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
            display.setCursor(0, 20);
            display.printf("IP: %s", ipString.c_str());
            display.display();
            firstConnect = false;
        }
        
        if (currentMillis - previousMillis >= 40) { // 每20毫秒发送一次消息，相当于50Hz
          previousMillis = currentMillis;

          // 读取引脚状态，引脚上拉输入，所以进行逻辑反转
          int js_state = 1 - digitalRead(js_button);  
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
            doc["js_button"] = js_state;
            doc["up_button"] = button_state_up;
            doc["down_button"] = button_state_down;
            doc["hall_adc"] = adcValue_hall;
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


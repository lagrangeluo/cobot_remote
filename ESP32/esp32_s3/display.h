/*
 * display.h
 *
 * Created on: 2024
 * Description:
 * lcd屏幕显示相关的文件，显示屏是128X32的。
 * 主要功能：
 * 1. 初始化i2c总线，加载虚拟进度条，主要为了美观
 * 2. 显示电压数值，电池信息，wifi连接状态等关键信息
 * Copyright (c) 2024 AgileX Robotics
 */

//iic
#include <Wire.h>
//屏幕驱动
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// 定义 OLED 显示屏的宽度和高度，适用于 128x32 OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
// 定义 I2C 引脚
#define SDA_PIN     5
#define SCL_PIN     6
#define OLED_RESET -1  // 复位引脚（没有使用）
#define SCREEN_ADDRESS 0x3C  // I2C 地址（大多数 OLED 显示屏地址为 0x3C）

// 定义电池图标的大小和位置
#define BATTERY_WIDTH 20
#define BATTERY_HEIGHT 10
#define BATTERY_X (SCREEN_WIDTH - BATTERY_WIDTH - 2)  // 距离右下角 2 像素
#define BATTERY_Y (SCREEN_HEIGHT - BATTERY_HEIGHT - 2)
// 电池电量百分比
float batteryPercentage = 75.0;  // 假设电池百分比，实际可以根据电压值计算

// 声明一个 OLED 显示屏对象
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
// 显示的电池电压信息
char* battery_info = (char*)malloc(32 * sizeof(char));

void init_display()
{
    // 初始化 I2C 总线
    Wire.begin(SDA_PIN, SCL_PIN);
    // 初始化显示屏
    if (!display.begin(SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for (;;);
    }
    // 清除缓冲区
    display.clearDisplay();
    // 虚拟进度条
    static int progress = 0;
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

//显示wifi的连接状态
void display_wifi_status(char* ssid,char* hand_name)
{
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
}

//显示电压的数值
void display_battery()
{
  // 设置文本大小
  display.setTextSize(1);
  // 将光标移到右下角
  display.setCursor(SCREEN_WIDTH - 10 * 2, SCREEN_HEIGHT - 8); // 10 * 2是大概的数字宽度，8是行高
  // 显示数字
  display.print(battery_info);
  // 刷新显示屏
  display.display();
}

// 显示电池电量ui
void displayBatteryIcon(float batteryPercentage) {
  // 绘制电池外壳
  display.drawRect(BATTERY_X, BATTERY_Y, BATTERY_WIDTH, BATTERY_HEIGHT, SSD1306_WHITE);

  // 绘制电池头部
  display.drawRect(BATTERY_X + BATTERY_WIDTH, BATTERY_Y + 3, 3, 4, SSD1306_WHITE);

  // 计算电量格数
  int numBars = (batteryPercentage / 100.0) * 4;  // 四格电量

  // 填充电量格
  for (int i = 0; i < numBars; i++) {
    display.fillRect(BATTERY_X + 2 + i * 4, BATTERY_Y + 2, 3, BATTERY_HEIGHT - 4, WHITE);
  }
}
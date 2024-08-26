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

// 声明一个 OLED 显示屏对象
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
int progress = 0;

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

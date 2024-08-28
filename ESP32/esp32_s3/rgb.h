// rgb相关外设

#include "Freenove_WS2812_Lib_for_ESP32.h"

#define LEDS_COUNT  8
#define LEDS_PIN	8
#define CHANNEL		0

Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL);

void init_rgb() {

  // XIAO esp32 板子自带的led灯
  pinMode(LED_BUILTIN, OUTPUT);
  // 遥操作手柄外接的rgb灯珠初始化
  strip.begin();
  strip.setBrightness(20);
  // 初始颜色为白色
  strip.setLedColorData(0, 255, 255, 255);
  strip.show();

}

void set_rgb_black(){
    strip.setLedColorData(0, 0, 0, 0);
    strip.show();
}
void set_rgb_white(){
    strip.setLedColorData(0, 255, 255, 255);
    strip.show();
}
void set_rgb_red(){
    strip.setLedColorData(0, 255, 0, 0);
    strip.show();
}
void set_rgb_green(){
    strip.setLedColorData(0, 0, 255, 0);
    strip.show();
}
void set_rgb_blue(){
    strip.setLedColorData(0, 0, 0, 255);
    strip.show();
}
void set_rgb_yellow(){
    strip.setLedColorData(0, 255, 255, 0);
    strip.show();
}
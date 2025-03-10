/*
 * gpio.h
 * author: Junyu Luo
 * Created on: 2025
 * Description:
 * 包含XIAO esp32 s3工程的gpio部分
 *
 * Copyright (c) 2025 MagicCube Robotics
 */


 // 两个按键，up和down是历史遗留名字
#define button_up   4
#define button_down 43
// #define hall_adc    7
#define Bat_sample  9

void init_gpio()
{
    // 将引脚设置为上拉输入模式
    pinMode(button_down, INPUT_PULLUP);
    pinMode(button_up, INPUT_PULLUP);
}
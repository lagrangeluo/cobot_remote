/*
 * wifi.h
 * author: Junyu Luo
 * Created on: 2025
 * Description:
 * 包含XIAO esp32 s3工程的wifi部分，包含启动wifi，通过wifi远程ota等功能
 *
 * Copyright (c) 2025 MagicCube Robotics
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <AsyncWebSocket.h>
//OTA
#include <ArduinoOTA.h>

static char* ssid = (char*)malloc(32 * sizeof(char));        // 设置Wi-Fi名称
static char* password = (char*)malloc(32 * sizeof(char));    // 设置Wi-Fi密码

void init_wifi()
{
    // 连接wifi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    // 初始化远程OTA升级，初始化后可以支持wifi烧录
    // ArduinoOTA.setPort(3232);  // OTA端口号，就这样固定
    // ArduinoOTA.begin();
}
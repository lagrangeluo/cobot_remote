/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/
// #define USE_USBCON
// #define CONFIG_FREERTOS_HZ 1000
// #define CONFIG_ARDUINO_LOOP_STACK_SIZE 8192
//json字符串解析
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ros.h>
#include <std_msgs/String.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// 创建 ROS 句柄
ros::NodeHandle nh;
// 创建 ROS 话题
std_msgs::String ros_msg;
ros::Publisher json_pub("json_data", &ros_msg);


// 解析 JSON 数据
void parseAndPublishJson(const std::string &jsonData) {
  StaticJsonDocument<200> doc;  // 根据 JSON 大小调整
  DeserializationError error = deserializeJson(doc, jsonData);

  // 如果解析出现错误，说明json字符串丢包，则跳过此条消息
  if (error) {
    // Serial.println("JSON 解析失败");
    return;
  }
  // 将json字符串发布到ROS话题
  ros_msg.data = jsonData.c_str();
  json_pub.publish(&ros_msg);
}

// 继承 BLECharacteristicCallbacks 类来处理客户端写入数据的回调
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    // 获取客户端写入的值
    std::string value = pCharacteristic->getValue().c_str();
    // 解析 JSON
    StaticJsonDocument<200> doc;  // 根据 JSON 大小调整
    DeserializationError error = deserializeJson(doc, value);
    if (error) {
      //Serial.println("JSON 解析失败，数据可能不完整");
    } else {
      // 解析json字符串并发布到ROS
      parseAndPublishJson(value);
    }
  }
};

void setup() {
  Serial.begin(115200);
  //ros node初始化
  nh.getHardware()->setPort((HardwareSerial*)&Serial);  // 确保使用 USB-CDC 串口
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(json_pub);
  // Serial.println("ESP32 ROS 话题发布器已启动");

  BLEDevice::init("UMI-Air-Server");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic =
    pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | 
                                                        BLECharacteristic::PROPERTY_WRITE |
                                                        BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristic->setValue("Hello World says Neil");
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());  // 设置回调函数

  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();  // 处理 ROS 消息队列
  delay(10);
}

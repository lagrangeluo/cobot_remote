/*
 * ble.h
 * author: Junyu Luo
 * Created on: 2025
 * Description:
 * 包含XIAO esp32 s3工程的蓝牙BLE部分
 *
 * Copyright (c) 2025 MagicCube Robotics
 */

#include "BLEDevice.h"
//#include "BLEScan.h"

// The remote service we wish to connect to.
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

// 标志位，表示开始一次蓝牙连接
static boolean doConnect = false;
// 标志位，表示当前的蓝牙连接状态
static boolean connected = false;
// 标志位，表示开始蓝牙扫描
static boolean doScan = false;
// 蓝牙Characteristic特征值
static BLERemoteCharacteristic *pRemoteCharacteristic;
// BLE蓝牙的client设备实例指针
static BLEAdvertisedDevice *myDevice;

// 使用蓝牙设备的主任务回调函数
typedef String (* BLEClientCallbackFuncPtr)(void);
// 蓝牙设备断联后的回调函数
typedef void (* BLEClientErrorFuncPtr)(void);

static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
  Serial.print("Notify callback for characteristic ");
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient *pclient) {}

  void onDisconnect(BLEClient *pclient) {
    // 连接标志位置false，表示已经断开连接
    connected = false;
    // 扫描标志位置true，开始启动蓝牙扫描
    doScan = true;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient *pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");
  pClient->setMTU(517);  //set client to request maximum MTU from server (default is 23 otherwise)

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  // Read the value of the characteristic.
  if (pRemoteCharacteristic->canRead()) {
    String value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  }

  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
  }

  connected = true;
  return true;
}

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      // 标志位置位，触发蓝牙连接
      doConnect = true;
      // 搜索到目标设备，停止搜索
      doScan = false;
    }  // Found our server
  }  // onResult
};  // MyAdvertisedDeviceCallbacks

// 开始扫描蓝牙设备，如果有扫描到了目标设备，则doConnect标志位置True，开始连接该设备
void start_scan() {
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan *pBLEScan = BLEDevice::getScan();
  // 清除之前的扫描结果
  pBLEScan->clearResults();
  // 注册扫描到设备后的回调函数
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(3, true);
  // 如果扫描完成后没有匹配到目标设备，继续扫描
  if (doConnect == false)
  {
    doScan = true;
  }
}

void init_BLE() {
  BLEDevice::init("");
  Serial.println("Starting Arduino BLE Client application...");
  start_scan();
  Serial.println("Finished BLE Client Init");
}

// 蓝牙设备循环函数，实现以下功能：
// 1.不断循环检查蓝牙连接是否异常，如果有异常则重新扫描并连接
// 2.蓝牙连接正常后，运行callback回调功能函数
void BLEDeviceLoop(BLEClientCallbackFuncPtr callback, BLEClientErrorFuncPtr error_handle, int time_interval) {

  // 执行蓝牙连接程序的标识符，一次连接只需要运行一次
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothing more we will do.");
    }
    doConnect = false;
  }

  // 当蓝牙连接状态正常，执行回调函数
  if (connected) {
    // 执行回调函数
    String json_string = callback();
    // 通信循环测试
    // String newValue = "Timcount: 123");
    Serial.println("Json value to \"" + json_string + "\"");
    // // 向服务端写入数值
    pRemoteCharacteristic->writeValue(json_string.c_str(), json_string.length());
  }
  // 当蓝牙断开连接，执行重新扫描函数 
  else if (doScan) {
    Serial.println("start scan for bluetooth device");
    start_scan();
    error_handle();
  }
  delay(time_interval);  // Delay a second between loops.
}  // End of loop

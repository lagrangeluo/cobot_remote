/*
 * nvs_user.h
 *
 * Created on: 2024
 * Description:
 * nvs存储相关配置，当前版本的文件系统的内容做不到随用随存，所以将静态存储的参数全部放在nvs存储空间中
 * 主要功能：
 * 1. 初始化nvs相关
 * 2. 读写操作函数，读取到的都是json格式的字符串，然后再进行序列化或者反序列化
 * Copyright (c) 2024 AgileX Robotics
 */
#include "nvs_flash.h"
#include "nvs.h"

nvs_handle_t my_handle;
esp_err_t ret = nvs_flash_init();

void write_init_nvs_data()
{
    //打开 NVS 命名空间
    ret = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (ret != ESP_OK) {
        Serial.println("Error opening NVS handle!");
    } else {
        // 创建 JSON 对象
        StaticJsonDocument<256> doc;
        doc["ssid"] = "TP-LINK";
        doc["password"] = "12345678";

        // 序列化 JSON 对象到字符串
        char json_string[256];
        serializeJson(doc, json_string);

        // 将 JSON 字符串存储到 NVS
        ret = nvs_set_str(my_handle, "wifi_config", json_string);
        if (ret == ESP_OK) {
            Serial.println("WiFi config stored successfully!");
        } else {
            Serial.printf("Failed to store WiFi config (%s)\n", esp_err_to_name(ret));
        }

        // 确保数据已写入 NVS
        ret = nvs_commit(my_handle);
        if (ret != ESP_OK) {
            Serial.printf("Failed to commit updates in NVS (%s)\n", esp_err_to_name(ret));
        }

        // 关闭 NVS
        nvs_close(my_handle);
    }
}

bool write_nvs_data(char* buffer)
{
  // 打开 NVS 命名空间
  ret = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (ret != ESP_OK) {
      return false;
  } else {
      // 将 JSON 字符串存储到 NVS
      nvs_set_str(my_handle, "wifi_config", (char*) buffer);
      if (ret != ESP_OK) {
          return false;
      }
      // 确保数据已写入 NVS
      ret = nvs_commit(my_handle);
      if (ret != ESP_OK) {
          return false;
      }
      // 关闭 NVS
      nvs_close(my_handle);
  }
}

//从nvs读取wifi名字密码，并传递给参数指针
void read_nvs_data(char* ssid,char* password,char* ros_ip)
{
  // 打开 NVS 命名空间
  ret = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (ret == ESP_OK) {
      // 读取 JSON 字符串的长度
      size_t required_size;
      ret = nvs_get_str(my_handle, "wifi_config", NULL, &required_size);

      if (ret == ESP_OK) {
          // 分配足够的内存来存储 JSON 字符串
          char* json_string = (char*)malloc(required_size);
          ret = nvs_get_str(my_handle, "wifi_config", json_string, &required_size);

          if (ret == ESP_OK) {
              // 反序列化 JSON 字符串
              StaticJsonDocument<256> doc;
              DeserializationError error = deserializeJson(doc, json_string);

              if (!error) {
                if(doc["ssid"] != "")
                {
                  const char* ssid_nvs = doc["ssid"];
                  strcpy(ssid, ssid_nvs);
                }
                if(doc["password"] != "")
                {
                  const char* password_nvs = doc["password"];
                  strcpy(password, password_nvs);
                }
                if(doc["ros_master_ip"] != "")
                {
                  const char* ros_master_ip_nvs = doc["ros_master_ip"];
                  strcpy(ros_ip,ros_master_ip_nvs);
                }
              } else {
                  Serial.println("Failed to deserialize JSON");
              }
          } else {
              Serial.printf("Failed to read WiFi config (%s)\n", esp_err_to_name(ret));
          }
          free(json_string);
      } else {
          Serial.println("No WiFi config found!");
      }
      nvs_close(my_handle);
  }
}

//从nvs中读取"hand"标签的数据，该标签有“left”和“right”,分别代表左手和右手
void read_nvs_hand(char* hand_name)
{
  // 打开 NVS 命名空间
  ret = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (ret == ESP_OK) {
      // 读取 JSON 字符串的长度
      size_t required_size;
      ret = nvs_get_str(my_handle, "wifi_config", NULL, &required_size);

      if (ret == ESP_OK) {
          // 分配足够的内存来存储 JSON 字符串
          char* json_string = (char*)malloc(required_size);
          ret = nvs_get_str(my_handle, "wifi_config", json_string, &required_size);

          if (ret == ESP_OK) {
              // 反序列化 JSON 字符串
              StaticJsonDocument<256> doc;
              DeserializationError error = deserializeJson(doc, json_string);

              if (!error && doc["hand"] != "") {
                  const char* hand_nvs = doc["hand"];
                  strcpy(hand_name, hand_nvs);
              } else {
                  Serial.println("Failed to deserialize JSON");
              }
          } else {
              Serial.printf("Failed to read WiFi config (%s)\n", esp_err_to_name(ret));
          }
          free(json_string);
      }
      nvs_close(my_handle);
  }
}

//判定nvs是否初始化成功
void init_nvs()
{
    // 如果NVS分区中没有空闲页面或者找不到NVS版本，可能需要擦除并重新初始化NVS
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());  // 擦除NVS分区
        ret = nvs_flash_init();              // 重新初始化NVS
    }
    ESP_ERROR_CHECK(ret);  // 检查是否初始化成功
}
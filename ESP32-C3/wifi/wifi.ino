#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ros.h>
#include <std_msgs/String.h>

const char* ssid = "ESP32_Server";
const char* password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    data[len] = 0; // Ensure null-terminated string
    // Serial.printf("Data received: %s\n", (char*)data);
  }
}

void setup() {
  Serial.begin(115200);
  // Serial.println("start init WiFi softAP");
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  // Serial.print("AP IP address: ");
  // Serial.println(IP);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

  nh.initNode();
  nh.advertise(chatter);
}

void loop() {
  // The loop is empty because the server and WebSocket are handled asynchronously
  // Serial.println("test");
  str_msg.data = "Hello, ROS!";
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(1000);
}

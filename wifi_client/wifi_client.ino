#include <WiFi.h>
#include <WebSocketsClient.h>

const char* ssid = "ESP32_Server";
const char* password = "12345678";
const char* host = "192.168.4.1";  // Server IP address

WebSocketsClient webSocket;

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("Disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("Connected");
      break;
    case WStype_TEXT:
      Serial.printf("Received text: %s\n", payload);
      break;
    case WStype_BIN:
      Serial.println("Received binary data");
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");

  webSocket.begin(host, 80, "/ws");
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  static unsigned long lastSendTime = 0;
  const int sendInterval = 20;  // 20 milliseconds for 50Hz

  webSocket.loop();

  if (millis() - lastSendTime >= sendInterval) {
    lastSendTime = millis();
    webSocket.sendTXT("Hello from ESP32 Client!");
  }
}

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#define MOTOR_PIN 5   // motor rung khi có cảnh báo

// --- WiFi ---
const char* ssid = "DESKTOP-FTLL525 2479";
const char* password = "9z7-6B47";

// --- MQTT ThingSpeak ---
const char* mqttServer = "mqtt3.thingspeak.com";
const int mqttPort = 8883;
const char* mqttClientID = "ARg9Ji80OgApFyUNOAAJBAg";
const char* mqttUser     = "ARg9Ji80OgApFyUNOAAJBAg";
const char* mqttPass     = "Erv6WVgSdsCDx+s33T+DMvBq";
const char* channelID    = "3111657";

// --- UDP ---
const int udpPort = 8888;
WiFiUDP udp;

WiFiClientSecure espClient;
PubSubClient client(espClient);

void reconnectMQTT();

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  // WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // MQTT
  espClient.setInsecure();
  client.setServer(mqttServer, mqttPort);

  // UDP
  udp.begin(udpPort);
  Serial.print("Listening for UDP packets on port ");
  Serial.println(udpPort);
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("MQTT reconnecting...");
    if (client.connect(mqttClientID, mqttUser, mqttPass)) {
      Serial.println("Connected!");
    } else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();

  // --- Nhận dữ liệu UDP ---
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char packetBuffer[255];
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = '\0';

    String data = String(packetBuffer);
    Serial.println("Received: " + data);

    // Parse "nodeId,temp"
    int commaIndex = data.indexOf(',');
    if (commaIndex != -1) {
      int nodeId = data.substring(0, commaIndex).toInt();
      float temp = data.substring(commaIndex + 1).toFloat();

      Serial.printf("Node %d → Temp: %.2f°C\n", nodeId, temp);

      // Nếu vượt ngưỡng thì rung
      if (temp > 37.5) {
        digitalWrite(MOTOR_PIN, HIGH);
        delay(1000);
        digitalWrite(MOTOR_PIN, LOW);
        Serial.println("High temperature detected!");
      }

      // --- Gửi MQTT lên ThingSpeak ---
      String topic = "channels/" + String(channelID) + "/publish";
      String payload = "field1=" + String(temp) + "&field2=" + String(nodeId);

      if (client.publish(topic.c_str(), payload.c_str())) {
        Serial.println("Published: " + payload);
      } else {
        Serial.println("Publish failed!");
      }
    }
  }

  delay(2000);
}

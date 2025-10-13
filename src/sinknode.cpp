#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#define MOTOR_PIN 5   // Motor rung khi cảnh báo

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

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  // --- WiFi ---
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // --- MQTT ---
  espClient.setInsecure();
  client.setServer(mqttServer, mqttPort);

  // --- UDP ---
  udp.begin(udpPort);
  Serial.print("Listening UDP on port ");
  Serial.println(udpPort);
}

void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();

  int packetSize = udp.parsePacket();
  if (packetSize) {
    char packetBuffer[255];
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = '\0';

    String data = String(packetBuffer);
    Serial.println("Received: " + data);

    // --- Tách dữ liệu 13 giá trị ---
    float values[13] = {0};
    int index = 0;
    int lastComma = -1;
    for (int i = 0; i <= data.length(); i++) {
      if (data[i] == ',' || i == data.length()) {
        String val = data.substring(lastComma + 1, i);
        values[index++] = val.toFloat();
        lastComma = i;
        if (index >= 13) break;
      }
    }

    int nodeId = (int)values[0];
    float tempC = values[1];
    float accX = values[2], accY = values[3], accZ = values[4];
    float gyroX = values[5], gyroY = values[6], gyroZ = values[7];
    float angleX = values[8], angleY = values[9], angleZ = values[10];
    float hr = values[11], spo2 = values[12];

    // --- Hiển thị Serial ---
    Serial.printf("Node %d\n", nodeId);
    Serial.printf("Temp: %.2f°C\n", tempC);
    Serial.printf("Acc: %.2f, %.2f, %.2f\n", accX, accY, accZ);
    Serial.printf("Gyro: %.2f, %.2f, %.2f\n", gyroX, gyroY, gyroZ);
    Serial.printf("Angle: %.2f, %.2f, %.2f\n", angleX, angleY, angleZ);
    Serial.printf("HR: %.0f, SpO2: %.0f\n", hr, spo2);
    Serial.println("------------------------------------");

    // --- Rung cảnh báo ---
    if (tempC > 37.5) {
      digitalWrite(MOTOR_PIN, HIGH);
      delay(1000);
      digitalWrite(MOTOR_PIN, LOW);
      Serial.println(" Nhiệt độ cao!");
    }

    // --- Publish 7 field lên ThingSpeak ---
    String topic = "channels/" + String(channelID) + "/publish";
    String payload = "field1=" + String(nodeId) +
                     "&field2=" + String(tempC) +
                     "&field3=" + String(accX) + "," + String(accY) + "," + String(accZ) +
                     "&field4=" + String(gyroX) + "," + String(gyroY) + "," + String(gyroZ) +
                     "&field5=" + String(angleX) + "," + String(angleY) + "," + String(angleZ) +
                     "&field6=" + String(hr) +
                     "&field7=" + String(spo2);

    if (client.publish(topic.c_str(), payload.c_str())) {
      Serial.println("Published to ThingSpeak: " + payload);
    } else {
      Serial.println("Publish failed!");
    }
  }

  delay(2000);
}

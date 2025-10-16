#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include "dense_autoencoder_esp32_int8.h"
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/schema/schema_generated.h>

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

// --- MQTT client ---
WiFiClientSecure espClient;
PubSubClient client(espClient);

// --- TensorFlow Lite ---
constexpr int kTensorArenaSize = 16 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

const tflite::Model* model_tflite = tflite::GetModel(dense_autoencoder_esp32_int8_tflite);
tflite::AllOpsResolver resolver;             
tflite::MicroInterpreter* interpreter;
tflite::MicroErrorReporter micro_error_reporter;

TfLiteTensor* input;
TfLiteTensor* output;

// --- Parameters ---
#define TIME_STEPS 10
#define N_FEATURES 11
const float MSE_THRESHOLD = 0.0008969453;

float sequence_buffer[TIME_STEPS][N_FEATURES];
int seq_index = 0;

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

void setupTFLite() {
  static tflite::MicroInterpreter static_interpreter(
    model_tflite,
    resolver,
    tensor_arena,
    kTensorArenaSize,
    &micro_error_reporter
  );
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("AllocateTensors() failed!");
    while (1);
  }

  input = interpreter->input(0);
  output = interpreter->output(0);
  Serial.println("TFLite setup done!");
}

float compute_mse() {
  float mse = 0;
  for (int i = 0; i < TIME_STEPS; i++) {
    for (int j = 0; j < N_FEATURES; j++) {
      int8_t q = output->data.int8[i * N_FEATURES + j];
      float recon = (q - output->params.zero_point) * output->params.scale;
      float diff = sequence_buffer[i][j] - recon;
      mse += diff * diff;
    }
  }
  mse /= (TIME_STEPS * N_FEATURES);
  return mse;
}

void process_sequence(float nodeId, float temp, float accX, float accY, float accZ,
                      float gyroX, float gyroY, float gyroZ,
                      float angleX, float angleY, float angleZ,
                      float hr) {
  // Convert float -> int8 input
  for (int i = 0; i < TIME_STEPS; i++) {
    for (int j = 0; j < N_FEATURES; j++) {
      int8_t q = (int8_t)(sequence_buffer[i][j] / input->params.scale + input->params.zero_point);
      if (q > 127) q = 127;
      if (q < -128) q = -128;
      input->data.int8[i * N_FEATURES + j] = q;
    }
  }

  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("Invoke failed!");
    return;
  }

  float mse = compute_mse();
  bool alert = mse > MSE_THRESHOLD;

  // Rung motor nếu cảnh báo
  digitalWrite(MOTOR_PIN, alert ? HIGH : LOW);
  if (alert) Serial.println("ALERT: Anomaly detected!");

  // --- Publish lên ThingSpeak ---
  String topic = "channels/" + String(channelID) + "/publish";
  String payload = "field1=" + String(nodeId) +
                   "&field2=" + String(temp, 2) +
                   "&field3=" + String(accX, 2) + "," + String(accY, 2) + "," + String(accZ, 2) +
                   "&field4=" + String(gyroX, 2) + "," + String(gyroY, 2) + "," + String(gyroZ, 2) +
                   "&field5=" + String(angleX, 2) + "," + String(angleY, 2) + "," + String(angleZ, 2) +
                   "&field6=" + String(hr, 0) +
                   "&field7=" + String(mse, 6) +
                   "&field8=" + String(alert ? 1 : 0);

  if (client.publish(topic.c_str(), payload.c_str())) {
    Serial.println("Published: " + payload);
  } else {
    Serial.println("Publish failed!");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  // WiFi
  Serial.println("Connecting WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected! IP: " + String(WiFi.localIP()));

  // MQTT
  espClient.setInsecure();
  client.setServer(mqttServer, mqttPort);

  // UDP
  udp.begin(udpPort);
  Serial.printf("Listening UDP on port %d\n", udpPort);

  // TFLite
  setupTFLite();
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

    // Parse 12 values: nodeId + 11 features
    float values[12] = {0};
    int index = 0;
    int lastComma = -1;
    for (int i = 0; i <= data.length(); i++) {
      if (data[i] == ',' || i == data.length()) {
        String val = data.substring(lastComma + 1, i);
        values[index++] = val.toFloat();
        lastComma = i;
        if (index >= 12) break;
      }
    }

    // Add to sequence buffer
    for (int j = 0; j < N_FEATURES; j++) {
      sequence_buffer[seq_index][j] = values[j + 1]; // skip nodeId
    }
    seq_index++;

    if (seq_index >= TIME_STEPS) {
      process_sequence(values[0], values[1], values[2], values[3], values[4],
                       values[5], values[6], values[7], values[8], values[9], values[10], values[11]);

      // shift left buffer 1 step
      for (int i = 1; i < TIME_STEPS; i++)
        for (int j = 0; j < N_FEATURES; j++)
          sequence_buffer[i - 1][j] = sequence_buffer[i][j];
      seq_index = TIME_STEPS - 1;
    }
  }
}

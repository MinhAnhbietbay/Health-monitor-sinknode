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
constexpr int kTensorArenaSize = 32 * 1024;  // tăng để tránh thiếu RAM
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
float MSE_THRESHOLD = 0.0008080511; // ngưỡng cảnh báo ban đầu

float sequence_buffer[TIME_STEPS][N_FEATURES];
int seq_index = 0;

// --- Min & Max của 11 đặc trưng
const float feature_mins[N_FEATURES] = {
  32.5, -0.140388, -2.000102, -1.2661167,
  -3.598768, -2.1705351, -3.161072,
  -179.99988, -89.971024, -179.99937,
  92.307678
};

const float feature_maxs[N_FEATURES] = {
  33.689999, 1.5592226, 0.32112229, 0.1557696,
  4.330781, 2.648262, 2.375591,
  179.99944, 89.486946, 179.99696,
  200.00003
};

// --- Calibration MSE ---
float max_mse_normal = 0;
bool calibrating = true;
unsigned long calibrate_start = 0;  // thời gian bắt đầu calibrate

// --- Functions ---
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
  int count = 0;
  for (int i = 0; i < TIME_STEPS; i++) {
    for (int j = 0; j < N_FEATURES; j++) {
      if (j == 9) continue;               // bỏ angleZ
      if (j == 10 && sequence_buffer[i][j] < -900) continue; // bỏ hr bị mất
      int8_t q = output->data.int8[i * N_FEATURES + j];
      float recon = (q - output->params.zero_point) * output->params.scale;
      float diff = sequence_buffer[i][j] - recon;
      mse += diff * diff;
      count++;
    }
  }
  if (count == 0) return 0;
  return mse / count;
}

void process_sequence(float nodeId, float raw_values[N_FEATURES]) {
  // Chuyển góc sang rad
  for (int i = 0; i < TIME_STEPS; i++) {
      sequence_buffer[i][8] = sequence_buffer[i][8] * 3.14159265358979323846 / 180.0; // angleX
      sequence_buffer[i][9] = sequence_buffer[i][9] * 3.14159265358979323846 / 180.0; // angleY
  }
  
  // Chuẩn hóa
  float normalized_buffer[TIME_STEPS][N_FEATURES];
  for (int i = 0; i < TIME_STEPS; i++) {
    for (int j = 0; j < N_FEATURES; j++) {
      float scaled = (sequence_buffer[i][j] - feature_mins[j]) / (feature_maxs[j] - feature_mins[j]);
      if (scaled < 0) scaled = 0;
      if (scaled > 1) scaled = 1;
      normalized_buffer[i][j] = scaled;
    }
  }

  // Float -> int8
  for (int i = 0; i < TIME_STEPS; i++) {
    for (int j = 0; j < N_FEATURES; j++) {
      int8_t q = (int8_t)(normalized_buffer[i][j] / input->params.scale + input->params.zero_point);
      if (q > 127) q = 127;
      if (q < -128) q = -128;
      input->data.int8[i * N_FEATURES + j] = q;
    }
  }

  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("Invoke failed!");
    return;
  }

  memcpy(sequence_buffer, normalized_buffer, sizeof(normalized_buffer));
  float mse = compute_mse();

  // --- Calibrate ---
  if (calibrating) {
    if (mse > max_mse_normal) max_mse_normal = mse;
  }

  bool alert = false;
  if (!calibrating) alert = mse > MSE_THRESHOLD;

  // Rung motor nếu cảnh báo
  digitalWrite(MOTOR_PIN, alert ? HIGH : LOW);
  if (alert) Serial.println("ALERT: Anomaly detected!");

  // --- Publish dữ liệu ---
  String topic = "channels/" + String(channelID) + "/publish";
  String payload = "field1=" + String(nodeId) +
                   "&field2=" + String(raw_values[0], 2) +
                   "&field3=" + String(raw_values[1], 2) + "," + String(raw_values[2], 2) + "," + String(raw_values[3], 2) +
                   "&field4=" + String(raw_values[4], 2) + "," + String(raw_values[5], 2) + "," + String(raw_values[6], 2) +
                   "&field5=" + String(raw_values[7], 2) + "," + String(raw_values[8], 2) + "," + String(raw_values[9], 2) +
                   "&field6=" + String(raw_values[10], 0) +
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

  // Bắt đầu calibrate
  calibrate_start = millis();
  Serial.println("Calibration started: collecting normal data for MSE threshold...");
}

void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();

  // --- Kiểm tra kết thúc calibrate ---
  if (calibrating && millis() - calibrate_start > 120000) { // 2 phút
    calibrating = false;
    Serial.print("Calibration done. Max MSE observed: ");
    Serial.println(max_mse_normal, 6);
    MSE_THRESHOLD = max_mse_normal * 1.1; // threshold = max MSE * 1.1
    Serial.print("Set new MSE threshold: ");
    Serial.println(MSE_THRESHOLD, 6);
  }

  int packetSize = udp.parsePacket();  if (packetSize) {
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

    // Lưu dữ liệu thô vào buffer
    for (int j = 0; j < N_FEATURES; j++) {
      sequence_buffer[seq_index][j] = values[j + 1];
    }
    seq_index++;

    if (seq_index >= TIME_STEPS) {
      float raw_vals[N_FEATURES];
      for (int j = 0; j < N_FEATURES; j++) raw_vals[j] = values[j + 1];
      process_sequence(values[0], raw_vals);

      // shift left buffer 1 step
      for (int i = 1; i < TIME_STEPS; i++)
        for (int j = 0; j < N_FEATURES; j++)
          sequence_buffer[i - 1][j] = sequence_buffer[i][j];
      seq_index = TIME_STEPS - 1;
    }
  }
}

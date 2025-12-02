/*
 * Smart Garden - Board 6: Environmental Monitor with I2C/SPI Communication (ESP32)
 * Chức năng: Giám sát môi trường tổng hợp, giao tiếp I2C/SPI với thiết bị khác
 * Áp dụng: Multiple I2C/SPI devices, Environmental sensors, Data aggregation
 * Môi trường: Wokwi simulation
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>

// WiFi Configuration
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// MQTT Configuration
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* CLIENT_ID = "ESP_BOARD6_ENV_MONITOR_01";
const char* PUBLISH_TOPIC = "garden/sensor/environment";
const char* SUBSCRIBE_TOPIC = "garden/control/env_";

// Hardware Configuration - Board 6
#define LDR_PIN 32
#define TEMP_PIN 4
#define SOIL_PIN 35
#define SOIL_TEMP_PIN 34
#define BUTTON_MODE_PIN 25
#define LED_STATUS_PIN 2
#define LED_DATA_PIN 12

// I2C Configuration - Multiple devices
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_DEVICE1_ADDR 0x48  // Temperature sensor
#define I2C_DEVICE2_ADDR 0x49  // Humidity sensor
#define I2C_DEVICE3_ADDR 0x50  // Light sensor
#define I2C_DEVICE4_ADDR 0x51  // Pressure sensor

// SPI Configuration
#define SPI_MOSI_PIN 23
#define SPI_MISO_PIN 19
#define SPI_SCK_PIN 18
#define SPI_CS_PIN 5

// WatchDog Timer
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 20

// MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;

// Global Variables - Sensor readings
float temperature = 0.0;
float humidity = 0.0;
float lightLevel = 0.0;
float pressure = 0.0;
float soilMoisture = 0.0;
float soilTemperature = 0.0;
float airQuality = 0.0;

// I2C Communication variables
String receivedI2CData = "";
String receivedSPIData = "";
unsigned long lastI2CCommunication = 0;
unsigned long lastSPICommunication = 0;

// Mode variables
volatile bool buttonModePressed = false;
int currentMode = 0; // 0=Monitor, 1=Data Logger, 2=Communication Hub
bool dataLogging = false;
unsigned long loggingStartTime = 0;

// Data aggregation
struct EnvironmentData {
  float temp, humidity, light, pressure, soilMoist, soilTemp, airQuality;
  unsigned long timestamp;
  String source;
};

// Buffer để lưu trữ dữ liệu tạm thời
#define DATA_BUFFER_SIZE 10
EnvironmentData dataBuffer[DATA_BUFFER_SIZE];
int bufferIndex = 0;
int bufferCount = 0;

void IRAM_ATTR buttonModeISR() {
  buttonModePressed = true;
}

void setup_wifi() {
  delay(10);
  WiFi.begin(ssid, password);
  
  Serial.print("WiFi kết nối: " + String(ssid));
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 60) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("✅ WiFi kết nối thành công!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("❌ WiFi kết nối thất bại");
  }
}

void setup_i2c() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  
  // Test I2C devices
  testI2CDevices();
  
  Serial.println("✅ I2C initialized");
}

void setup_spi() {
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN);
  pinMode(SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  
  Serial.println("✅ SPI initialized");
}

void setup_interrupts() {
  pinMode(BUTTON_MODE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_MODE_PIN), buttonModeISR, FALLING);
  
  Serial.println("✅ Interrupts initialized");
}

void testI2CDevices() {
  Serial.println("Testing I2C devices...");
  
  // Test Device 1 - Temperature sensor
  if (testI2CDevice(I2C_DEVICE1_ADDR)) {
    Serial.println("✅ I2C Device 1 (0x48) - Temperature sensor: OK");
  } else {
    Serial.println("❌ I2C Device 1 (0x48) - Temperature sensor: FAILED");
  }
  
  // Test Device 2 - Humidity sensor
  if (testI2CDevice(I2C_DEVICE2_ADDR)) {
    Serial.println("✅ I2C Device 2 (0x49) - Humidity sensor: OK");
  } else {
    Serial.println("❌ I2C Device 2 (0x49) - Humidity sensor: FAILED");
  }
  
  // Test Device 3 - Light sensor
  if (testI2CDevice(I2C_DEVICE3_ADDR)) {
    Serial.println("✅ I2C Device 3 (0x50) - Light sensor: OK");
  } else {
    Serial.println("❌ I2C Device 3 (0x50) - Light sensor: FAILED");
  }
  
  // Test Device 4 - Pressure sensor
  if (testI2CDevice(I2C_DEVICE4_ADDR)) {
    Serial.println("✅ I2C Device 4 (0x51) - Pressure sensor: OK");
  } else {
    Serial.println("❌ I2C Device 4 (0x51) - Pressure sensor: FAILED");
  }
}

bool testI2CDevice(int address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

float readI2CSensor(int deviceAddr, int regAddr) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(regAddr);
  Wire.endTransmission();
  
  Wire.requestFrom(deviceAddr, 2);
  if (Wire.available() >= 2) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    return ((highByte << 8) | lowByte) / 100.0;
  }
  return -1.0;
}

void writeI2CDevice(int deviceAddr, int regAddr, int value) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(regAddr);
  Wire.write(value);
  Wire.endTransmission();
}

void sendI2CData() {
  // Gửi dữ liệu tổng hợp đến thiết bị khác qua I2C
  DynamicJsonDocument commDoc(256);
  commDoc["board"] = CLIENT_ID;
  commDoc["type"] = "environment_data";
  commDoc["temperature"] = temperature;
  commDoc["humidity"] = humidity;
  commDoc["light"] = lightLevel;
  commDoc["pressure"] = pressure;
  commDoc["timestamp"] = millis();
  
  // Gửi đến địa chỉ 0x60 (gateway device)
  Wire.beginTransmission(0x60);
  String jsonStr;
  serializeJson(commDoc, jsonStr);
  Wire.write(jsonStr.c_str());
  Wire.endTransmission();
  
  Serial.println("[I2C] Đã gửi dữ liệu môi trường");
}

void receiveI2CData() {
  // Nhận dữ liệu từ thiết bị khác
  Wire.requestFrom(0x61, 32); // Request from sensor hub
  String receivedData = "";
  
  while (Wire.available()) {
    char c = Wire.read();
    receivedData += c;
  }
  
  if (receivedData.length() > 0) {
    Serial.println("[I2C] Nhận dữ liệu: " + receivedData);
    receivedI2CData = receivedData;
    lastI2CCommunication = millis();
  }
}

void sendSPIData() {
  // Gửi dữ liệu qua SPI
  DynamicJsonDocument commDoc(256);
  commDoc["board"] = CLIENT_ID;
  commDoc["type"] = "environment_summary";
  commDoc["avg_temp"] = getAverageTemperature();
  commDoc["avg_humidity"] = getAverageHumidity();
  commDoc["data_points"] = bufferCount;
  commDoc["timestamp"] = millis();
  
  digitalWrite(SPI_CS_PIN, LOW);
  
  String jsonStr;
  serializeJson(commDoc, jsonStr);
  SPI.transfer(jsonStr.c_str(), strlen(jsonStr.c_str()));
  
  digitalWrite(SPI_CS_PIN, HIGH);
  
  Serial.println("[SPI] Đã gửi dữ liệu tổng hợp");
}

String receiveSPIData() {
  // Nhận dữ liệu qua SPI (mô phỏng)
  digitalWrite(SPI_CS_PIN, LOW);
  
  // Trong thực tế sẽ dùng SPI.transfer() để nhận dữ liệu
  String received = "SPI_DATA_FROM_EXTERNAL_DEVICE";
  
  digitalWrite(SPI_CS_PIN, HIGH);
  
  if (received.length() > 0) {
    receivedSPIData = received;
    lastSPICommunication = millis();
  }
  
  return received;
}

float getAverageTemperature() {
  if (bufferCount == 0) return 0.0;
  
  float sum = 0.0;
  for (int i = 0; i < bufferCount; i++) {
    sum += dataBuffer[i].temp;
  }
  return sum / bufferCount;
}

float getAverageHumidity() {
  if (bufferCount == 0) return 0.0;
  
  float sum = 0.0;
  for (int i = 0; i < bufferCount; i++) {
    sum += dataBuffer[i].humidity;
  }
  return sum / bufferCount;
}

void addToDataBuffer(float temp, float hum, float light, float press, float soil, float soilTemp, float airQ) {
  EnvironmentData data;
  data.temp = temp;
  data.humidity = hum;
  data.light = light;
  data.pressure = press;
  data.soilMoist = soil;
  data.soilTemp = soilTemp;
  data.airQuality = airQ;
  data.timestamp = millis();
  data.source = CLIENT_ID;
  
  dataBuffer[bufferIndex] = data;
  bufferIndex = (bufferIndex + 1) % DATA_BUFFER_SIZE;
  if (bufferCount < DATA_BUFFER_SIZE) {
    bufferCount++;
  }
}

void readAllSensors() {
  // Đọc dữ liệu từ các cảm biến analog
  lightLevel = (analogRead(LDR_PIN) / 4095.0) * 100;
  soilMoisture = (analogRead(SOIL_PIN) / 4095.0) * 100;
  soilTemperature = (analogRead(SOIL_TEMP_PIN) / 4095.0) * 50; // 0-50°C
  
  // Đọc dữ liệu từ I2C devices
  temperature = readI2CSensor(I2C_DEVICE1_ADDR, 0x00);
  humidity = readI2CSensor(I2C_DEVICE2_ADDR, 0x00);
  pressure = readI2CSensor(I2C_DEVICE4_ADDR, 0x00);
  
  // Tính toán chất lượng không khí dựa trên nhiệt độ và độ ẩm
  airQuality = 100.0 - abs(temperature - 25.0) * 2.0 - abs(humidity - 50.0);
  airQuality = constrain(airQuality, 0.0, 100.0);
  
  // Thêm vào buffer
  addToDataBuffer(temperature, humidity, lightLevel, pressure, soilMoisture, soilTemperature, airQuality);
}

void handleModeButton() {
  if (buttonModePressed) {
    buttonModePressed = false;
    
    currentMode = (currentMode + 1) % 3;
    
    switch (currentMode) {
      case 0:
        Serial.println("[MODE] Chuyển sang chế độ: MONITOR");
        dataLogging = false;
        break;
      case 1:
        Serial.println("[MODE] Chuyển sang chế độ: DATA LOGGER");
        dataLogging = true;
        loggingStartTime = millis();
        break;
      case 2:
        Serial.println("[MODE] Chuyển sang chế độ: COMMUNICATION HUB");
        dataLogging = false;
        break;
    }
    
    // Báo hiệu LED
    digitalWrite(LED_STATUS_PIN, HIGH);
    delay(200);
    digitalWrite(LED_STATUS_PIN, LOW);
  }
}

void processMode() {
  switch (currentMode) {
    case 0: // Monitor mode
      // Hiển thị dữ liệu realtime
      if (millis() % 2000 < 100) { // Chớp LED mỗi 2 giây
        digitalWrite(LED_DATA_PIN, HIGH);
        delay(100);
        digitalWrite(LED_DATA_PIN, LOW);
      }
      break;
      
    case 1: // Data logger mode
      // Lưu trữ dữ liệu
      if (dataLogging) {
        if (millis() % 1000 < 100) { // Chớp LED mỗi giây
          digitalWrite(LED_DATA_PIN, HIGH);
          delay(50);
          digitalWrite(LED_DATA_PIN, LOW);
        }
        
        // Kiểm tra thời gian ghi dữ liệu
        if (millis() - loggingStartTime > 30000) { // 30 giây
          dataLogging = false;
          Serial.println("[LOGGER] Đã hoàn thành ghi dữ liệu");
        }
      }
      break;
      
    case 2: // Communication hub mode
      // Giao tiếp với thiết bị khác
      receiveI2CData();
      
      if (millis() % 3000 < 100) { // Gửi dữ liệu mỗi 3 giây
        sendI2CData();
        sendSPIData();
      }
      break;
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println("[MQTT] Nhận: " + String(topic) + " = " + message);
  
  String topicStr = String(topic);
  
  if (topicStr == "garden/control/env_mode") {
    if (message == "MONITOR") {
      currentMode = 0;
    } else if (message == "LOGGER") {
      currentMode = 1;
    } else if (message == "COMMUNICATION") {
      currentMode = 2;
    }
  }
  
  if (topicStr == "garden/control/env_config") {
    DynamicJsonDocument doc(512);
    deserializeJson(doc, message);
    
    // Cấu hình các tham số
    Serial.println("[CONFIG] Environment monitoring config updated");
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT kết nối...");
    
    if (client.connect(CLIENT_ID)) {
      Serial.println("✅ Đã kết nối!");
      
      client.subscribe(SUBSCRIBE_TOPIC);
      client.subscribe("garden/control/env_mode");
      client.subscribe("garden/control/env_config");
      
      client.publish("garden/status/board6", "ONLINE");
      
      DynamicJsonDocument infoDoc(512);
      infoDoc["device_id"] = CLIENT_ID;
      infoDoc["board"] = "BOARD6_ENV_MONITOR";
      infoDoc["version"] = "2.0";
      infoDoc["current_mode"] = currentMode;
      infoDoc["data_logging"] = dataLogging;
      infoDoc["ip"] = WiFi.localIP().toString();
      infoDoc["features"] = "multi_sensor,i2c_spi_communication,data_aggregation,multiple_modes";
      
      String infoStr;
      serializeJson(infoDoc, infoStr);
      client.publish("garden/system/device_info", infoStr.c_str());
      
    } else {
      Serial.print("Thất bại, rc=");
      Serial.print(client.state());
      Serial.println(" Thử lại sau 3 giây");
      delay(3000);
    }
  }
}

void sendSensorData() {
  DynamicJsonDocument doc(1024);
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["light_level"] = lightLevel;
  doc["pressure"] = pressure;
  doc["soil_moisture"] = soilMoisture;
  doc["soil_temperature"] = soilTemperature;
  doc["air_quality"] = airQuality;
  doc["current_mode"] = currentMode;
  doc["data_logging"] = dataLogging;
  doc["buffer_size"] = bufferCount;
  doc["avg_temperature"] = getAverageTemperature();
  doc["avg_humidity"] = getAverageHumidity();
  doc["device_id"] = CLIENT_ID;
  doc["timestamp"] = millis();
  doc["rssi"] = WiFi.RSSI();
  doc["free_heap"] = ESP.getFreeHeap();
  
  String payload;
  serializeJson(doc, payload);
  
  client.publish(PUBLISH_TOPIC, payload.c_str());
  Serial.println("[SENSOR] Gửi dữ liệu môi trường: T=" + String(temperature, 1) + 
                 "°C H=" + String(humidity, 1) + "% L=" + String(lightLevel, 0));
}

void sendStatusReport() {
  DynamicJsonDocument statusDoc(512);
  statusDoc["device_id"] = CLIENT_ID;
  statusDoc["status"] = "ONLINE";
  statusDoc["current_mode"] = currentMode;
  statusDoc["data_logging"] = dataLogging;
  statusDoc["i2c_last_comm"] = lastI2CCommunication;
  statusDoc["spi_last_comm"] = lastSPICommunication;
  statusDoc["buffer_count"] = bufferCount;
  statusDoc["uptime"] = millis();
  statusDoc["rssi"] = WiFi.RSSI();
  statusDoc["free_heap"] = ESP.getFreeHeap();
  
  String statusStr;
  serializeJson(statusDoc, statusStr);
  client.publish("garden/status/board6", statusStr.c_str());
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Smart Garden - Board 6: Environmental Monitor ===");
  Serial.println("Client ID: " + String(CLIENT_ID));
  Serial.println("Features: Multi-sensor, I2C/SPI Communication, Data Aggregation");
  
  // Initialize hardware
  setup_i2c();
  setup_spi();
  setup_interrupts();
  
  // Set pin modes
  pinMode(LDR_PIN, INPUT);
  pinMode(TEMP_PIN, INPUT);
  pinMode(SOIL_PIN, INPUT);
  pinMode(SOIL_TEMP_PIN, INPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(LED_DATA_PIN, OUTPUT);
  
  digitalWrite(LED_STATUS_PIN, LOW);
  digitalWrite(LED_DATA_PIN, LOW);
  
  // WatchDog Timer
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Kết nối WiFi và MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  Serial.println("✅ Board 6 sẵn sàng hoạt động!");
  Serial.println("Modes: 0=Monitor, 1=Data Logger, 2=Communication Hub");
}

void loop() {
  // WatchDog reset
  esp_task_wdt_reset();
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  long now = millis();
  
  // Đọc dữ liệu cảm biến định kỳ
  if (now - lastMsg > 15000) {
    lastMsg = now;
    readAllSensors();
    sendSensorData();
  }
  
  // Xử lý mode button
  handleModeButton();
  
  // Xử lý chế độ hiện tại
  processMode();
  
  // Báo cáo trạng thái mỗi 5 phút
  static unsigned long lastStatusReport = 0;
  if (now - lastStatusReport > 300000) {
    lastStatusReport = now;
    sendStatusReport();
  }
  
  delay(100);
}
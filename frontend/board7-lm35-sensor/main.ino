/*
 * Smart Garden LM35 Temperature Sensor with MQTT
 * ESP32 + LM35 Custom Chip + MQTT + MongoDB Backend
 *
 * Features:
 * - Read LM35 temperature sensor every 5 seconds
 * - Publish to MQTT broker.hivemq.com
 * - Store data in MongoDB Atlas via Node.js backend
 * - Device ID: ESP_SENSOR_LM35_01
 * - Author: Huynh Quoc Huy
 *
 * MQTT Topic: garden/sensor/temperature_data
 * Database Collection: temperature_readings
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ===== WiFi Configuration =====
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// ===== MQTT Configuration =====
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_client_id = "ESP_SENSOR_LM35_01";
const char* mqtt_user = "";
const char* mqtt_password = "";

// MQTT Topics
const char* SENSOR_DATA_TOPIC = "garden/sensor/temperature_data";
const char* DEVICE_INFO_TOPIC = "garden/system/device_info";
const char* STATUS_TOPIC = "garden/status/lm35_board";

// ===== Hardware Configuration =====
#define PIN_LM35 34             // GPIO34 (ADC1_CH6) - Valid ADC pin on ESP32
#define LED_PIN 4               // Status LED

// ===== ADC Configuration for ESP32 =====
// ESP32 ADC với custom chip Wokwi xuất 0-5V
#define ADC_RESOLUTION 4095.0   // ESP32 has 12-bit ADC (0-4095)
// Với ADC_11db attenuation, ADC đọc 0-3.3V
// Nhưng custom chip output 0-5V, nên cần scale
#define ADC_MAX_VOLTAGE 3300.0  // mV (với 11db attenuation)

// ===== Timing Configuration =====
#define PUBLISH_INTERVAL 5000    // 5 seconds
#define WIFI_TIMEOUT 10000       // 10 seconds
#define MQTT_TIMEOUT 5000        // 5 seconds

// ===== Device Configuration =====
#define DEVICE_NAME "SmartGarden-LM35-Sensor"
#define DEVICE_VERSION "1.0.1"
#define DEVICE_LOCATION "SmartGarden_A"

// ===== Global Variables =====
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastPublishTime = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastMqttCheck = 0;
int wifiReconnectAttempts = 0;
int mqttReconnectAttempts = 0;
bool deviceRegistered = false;

// Sensor variables
float currentTempC = 0.0;
float currentTempF = 0.0;
unsigned long readingCount = 0;

// ===== Function Prototypes =====
void setupWiFi();
void setupMQTT();
void connectWiFi();
void connectMQTT();
void readTemperature();
void publishSensorData();
void publishDeviceInfo();
void publishStatus();
void updateStatusLED();
void blinkLED(int times, int delayMs);
void callback(char* topic, byte* payload, unsigned int length);

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║  Smart Garden LM35 Temperature Sensor         ║");
  Serial.println("║  ESP32 + MQTT + MongoDB Backend               ║");
  Serial.println("║  Version: 1.0.1 (Fixed ADC Pin)               ║");
  Serial.println("╚════════════════════════════════════════════════╝\n");
  
  // Initialize hardware
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Configure ADC
  analogReadResolution(12);  // ESP32 supports 12-bit resolution
  analogSetAttenuation(ADC_11db);  // Full range: 0-3.3V (150mV-2450mV typical)
  
  Serial.println("🔧 Initializing LM35 sensor...");
  Serial.printf("   Pin: GPIO%d (ADC1_CH6)\n", PIN_LM35);
  Serial.printf("   ADC Resolution: 12-bit (0-4095)\n");
  Serial.printf("   ADC Range: 0-3300mV (11db attenuation)\n");
  Serial.printf("   LM35 Output: 10mV per °C\n");
  
  // Wait for ADC to stabilize
  delay(500);
  
  // Test sensor reading
  Serial.println("🧪 Testing LM35 sensor...");
  for (int i = 0; i < 3; i++) {
    readTemperature();
    delay(500);
  }
  Serial.printf("✅ LM35 test complete: T=%.1f°C (%.1f°F)\n", currentTempC, currentTempF);
  
  // Initialize WiFi and MQTT
  setupWiFi();
  setupMQTT();
  
  // Publish device info
  publishDeviceInfo();
  deviceRegistered = true;
  
  Serial.println("\n🚀 Sensor board ready!");
  Serial.println("📋 Publishing temperature data every 5 seconds\n");
  
  blinkLED(3, 200);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    if (currentTime - lastWifiCheck > WIFI_TIMEOUT) {
      Serial.println("🔌 WiFi disconnected, attempting reconnection...");
      connectWiFi();
      lastWifiCheck = currentTime;
    }
  }
  
  // Check MQTT connection
  if (!client.connected()) {
    if (currentTime - lastMqttCheck > MQTT_TIMEOUT) {
      Serial.println("📡 MQTT disconnected, attempting reconnection...");
      connectMQTT();
      lastMqttCheck = currentTime;
    }
  } else {
    client.loop();
  }
  
  // Read sensor and publish data
  if (currentTime - lastPublishTime >= PUBLISH_INTERVAL) {
    readTemperature();
    publishSensorData();
    publishStatus();
    lastPublishTime = currentTime;
  }
  
  updateStatusLED();
  delay(50);
}

void setupWiFi() {
  delay(10);
  Serial.println("📡 Setting up WiFi connection...");
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("✅ WiFi connected successfully!");
    Serial.print("📍 IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("📶 Signal strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    wifiReconnectAttempts = 0;
  } else {
    Serial.println();
    Serial.println("❌ WiFi connection failed!");
    Serial.println("⚠️ Continuing with offline mode...");
  }
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }
  
  Serial.printf("🔄 WiFi reconnection attempt %d...\n", wifiReconnectAttempts + 1);
  
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 10) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("✅ WiFi reconnected!");
    wifiReconnectAttempts = 0;
  } else {
    Serial.println();
    Serial.println("❌ WiFi reconnection failed!");
    wifiReconnectAttempts++;
  }
}

void setupMQTT() {
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setBufferSize(512);
  
  connectMQTT();
}

void connectMQTT() {
  while (!client.connected() && mqttReconnectAttempts < 5) {
    Serial.printf("📡 MQTT connection attempt %d...\n", mqttReconnectAttempts + 1);
    
    if (client.connect(mqtt_client_id, mqtt_user, mqtt_password)) {
      Serial.println("✅ MQTT connected to broker.hivemq.com!");
      mqttReconnectAttempts = 0;
      blinkLED(2, 100);
    } else {
      Serial.print("❌ MQTT connection failed, rc=");
      Serial.print(client.state());
      Serial.print(" (");
      
      switch (client.state()) {
        case -4: Serial.println("MQTT_CONNECTION_TIMEOUT)"); break;
        case -3: Serial.println("MQTT_CONNECTION_LOST)"); break;
        case -2: Serial.println("MQTT_CONNECT_BAD_PROTOCOL)"); break;
        case -1: Serial.println("MQTT_CONNECT_BAD_CLIENT_ID)"); break;
        case 0: Serial.println("MQTT_CONNECT_UNAVAILABLE)"); break;
        case 1: Serial.println("MQTT_CONNECT_BAD_CREDENTIALS)"); break;
        case 2: Serial.println("MQTT_CONNECT_UNAUTHORIZED)"); break;
        default: Serial.println("UNKNOWN_ERROR)"); break;
      }
      
      mqttReconnectAttempts++;
      delay(5000);
    }
  }
}

void readTemperature() {
  // Read multiple samples for stability
  const int numSamples = 10;
  long adcSum = 0;
  
  for (int i = 0; i < numSamples; i++) {
    adcSum += analogRead(PIN_LM35);
    delay(10);
  }
  
  int adcVal = adcSum / numSamples;
  
  // Convert ADC to voltage (in millivolts)
  // ESP32 ADC with 11db attenuation: 0-4095 maps to 0-3300mV
  float milliVolt = (adcVal * ADC_MAX_VOLTAGE) / ADC_RESOLUTION;
  
  // LM35 custom chip outputs voltage proportional to slider (0-5V range)
  // Since chip control is "moisture" from 0-5, this maps to 0-5V output
  // Temperature = Voltage / 10mV per °C
  // But we need to scale: chip 5V max → should represent ~50°C
  // Formula: Temp = (voltage_mV / 3300) * 500 / 10 = voltage_mV * 0.01515
  
  // Simple conversion: LM35 = 10mV per °C
  currentTempC = milliVolt / 10.0;
  
  // If reading seems off due to voltage scaling, apply correction factor
  // Custom chip "moisture" 0-5 maps to 0-50°C (or adjust as needed)
  
  // Convert to Fahrenheit
  currentTempF = (currentTempC * 1.8) + 32.0;
  
  readingCount++;
  
  Serial.printf("📊 Reading #%lu: ADC=%d, Voltage=%.1fmV, T=%.1f°C (%.1f°F)\n",
                readingCount, adcVal, milliVolt, currentTempC, currentTempF);
}

void publishSensorData() {
  if (!client.connected()) {
    Serial.println("⚠️ Cannot publish: MQTT not connected");
    return;
  }
  
  Serial.println("📤 Publishing sensor data...");
  
  DynamicJsonDocument doc(512);
  
  doc["device_id"] = mqtt_client_id;
  doc["temperature_celsius"] = round(currentTempC * 10.0) / 10.0;
  doc["temperature_fahrenheit"] = round(currentTempF * 10.0) / 10.0;
  doc["sensor_type"] = "LM35";
  doc["reading_id"] = readingCount;
  doc["uptime_seconds"] = millis() / 1000;
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["free_heap"] = ESP.getFreeHeap();
  
  // Add temperature status
  if (currentTempC < 15.0) {
    doc["status"] = "cold";
  } else if (currentTempC > 30.0) {
    doc["status"] = "hot";
  } else {
    doc["status"] = "normal";
  }
  
  String payload;
  serializeJson(doc, payload);
  
  bool publishSuccess = false;
  int maxRetries = 3;
  
  for (int attempt = 0; attempt < maxRetries && !publishSuccess; attempt++) {
    if (!client.connected()) {
      Serial.println("⚠️ MQTT disconnected, attempting reconnect...");
      connectMQTT();
      delay(1000);
    }
    
    if (client.publish(SENSOR_DATA_TOPIC, payload.c_str())) {
      Serial.printf("✅ Published: T=%.1f°C (%.1f°F) | Readings: %lu\n",
                    currentTempC, currentTempF, readingCount);
      publishSuccess = true;
      blinkLED(1, 20);
    } else {
      Serial.printf("❌ MQTT publish failed (attempt %d/%d), rc=%d\n",
                    attempt + 1, maxRetries, client.state());
      
      if (attempt < maxRetries - 1) {
        Serial.println("🔄 Retrying in 2 seconds...");
        delay(2000);
      }
    }
  }
  
  if (!publishSuccess) {
    Serial.println("❌ MQTT publish failed after all retries!");
    blinkLED(3, 50);
  }
}

void publishDeviceInfo() {
  if (!client.connected()) {
    return;
  }
  
  DynamicJsonDocument doc(512);
  
  doc["device_id"] = mqtt_client_id;
  doc["type"] = "temperature_sensor";
  doc["name"] = DEVICE_NAME;
  doc["version"] = DEVICE_VERSION;
  doc["location"] = DEVICE_LOCATION;
  doc["firmware_date"] = "2025-12-19";
  doc["author"] = "Huynh Quoc Huy";
  
  // Hardware info
  doc["board"] = "ESP32 DevKit C V4";
  doc["processor"] = "ESP32";
  doc["flash_size"] = ESP.getFlashChipSize();
  doc["free_heap"] = ESP.getFreeHeap();
  doc["cpu_freq"] = ESP.getCpuFreqMHz();
  
  // Sensor info
  doc["sensors"]["lm35"]["pin"] = PIN_LM35;
  doc["sensors"]["lm35"]["model"] = "LM35";
  doc["sensors"]["lm35"]["type"] = "analog_temperature";
  doc["sensors"]["lm35"]["status"] = "active";
  doc["sensors"]["lm35"]["range"] = "0-150°C";
  doc["sensors"]["lm35"]["accuracy"] = "±0.5°C";
  
  // MQTT info
  doc["mqtt"]["broker"] = mqtt_server;
  doc["mqtt"]["port"] = mqtt_port;
  doc["mqtt"]["topics"]["data"] = SENSOR_DATA_TOPIC;
  doc["mqtt"]["topics"]["status"] = STATUS_TOPIC;
  
  // Network info
  doc["network"]["wifi_ssid"] = ssid;
  doc["network"]["ip"] = WiFi.localIP().toString();
  doc["network"]["mac"] = WiFi.macAddress();
  doc["network"]["rssi"] = WiFi.RSSI();
  
  doc["status"] = "online";
  
  String payload;
  serializeJson(doc, payload);
  
  if (client.publish(DEVICE_INFO_TOPIC, payload.c_str())) {
    Serial.println("📤 Device info published to backend");
  }
}

void publishStatus() {
  if (!client.connected()) {
    return;
  }
  
  DynamicJsonDocument doc(256);
  
  doc["device_id"] = mqtt_client_id;
  doc["status"] = "online";
  doc["current_temperature"] = currentTempC;
  doc["uptime_seconds"] = millis() / 1000;
  doc["free_heap"] = ESP.getFreeHeap();
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["wifi_quality"] = WiFi.RSSI() > -70 ? "good" : WiFi.RSSI() > -80 ? "fair" : "poor";
  doc["mqtt_connected"] = client.connected();
  doc["reading_count"] = readingCount;
  
  String payload;
  serializeJson(doc, payload);
  
  client.publish(STATUS_TOPIC, payload.c_str());
}

void updateStatusLED() {
  static unsigned long lastLEDUpdate = 0;
  static bool ledState = false;
  
  unsigned long currentTime = millis();
  
  if (WiFi.status() == WL_CONNECTED && client.connected()) {
    // Connected: slow blink (every 2 seconds)
    if (currentTime - lastLEDUpdate > 2000) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      lastLEDUpdate = currentTime;
    }
  } else {
    // Disconnected: fast blink (every 500ms)
    if (currentTime - lastLEDUpdate > 500) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      lastLEDUpdate = currentTime;
    }
  }
}

void blinkLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    delay(delayMs);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.printf("📨 MQTT Message on %s: %s\n", topic, message.c_str());
}

/*
 * IMPORTANT NOTES:
 * 
 * 1. ESP32 ADC Pins (ADC1):
 *    GPIO32-39 are ADC1 pins (safe to use with WiFi)
 *    GPIO34 (ADC1_CH6) is used in this project
 * 
 * 2. Custom Chip Wiring:
 *    - chip2:OUT → GPIO34 (not GPIO2!)
 *    - Update diagram.json connection
 * 
 * 3. LM35 Custom Chip:
 *    - Control slider "moisture" 0-5 represents voltage 0-5V
 *    - LM35: 10mV per °C
 *    - Slider at 2.5 → 25°C
 * 
 * 4. Expected JSON format for MongoDB:
 * {
 *   "device_id": "ESP_SENSOR_LM35_01",
 *   "temperature_celsius": 25.4,
 *   "temperature_fahrenheit": 77.7,
 *   "sensor_type": "LM35",
 *   "reading_id": 123,
 *   "uptime_seconds": 3600,
 *   "wifi_rssi": -45,
 *   "free_heap": 180000,
 *   "status": "normal"
 * }
 */
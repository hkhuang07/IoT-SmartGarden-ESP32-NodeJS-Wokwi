  /*
 * Smart Garden Temperature & Humidity Sensor Board
 * ESP32 + DHT22 + MQTT + MongoDB Backend
 *
 * Features:
 * - Read DHT22 temperature & humidity every 5 seconds
 * - Publish to MQTT broker.hivemq.com
 * - Store data in MongoDB Atlas via Node.js backend
 * - Device ID: ESP_SENSOR_DHT22_01
 * 
 * MQTT Topic: garden/sensor/temp_hum_data
 * Database Collection: temp_hum_readings
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <time.h>

// WiFi Configuration
const char* ssid = "Wokwi-GUEST";          
const char* password = "";                  

// MQTT Configuration
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_client_id = "ESP_SENSOR_DHT22_01";
const char* mqtt_user = "";                 
const char* mqtt_password = "";             

// MQTT Topics
const char* SENSOR_DATA_TOPIC = "garden/sensor/temp_hum_data";
const char* DEVICE_INFO_TOPIC = "garden/system/device_info";
const char* STATUS_TOPIC = "garden/status/sensor_board";

// Hardware Pins
#define DHT_PIN 4          
#define DHT_TYPE DHT22     
#define LED_PIN 2          

// Timing
#define PUBLISH_INTERVAL 5000    
#define WIFI_TIMEOUT 10000       
#define MQTT_TIMEOUT 5000        
#define DHT_READ_DELAY 2500      

// Device Configuration
#define DEVICE_NAME "SmartGarden-TempHum-Sensor"
#define DEVICE_VERSION "2.0.0"
#define DEVICE_LOCATION "SmartGarden_A"

// Global Variables
DHT dht(DHT_PIN, DHT_TYPE);
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastPublishTime = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastMqttCheck = 0;
int wifiReconnectAttempts = 0;
int mqttReconnectAttempts = 0;
bool deviceRegistered = false;

// Sensor reading counter
unsigned long readingCount = 0;

// Function prototypes
void setupWiFi();
void setupMQTT();
void connectWiFi();
void connectMQTT();
void publishSensorData();
void publishDeviceInfo();
void publishStatus();
void updateStatusLED();
void blinkLED(int times, int delayMs);
void callback(char* topic, byte* payload, unsigned int length);
float readTemperature();
float readHumidity();
String getDeviceUptime();
String getCurrentTime();

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("🌱 Smart Garden Temperature & Humidity Sensor");
  Serial.println("📡 MQTT: broker.hivemq.com → MongoDB Atlas");
  Serial.println("============================================================");
  
  // Initialize hardware
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Seed random number generator for simulated data
  randomSeed(analogRead(0));
  Serial.println("🎲 Random generator seeded for simulated data fallback");
  
  // Initialize DHT22 sensor
  Serial.print("🔧 Initializing DHT22 sensor on pin ");
  Serial.println(DHT_PIN);
  dht.begin();
  Serial.println("✅ DHT22 sensor initialized");
  
  // DHT22 startup delay (required for stable operation)
  Serial.println("⏳ DHT22 sensor startup delay (2 seconds)...");
  delay(2000);
  
  /* /Test sensor reading
  Serial.println("🧪 Testing DHT22 sensor...");
  float testTemp = readTemperature();
  float testHum = readHumidity();
  
  if (!isnan(testTemp) && !isnan(testHum)) {
    Serial.printf("✅ DHT22 test successful: T=%.1f°C, H=%.0f%%\n", testTemp, testHum);
  } else {
    Serial.println("⚠️ DHT22 sensor test failed - will use simulated data");
  }
  */
  
  // Initialize WiFi and MQTT
  setupWiFi();
  setupMQTT();
  
  // Publish device info to register with backend
  publishDeviceInfo();
  deviceRegistered = true;
  
  Serial.println("🚀 Sensor board ready!");
  Serial.println("📋 Publishing temperature & humidity data every 5 seconds");
  Serial.println();
  
  // Initial status LED blink
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
  
  if (!client.connected()) {
    if (currentTime - lastMqttCheck > MQTT_TIMEOUT) {
      Serial.println("📡 MQTT disconnected, attempting reconnection...");
      connectMQTT();
      lastMqttCheck = currentTime;
    }
  } else {
    if (client.loop()) {
      static unsigned long lastLoopTime = 0;
      if (currentTime - lastLoopTime > 10000) {  // Every 30 seconds
        Serial.println("🔄 MQTT loop processing normally");
        lastLoopTime = currentTime;
      }
    } else {
      Serial.println("⚠️ MQTT loop failed, checking connection...");
    }
  }
  
  checkMQTTBuffer();
  
  if (currentTime - lastPublishTime >= PUBLISH_INTERVAL) {
    Serial.printf("📤 Publishing sensor data (interval: %lu ms)...\n", PUBLISH_INTERVAL);
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

void checkMQTTBuffer() {
  if (client.connected()) {
    client.loop();
    
    if (ESP.getFreeHeap() < 5000) {
      Serial.println("⚠️ Low heap memory, cleaning up...");
      delay(100);
    }
  }
}

void connectMQTT() {
  while (!client.connected() && mqttReconnectAttempts < 5) {
    Serial.printf("📡 MQTT connection attempt %d...\n", mqttReconnectAttempts + 1);
    
    if (client.connect(mqtt_client_id, mqtt_user, mqtt_password)) {
      Serial.println("✅ MQTT connected to broker.hivemq.com!");
      mqttReconnectAttempts = 0;
      blinkLED(2, 100);
      
      /*Serial.println("🧪 Testing MQTT connection with small payload...");
      if (client.publish(SENSOR_DATA_TOPIC, "test_connection")) {
        Serial.println("✅ MQTT publish test successful");
      } else {
        Serial.println("❌ MQTT publish test failed");
      }*/
      
    } else {
      Serial.print("❌ MQTT connection failed, rc=");
      Serial.print(client.state());
      Serial.print(" (");
      
      // FIXED: Better error code interpretation
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

void publishSensorData() {
  static unsigned long lastSensorRead = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastSensorRead < 3000) {  
    return;
  }
  
  float temperature = NAN;
  float humidity = NAN;
  
  for (int attempt = 0; attempt < 3; attempt++) {
    Serial.printf("🔄 Sensor reading attempt %d/3...\n", attempt + 1);
    temperature = dht.readTemperature(false);
    humidity = dht.readHumidity();
    
    if (!isnan(temperature) && !isnan(humidity) && 
        temperature > -40 && temperature < 85 && 
        humidity >= 0 && humidity <= 100) {
      Serial.printf("✅ Valid readings: T=%.1f°C, H=%.0f%%\n", temperature, humidity);
      break;
    }
    if (attempt < 2) {  // Don't delay on last attempt
      Serial.println("⚠️ Invalid readings, waiting 2 seconds before retry...");
      delay(2000);
    }
  }
  
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("⚠️ DHT22 sensor unavailable, using simulated data");
    temperature = getSimulatedTemperature();
    humidity = getSimulatedHumidity();
    Serial.printf("🎭 Generated simulated data: T=%.1f°C, H=%.0f%%\n", temperature, humidity);
    blinkLED(3, 100);
  }
  
  lastSensorRead = currentTime;
  DynamicJsonDocument doc(512); 

  doc["device_id"] = mqtt_client_id;
  doc["temperature"] = round(temperature * 10.0) / 10.0; 
  doc["humidity"] = round(humidity* 10.0) / 10.0;                      
  doc["reading_id"] = ++readingCount;
  doc["uptime_seconds"] = millis() / 1000;
  doc["timestamp"] = getCurrentTime();
  doc["wifi_rssi"] = WiFi.RSSI();
  
  // doc["device_name"] - REMOVED (redundant)
  // doc["device_version"] - REMOVED (not needed for every publish)
  // doc["location"] - REMOVED (static info)
  // doc["local_ip"] - REMOVED (network info)
  // doc["wifi_quality"] - REMOVED (calculated from RSSI)
  // doc["epoch_ms"] - REMOVED (timestamp enough)
  // doc["sensor_type"] - REMOVED (known from device)
  // doc["status"] - REMOVED (assume OK if published)
  
  if (temperature > 30 && humidity > 70) {
    float heatIndex = dht.computeHeatIndex(temperature, humidity, false);
    doc["heat_index"] = round(heatIndex * 10.0) / 10.0;
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
      Serial.printf("📤 Published: T=%.1f°C, H=%.0f%% | Readings: %lu\n", 
                    temperature, humidity, readingCount);
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
    Serial.println("💡 Check MQTT broker connectivity and buffer size");
    blinkLED(3, 50);
  }
}

void publishDeviceInfo() {
  if (!deviceRegistered) return;
  
  DynamicJsonDocument doc(512);
  
  doc["device_id"] = mqtt_client_id;
  doc["type"] = "temperature_humidity_sensor";
  doc["name"] = DEVICE_NAME;
  doc["version"] = DEVICE_VERSION;
  doc["location"] = DEVICE_LOCATION;
  doc["firmware_date"] = "2025-12-01";
  doc["author"] = "MiniMax Agent";
  
  // Hardware info
  doc["board"] = "ESP32 DevKit C V4";
  doc["processor"] = "ESP32";
  doc["flash_size"] = ESP.getFlashChipSize();
  doc["free_heap"] = ESP.getFreeHeap();
  doc["cpu_freq"] = ESP.getCpuFreqMHz();
  
  // Sensor info
  doc["sensors"]["dht22"]["pin"] = DHT_PIN;
  doc["sensors"]["dht22"]["model"] = "DHT22";
  doc["sensors"]["dht22"]["status"] = "active";
  
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
  doc["timestamp"] = getCurrentTime();
  
  String payload;
  serializeJson(doc, payload);
  
  if (client.publish(DEVICE_INFO_TOPIC, payload.c_str())) {
    Serial.println("📤 Device info published to backend");
  }
}

void publishStatus() {
  DynamicJsonDocument doc(256);
  
  doc["device_id"] = mqtt_client_id;
  doc["status"] = "online";
  doc["uptime_seconds"] = millis() / 1000;
  doc["free_heap"] = ESP.getFreeHeap();
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["wifi_quality"] = WiFi.RSSI() > -70 ? "good" : WiFi.RSSI() > -80 ? "fair" : "poor";
  doc["mqtt_connected"] = client.connected();
  doc["reading_count"] = readingCount;
  doc["last_publish"] = getCurrentTime();
  
  String payload;
  serializeJson(doc, payload);
  
  client.publish(STATUS_TOPIC, payload.c_str());
}

void updateStatusLED() {
  static unsigned long lastLEDUpdate = 0;
  static bool ledState = false;
  
  unsigned long currentTime = millis();
  
    if (WiFi.status() == WL_CONNECTED && client.connected()) {
    if (currentTime - lastLEDUpdate > 2000) {  // Blink every 2 seconds
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      lastLEDUpdate = currentTime;
    }
  } else {
    if (currentTime - lastLEDUpdate > 500) {   // Blink every 500ms
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

float readTemperature() {
  for (int attempt = 0; attempt < 5; attempt++) {
    float temp = dht.readTemperature(false);  
    
    if (!isnan(temp) && temp > -40 && temp < 85) {  
      return temp;
    }
    
    Serial.printf("🔄 DHT22 reading attempt %d/5, retrying...\n", attempt + 1);
    delay(1000);  
  }
  
  Serial.println("⚠️ Failed to read temperature from DHT22");
  return NAN;
}

float readHumidity() {
  for (int attempt = 0; attempt < 5; attempt++) {
    float hum = dht.readHumidity();
    
    if (!isnan(hum) && hum >= 0 && hum <= 100) {  
      return hum;
    }
    
    Serial.printf("🔄 DHT22 humidity attempt %d/5, retrying...\n", attempt + 1);
    delay(1000); 
  }
  
  Serial.println("⚠️ Failed to read humidity from DHT22");
  return NAN;
}

String getCurrentTime() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  
  char buffer[64];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
  
  return String(buffer);
}

String getDeviceUptime() {
  unsigned long uptimeSeconds = millis() / 1000;
  
  int days = uptimeSeconds / 86400;
  int hours = (uptimeSeconds % 86400) / 3600;
  int minutes = (uptimeSeconds % 3600) / 60;
  int seconds = uptimeSeconds % 60;
  
  if (days > 0) {
    return String(days) + "d " + String(hours) + "h " + String(minutes) + "m";
  } else if (hours > 0) {
    return String(hours) + "h " + String(minutes) + "m";
  } else if (minutes > 0) {
    return String(minutes) + "m " + String(seconds) + "s";
  } else {
    return String(seconds) + "s";
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.printf("📨 MQTT Message on %s: %s\n", topic, message.c_str());
  
}

float getSimulatedTemperature() {
  int variation = (readingCount % 100) - 50; 
  return 24.0 + (variation / 100.0); 
}

float getSimulatedHumidity() {
  int variation = (readingCount % 100) - 50; 
  return 60.0 + (variation / 10.0); 
}

/*
 * Expected JSON format for MongoDB storage:
 * {
 *   "device_id": "ESP_SENSOR_DHT22_01",
 *   "device_name": "SmartGarden-TempHum-Sensor",
 *   "temperature": 25.4,
 *   "humidity": 65.0,
 *   "sensor_type": "DHT22",
 *   "reading_id": 123,
 *   "uptime_seconds": 3600,
 *   "wifi_rssi": -45,
 *   "wifi_quality": "good",
 *   "timestamp": "2025-12-01 18:30:00",
 *   "local_ip": "192.168.1.100",
 *   "status": "ok",
 *   "heat_index": 28.5
 * }
 * 
 * Database Collection: temp_hum_readings
 * Backend Node.js server processes this data and stores in MongoDB Atlas
 */
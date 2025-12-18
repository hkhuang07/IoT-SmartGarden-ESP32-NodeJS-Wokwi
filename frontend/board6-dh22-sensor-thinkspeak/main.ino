/*
 * Smart Garden DHT22 Sensor with ThingSpeak & MQTT Integration
 * ESP32 + DHT22 + LDR + Relay Control
 *
 * Features:
 * - Read DHT22 temperature, humidity & LDR light sensor
 * - Publish to ThingSpeak cloud platform
 * - Publish to MQTT broker.hivemq.com
 * - Automatic relay control based on temp/humidity thresholds
 * - Alert system for abnormal conditions
 * - Device ID: ESP_SENSOR_DHT22_THINGSPEAK_01
 * - Author: Huynh Quoc Huy
 *
 * MQTT Topics:
 * - garden/sensor/temp_hum_data (sensor readings)
 * - garden/alerts/climate (threshold alerts)
 * - garden/status/dht22_board (device status)
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <ThingSpeak.h>

// ===== WiFi Configuration =====
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// ===== ThingSpeak Configuration =====
const unsigned long ChannelNumber = 3083564;
const char* WriteAPIKey = "SH8BFGIL0Q9EIB6M";

// ===== MQTT Configuration =====
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_client_id = "ESP_SENSOR_DHT22_THINGSPEAK_01";

// MQTT Topics
const char* SENSOR_DATA_TOPIC = "garden/sensor/temp_hum_data";
const char* ALERT_TOPIC = "garden/alerts/climate";
const char* STATUS_TOPIC = "garden/status/dht22_board";
const char* DEVICE_INFO_TOPIC = "garden/system/device_info";

// ===== Hardware Pins =====
#define DHTPIN 4
#define DHTTYPE DHT22
#define LDR_PIN 32
#define RELAY_PIN 25        // Relay control pin
#define LED_PIN 2           // Status LED

// ===== Thresholds for Relay Control =====
#define TEMP_HIGH_THRESHOLD 30.0    // °C - Turn ON relay if exceeded
#define TEMP_LOW_THRESHOLD 15.0     // °C - Turn OFF relay if below
#define HUMIDITY_HIGH_THRESHOLD 75.0 // % - Turn ON relay if exceeded
#define HUMIDITY_LOW_THRESHOLD 40.0  // % - Turn OFF relay if below

// ===== Timing Configuration =====
#define THINGSPEAK_INTERVAL 20000   // 20 seconds (ThingSpeak rate limit: 15s)
#define MQTT_INTERVAL 5000          // 5 seconds
#define SENSOR_READ_INTERVAL 2000   // 2 seconds
#define STATUS_INTERVAL 30000       // 30 seconds

// ===== Global Objects =====
DHT dht(DHTPIN, DHTTYPE);
WiFiClient wifiClient;
WiFiClient mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);

// ===== Global Variables =====
unsigned long lastThingSpeakTime = 0;
unsigned long lastMqttTime = 0;
unsigned long lastSensorReadTime = 0;
unsigned long lastStatusTime = 0;

float currentTemperature = 0.0;
float currentHumidity = 0.0;
int currentLight = 0;
bool relayState = false;

unsigned long readingCount = 0;
unsigned long mqttPublishCount = 0;
unsigned long thingspeakPublishCount = 0;

bool mqttConnected = false;
int mqttReconnectAttempts = 0;

String alertMessage = "";
bool alertSent = false;

// ===== Function Prototypes =====
void setupWiFi();
void setupMQTT();
void setupHardware();
void connectMQTT();
void readSensors();
void publishToThingSpeak();
void publishToMQTT();
void publishStatus();
void publishDeviceInfo();
void checkRelayControl();
void updateStatusLED();
void blinkLED(int times, int delayMs);
void checkAlerts();
void mqttCallback(char* topic, byte* payload, unsigned int length);

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║  Smart Garden DHT22 + ThingSpeak + MQTT      ║");
  Serial.println("║  With Automatic Relay Control                 ║");
  Serial.println("║  Version: 3.0.0                               ║");
  Serial.println("╚════════════════════════════════════════════════╝\n");
  
  // Initialize hardware
  setupHardware();
  
  // Initialize DHT22 sensor
  Serial.println("🔧 Initializing DHT22 sensor...");
  dht.begin();
  delay(2000);  // DHT22 startup delay
  Serial.println("✅ DHT22 sensor initialized");
  
  // Connect to WiFi
  setupWiFi();
  
  // Initialize ThingSpeak
  Serial.println("☁️  Initializing ThingSpeak client...");
  ThingSpeak.begin(wifiClient);
  Serial.println("✅ ThingSpeak client ready");
  
  // Initialize MQTT
  setupMQTT();
  
  // Publish device info
  publishDeviceInfo();
  
  Serial.println("\n🚀 System ready!");
  Serial.println("📊 Publishing data to:");
  Serial.println("   - ThingSpeak (every 20 seconds)");
  Serial.println("   - MQTT broker (every 5 seconds)");
  Serial.println("🎛️  Relay control: AUTO mode");
  Serial.println();
  
  blinkLED(3, 200);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ WiFi disconnected, reconnecting...");
    setupWiFi();
  }
  
  // Check MQTT connection
  if (!mqttClient.connected()) {
    if (currentTime - lastStatusTime > 10000) {  // Try reconnect every 10s
      connectMQTT();
      lastStatusTime = currentTime;
    }
  } else {
    mqttClient.loop();
  }
  
  // Read sensors periodically
  if (currentTime - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    readSensors();
    checkRelayControl();
    checkAlerts();
    lastSensorReadTime = currentTime;
  }
  
  // Publish to ThingSpeak
  if (currentTime - lastThingSpeakTime >= THINGSPEAK_INTERVAL) {
    publishToThingSpeak();
    lastThingSpeakTime = currentTime;
  }
  
  // Publish to MQTT
  if (currentTime - lastMqttTime >= MQTT_INTERVAL) {
    if (mqttClient.connected()) {
      publishToMQTT();
      publishStatus();
    }
    lastMqttTime = currentTime;
  }
  
  updateStatusLED();
  delay(50);
}

void setupHardware() {
  // Initialize pins
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  digitalWrite(RELAY_PIN, LOW);   // Relay OFF initially
  digitalWrite(LED_PIN, LOW);     // LED OFF initially
  
  Serial.println("✅ Hardware initialized:");
  Serial.println("   - Relay: OFF");
  Serial.println("   - LED: OFF");
  Serial.println("   - DHT22 Pin: 4");
  Serial.println("   - LDR Pin: 32");
  Serial.println("   - Relay Pin: 25");
}

void setupWiFi() {
  Serial.println("📡 Connecting to WiFi...");
  Serial.print("   SSID: ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi connected!");
    Serial.print("   IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("   Signal: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println("\n❌ WiFi connection failed!");
  }
}

void setupMQTT() {
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);
  
  Serial.println("📡 Setting up MQTT connection...");
  connectMQTT();
}

void connectMQTT() {
  if (mqttClient.connected()) {
    return;
  }
  
  Serial.printf("📡 MQTT connection attempt %d...\n", mqttReconnectAttempts + 1);
  
  if (mqttClient.connect(mqtt_client_id)) {
    Serial.println("✅ MQTT connected to broker.hivemq.com!");
    mqttConnected = true;
    mqttReconnectAttempts = 0;
    blinkLED(2, 100);
  } else {
    Serial.print("❌ MQTT connection failed, rc=");
    Serial.println(mqttClient.state());
    mqttConnected = false;
    mqttReconnectAttempts++;
  }
}

void readSensors() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  int light = analogRead(LDR_PIN);
  
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("⚠️ Failed to read from DHT sensor!");
    return;
  }
  
  currentTemperature = temperature;
  currentHumidity = humidity;
  currentLight = light;
  readingCount++;
  
  Serial.printf("📊 Reading #%lu: T=%.1f°C, H=%.1f%%, Light=%d\n", 
                readingCount, temperature, humidity, light);
}

void publishToThingSpeak() {
  Serial.println("☁️  Publishing to ThingSpeak...");
  
  ThingSpeak.setField(1, currentTemperature);
  ThingSpeak.setField(2, currentHumidity);
  ThingSpeak.setField(3, relayState ? 1 : 0);  // Relay state
  ThingSpeak.setField(4, currentLight);
  
  int statusCode = ThingSpeak.writeFields(ChannelNumber, WriteAPIKey);
  
  if (statusCode == 200) {
    thingspeakPublishCount++;
    Serial.printf("✅ ThingSpeak: Data sent successfully (#%lu)\n", thingspeakPublishCount);
    blinkLED(1, 50);
  } else {
    Serial.printf("❌ ThingSpeak: Failed (HTTP %d)\n", statusCode);
  }
}

void publishToMQTT() {
  if (!mqttClient.connected()) {
    return;
  }
  
  DynamicJsonDocument doc(512);
  
  doc["device_id"] = mqtt_client_id;
  doc["temperature"] = round(currentTemperature * 10.0) / 10.0;
  doc["humidity"] = round(currentHumidity * 10.0) / 10.0;
  doc["light"] = currentLight;
  doc["relay_state"] = relayState;
  doc["reading_id"] = readingCount;
  doc["uptime_seconds"] = millis() / 1000;
  doc["wifi_rssi"] = WiFi.RSSI();
  
  // Add heat index if conditions met
  if (currentTemperature > 30 && currentHumidity > 70) {
    float heatIndex = dht.computeHeatIndex(currentTemperature, currentHumidity, false);
    doc["heat_index"] = round(heatIndex * 10.0) / 10.0;
  }
  
  String payload;
  serializeJson(doc, payload);
  
  if (mqttClient.publish(SENSOR_DATA_TOPIC, payload.c_str())) {
    mqttPublishCount++;
    Serial.printf("📤 MQTT: Data published (#%lu)\n", mqttPublishCount);
    blinkLED(1, 20);
  } else {
    Serial.println("❌ MQTT: Publish failed!");
  }
}

void publishStatus() {
  if (!mqttClient.connected()) {
    return;
  }
  
  DynamicJsonDocument doc(256);
  
  doc["device_id"] = mqtt_client_id;
  doc["status"] = "online";
  doc["relay_state"] = relayState;
  doc["uptime_seconds"] = millis() / 1000;
  doc["free_heap"] = ESP.getFreeHeap();
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["mqtt_connected"] = mqttConnected;
  doc["reading_count"] = readingCount;
  doc["mqtt_publishes"] = mqttPublishCount;
  doc["thingspeak_publishes"] = thingspeakPublishCount;
  
  String payload;
  serializeJson(doc, payload);
  
  mqttClient.publish(STATUS_TOPIC, payload.c_str());
}

void publishDeviceInfo() {
  if (!mqttClient.connected()) {
    return;
  }
  
  DynamicJsonDocument doc(512);
  
  doc["device_id"] = mqtt_client_id;
  doc["type"] = "climate_sensor_relay";
  doc["name"] = "DHT22-ThingSpeak-MQTT";
  doc["version"] = "3.0.0";
  doc["features"]["thingspeak"] = true;
  doc["features"]["mqtt"] = true;
  doc["features"]["relay_control"] = true;
  doc["features"]["alerts"] = true;
  
  doc["sensors"]["dht22"]["pin"] = DHTPIN;
  doc["sensors"]["ldr"]["pin"] = LDR_PIN;
  doc["relay"]["pin"] = RELAY_PIN;
  
  doc["thresholds"]["temp_high"] = TEMP_HIGH_THRESHOLD;
  doc["thresholds"]["temp_low"] = TEMP_LOW_THRESHOLD;
  doc["thresholds"]["humidity_high"] = HUMIDITY_HIGH_THRESHOLD;
  doc["thresholds"]["humidity_low"] = HUMIDITY_LOW_THRESHOLD;
  
  doc["network"]["ip"] = WiFi.localIP().toString();
  doc["network"]["mac"] = WiFi.macAddress();
  
  String payload;
  serializeJson(doc, payload);
  
  mqttClient.publish(DEVICE_INFO_TOPIC, payload.c_str());
  Serial.println("📤 Device info published");
}

void checkRelayControl() {
  bool shouldActivateRelay = false;
  String reason = "";
  
  // Check temperature thresholds
  if (currentTemperature > TEMP_HIGH_THRESHOLD) {
    shouldActivateRelay = true;
    reason = "High temperature";
  } else if (currentTemperature < TEMP_LOW_THRESHOLD) {
    shouldActivateRelay = false;
    reason = "Low temperature";
  }
  
  // Check humidity thresholds
  if (currentHumidity > HUMIDITY_HIGH_THRESHOLD) {
    shouldActivateRelay = true;
    reason = "High humidity";
  } else if (currentHumidity < HUMIDITY_LOW_THRESHOLD) {
    shouldActivateRelay = false;
    reason = "Low humidity";
  }
  
  // Update relay if state changed
  if (shouldActivateRelay != relayState) {
    relayState = shouldActivateRelay;
    digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
    
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.printf("🎛️  RELAY: %s\n", relayState ? "ON" : "OFF");
    Serial.printf("   Reason: %s\n", reason.c_str());
    Serial.printf("   Temp: %.1f°C, Humidity: %.1f%%\n", 
                  currentTemperature, currentHumidity);
    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    
    // Send alert via MQTT
    if (mqttClient.connected()) {
      DynamicJsonDocument alertDoc(256);
      alertDoc["device_id"] = mqtt_client_id;
      alertDoc["alert_type"] = "relay_state_change";
      alertDoc["relay_state"] = relayState;
      alertDoc["reason"] = reason;
      alertDoc["temperature"] = currentTemperature;
      alertDoc["humidity"] = currentHumidity;
      
      String alertPayload;
      serializeJson(alertDoc, alertPayload);
      mqttClient.publish(ALERT_TOPIC, alertPayload.c_str());
    }
    
    blinkLED(3, 100);
  }
}

void checkAlerts() {
  String newAlert = "";
  
  // Temperature alerts
  if (currentTemperature > TEMP_HIGH_THRESHOLD) {
    newAlert = "HIGH_TEMP";
  } else if (currentTemperature < TEMP_LOW_THRESHOLD) {
    newAlert = "LOW_TEMP";
  }
  
  // Humidity alerts
  if (currentHumidity > HUMIDITY_HIGH_THRESHOLD) {
    newAlert = "HIGH_HUMIDITY";
  } else if (currentHumidity < HUMIDITY_LOW_THRESHOLD) {
    newAlert = "LOW_HUMIDITY";
  }
  
  // Send alert if changed
  if (!newAlert.isEmpty() && newAlert != alertMessage) {
    alertMessage = newAlert;
    
    if (mqttClient.connected()) {
      DynamicJsonDocument alertDoc(256);
      alertDoc["device_id"] = mqtt_client_id;
      alertDoc["alert_type"] = alertMessage;
      alertDoc["temperature"] = currentTemperature;
      alertDoc["humidity"] = currentHumidity;
      alertDoc["timestamp"] = millis() / 1000;
      
      String alertPayload;
      serializeJson(alertDoc, alertPayload);
      mqttClient.publish(ALERT_TOPIC, alertPayload.c_str());
      
      Serial.printf("🚨 ALERT: %s\n", alertMessage.c_str());
    }
  } else if (newAlert.isEmpty()) {
    alertMessage = "";
  }
}

void updateStatusLED() {
  static unsigned long lastLEDUpdate = 0;
  static bool ledState = false;
  
  unsigned long currentTime = millis();
  
  if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    // Connected: slow blink
    if (currentTime - lastLEDUpdate > 2000) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      lastLEDUpdate = currentTime;
    }
  } else {
    // Disconnected: fast blink
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

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.printf("📨 MQTT: [%s] %s\n", topic, message.c_str());
}

/*
 * System Architecture:
 * 
 * 1. Sensor Readings:
 *    - DHT22: Temperature & Humidity (every 2s)
 *    - LDR: Light intensity (every 2s)
 * 
 * 2. Data Publishing:
 *    - ThingSpeak: Every 20 seconds (rate limit compliance)
 *    - MQTT: Every 5 seconds (garden/sensor/temp_hum_data)
 * 
 * 3. Relay Control (Automatic):
 *    - Temperature > 30°C → Relay ON
 *    - Temperature < 15°C → Relay OFF
 *    - Humidity > 75% → Relay ON
 *    - Humidity < 40% → Relay OFF
 * 
 * 4. Alert System:
 *    - Publishes to garden/alerts/climate when thresholds exceeded
 *    - Includes relay state changes
 * 
 * 5. Status Monitoring:
 *    - Device status published every 30 seconds
 *    - Includes uptime, connection status, statistics
 */
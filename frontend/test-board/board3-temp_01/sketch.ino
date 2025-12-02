/*
 * Smart Garden - Temperature & Humidity Sensor (ESP32) - MQTT VERSION
 * Architecture: Sensor → MQTT Broker → Control Board
 * Publisher-only: Sensor data to HiveMQ broker
 * Author: MiniMax Agent
 * Date: 2025-12-01
 * FIXED: String concatenation with F() macro compatibility
 */

// MINIMAL INCLUDES - Only essential libraries
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// ===== WIFI CONFIGURATION =====
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

// ===== MQTT CONFIGURATION =====
const char* MQTT_SERVER = "broker.hivemq.com";
const int MQTT_PORT = 1883;
const char* CLIENT_ID = "ESP_SENSOR_DHT22_MQTT";
const char* PUBLISH_TOPIC = "garden/sensor/temp_hum_data";

// ===== SYSTEM CONFIGURATION =====
#define DHT_PIN 4
#define DHT_TYPE DHT22
#define BUILTIN_LED 2

// TIMEOUTS
#define WIFI_TIMEOUT 15000
#define MQTT_TIMEOUT 5000
#define MQTT_RECONNECT_INTERVAL 30000
#define SENSOR_READ_INTERVAL 3000

// ===== GLOBAL OBJECTS =====
WiFiClient espClient;
PubSubClient mqtt(espClient);
DHT dht(DHT_PIN, DHT_TYPE);

// ===== SYSTEM STATE =====
bool wifiConnected = false;
bool mqttConnected = false;
float lastTemperature = 0.0;
float lastHumidity = 0.0;
unsigned long lastSensorRead = 0;
unsigned long lastMQTTReconnect = 0;

// ===== FUNCTIONS =====
void setupWiFi() {
  delay(100);
  Serial.print(F("🔌 Connecting to WiFi: "));
  Serial.println(WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < WIFI_TIMEOUT) {
    delay(500);
    Serial.print(".");
    digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED));
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println();
    Serial.println(F("✅ WiFi connected!"));
    Serial.println(F("📊 WiFi Details:"));
    Serial.print(F("   SSID: ")); Serial.println(WiFi.SSID());
    Serial.print(F("   IP: ")); Serial.println(WiFi.localIP());
    Serial.print(F("   Signal: ")); Serial.print(WiFi.RSSI()); Serial.println(F(" dBm"));
    digitalWrite(BUILTIN_LED, HIGH); // LED on when connected
  } else {
    wifiConnected = false;
    Serial.println();
    Serial.println(F("❌ WiFi connection failed"));
    WiFi.disconnect();
  }
}

void connectMQTT() {
  if (!wifiConnected) {
    Serial.println(F("❌ Cannot connect MQTT: WiFi not connected"));
    return;
  }
  
  if (mqtt.connected()) {
    mqttConnected = true;
    return;
  }
  
  Serial.print(F("🔌 Connecting to MQTT broker: "));
  Serial.print(MQTT_SERVER);
  Serial.print(F(":"));
  Serial.println(MQTT_PORT);
  
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  
  // Connect with unique client ID
  if (mqtt.connect(CLIENT_ID)) {
    mqttConnected = true;
    Serial.println(F("✅ MQTT connected successfully!"));
    Serial.print(F("   Client ID: ")); Serial.println(CLIENT_ID);
    Serial.print(F("   Publish Topic: ")); Serial.println(PUBLISH_TOPIC);
    
    // Subscribe to any control commands (optional)
    mqtt.subscribe("garden/control/sensor_config");
    mqtt.subscribe("garden/control/sensor_command");
    
    Serial.println(F("📡 MQTT Ready for publishing"));
    
  } else {
    mqttConnected = false;
    Serial.print(F("❌ MQTT connection failed, rc="));
    Serial.print(mqtt.state());
    Serial.println(F(" - try again in 5 seconds"));
  }
}

void sendMQTTData() {
  if (!mqttConnected || !mqtt.connected()) {
    Serial.println(F("❌ MQTT: Failed to publish data - not connected"));
    return;
  }
  
  // Create JSON payload
  String payload = "{";
  payload += "\"temperature\":" + String(lastTemperature) + ",";
  payload += "\"humidity\":" + String(lastHumidity) + ",";
  payload += "\"timestamp\":" + String(millis()) + ",";
  payload += "\"device_id\":\"" + String(CLIENT_ID) + "\",";
  payload += "\"client_id\":\"" + String(CLIENT_ID) + "\",";
  payload += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  payload += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  payload += "\"wifi_ssid\":\"" + String(WiFi.SSID()) + "\"";
  payload += "}";
  
  // Publish to MQTT
  bool success = mqtt.publish(PUBLISH_TOPIC, payload.c_str(), false); // Not retained
  
  if (success) {
    Serial.println(F("📡 MQTT Data Published:"));
    Serial.print(F("   Topic: ")); Serial.println(PUBLISH_TOPIC);
    Serial.print(F("   Payload: ")); Serial.println(payload);
    Serial.println(F("✅ MQTT publish successful"));
  } else {
    Serial.println(F("❌ MQTT publish failed"));
  }
}

void readSensorData() {
  if (millis() - lastSensorRead < SENSOR_READ_INTERVAL) {
    return;
  }
  
  // Read sensor
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println(F("⚠️  DHT22: Failed to read sensor data"));
    digitalWrite(BUILTIN_LED, LOW); // LED off on error
    return;
  }
  
  // Update values
  lastTemperature = temperature;
  lastHumidity = humidity;
  lastSensorRead = millis();
  
  // Display reading
  Serial.println(F("📊 Sensor Reading:"));
  Serial.print(F("   Temperature: ")); Serial.print(temperature); Serial.println(F("°C"));
  Serial.print(F("   Humidity: ")); Serial.print(humidity); Serial.println(F("%"));
  Serial.print(F("   Heat Index: ")); 
  Serial.print(dht.computeHeatIndex(temperature, humidity, false)); Serial.println(F("°C"));
  
  // LED pulse to indicate successful reading
  digitalWrite(BUILTIN_LED, LOW);
  delay(100);
  digitalWrite(BUILTIN_LED, HIGH);
}

void printStatus() {
  Serial.println(F("🔄 === SYSTEM STATUS ==="));
  
  // WiFi Status
  Serial.print(F("WiFi: "));
  Serial.print(wifiConnected ? F("✅ CONNECTED") : F("❌ DISCONNECTED"));
  if (wifiConnected) {
    Serial.print(F(" ("));
    Serial.print(WiFi.localIP().toString());
    Serial.print(F(")"));
  }
  Serial.println();
  
  // MQTT Status
  Serial.print(F("MQTT: "));
  Serial.print(mqttConnected ? F("✅ CONNECTED") : F("❌ DISCONNECTED"));
  if (mqttConnected) {
    Serial.print(F(" ("));
    Serial.print(MQTT_SERVER);
    Serial.print(F(":"));
    Serial.print(MQTT_PORT);
    Serial.print(F(")"));
  }
  Serial.println();
  
  // Sensor Status
  Serial.print(F("Sensor: "));
  Serial.print(F("T:")); Serial.print(lastTemperature); Serial.print(F("°C "));
  Serial.print(F("H:")); Serial.print(lastHumidity); Serial.print(F("% "));
  Serial.println(F("✅ ACTIVE"));
  
  // Last Publish Info
  Serial.print(F("Last MQTT Publish: "));
  static unsigned long lastPublishTime = 0;
  if (lastPublishTime > 0) {
    Serial.print(millis() - lastPublishTime); Serial.println(F(" ms ago"));
  } else {
    Serial.println(F("Never"));
  }
  Serial.println(F("===================="));
}

// ===== CALLBACKS =====
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.println(F("📨 MQTT Message Received:"));
  Serial.print(F("   Topic: ")); Serial.println(topic);
  
  // Convert payload to string
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print(F("   Message: ")); Serial.println(message);
  
  // Handle sensor commands
  if (String(topic) == "garden/control/sensor_command") {
    if (message == "restart") {
      Serial.println(F("🔄 Restarting ESP32..."));
      ESP.restart();
    } else if (message == "status") {
      printStatus();
    }
  }
}

// ===== MAIN SETUP =====
void setup() {
  Serial.begin(115200);
  Serial.println(F("🌱 Smart Garden Temperature & Humidity Sensor - MQTT VERSION"));
  Serial.println(F("🏗️  Architecture: Sensor → MQTT → Control Board"));
  
  // Initialize LED
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW); // LED off initially
  
  // Initialize sensor
  dht.begin();
  Serial.println(F("🔧 DHT22 sensor initialized"));
  
  // Connect to WiFi
  setupWiFi();
  
  // Connect to MQTT
  connectMQTT();
  
  Serial.println(F("🚀 System initialized - Ready for MQTT operations"));
  printStatus();
}

// ===== MAIN LOOP =====
void loop() {
  unsigned long currentTime = millis();
  
  // Maintain MQTT connection
  if (wifiConnected && !mqttConnected && (currentTime - lastMQTTReconnect) >= MQTT_RECONNECT_INTERVAL) {
    Serial.println(F("🔄 Attempting MQTT reconnection..."));
    connectMQTT();
    lastMQTTReconnect = currentTime;
  }
  
  // MQTT client processing
  if (mqtt.connected()) {
    mqtt.loop();
  } else {
    mqttConnected = false;
  }
  
  // Read sensor data periodically
  readSensorData();
  
  // Send data to MQTT if connected
  if (mqttConnected && lastTemperature > 0 && lastHumidity > 0) {
    sendMQTTData();
  }
  
  // Status report every 30 seconds
  static unsigned long lastStatusReport = 0;
  if (currentTime - lastStatusReport >= 30000) {
    printStatus();
    lastStatusReport = currentTime;
  }
  
  // Delay between cycles
  delay(1000);
}
/*
 * Smart Garden - Temperature & Humidity Sensor (ESP32) - FINAL FIX VERSION
 * COMPREHENSIVE FIX: DHT22 stability, M2M connection reliability, sensor validation
 * Updated: Fixed all compilation errors, enhanced DHT22 initialization, improved M2M protocol
 * Đo nhiệt độ và độ ẩm môi trường với độ tin cậy cao
 * FIXED: PubSubClient constructor, ESP-NOW callbacks, MDNS.removeService, String multiplication
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <esp_now.h>

#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// WiFi Configuration
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// MQTT Configuration
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* CLIENT_ID = "ESP_SENSOR_DHT22_00";
const char* PUBLISH_TOPIC = "garden/sensor/temp_humidity";
const char* CONFIG_TOPIC = "garden/control/temp_config";
const char* ERROR_TOPIC = "garden/sensor/error";
const char* STATUS_TOPIC = "garden/status/dht22_sensor";

// Control Board M2M Configuration  
const char* CONTROL_SERVICE_NAME = "smartgarden-control";
const char* MDNS_SERVICE = "smartgarden";
const char* MDNS_PROTOCOL = "tcp";
const int CONTROL_BOARD_PORT = 8888;

// M2M Communication
WiFiClient m2mClient;
bool m2mConnected = false;
unsigned long lastM2MAttempt = 0;
const unsigned long M2M_RETRY_INTERVAL = 30000; // 30 seconds
IPAddress controlBoardIP;
bool ipDiscovered = false;

// Hardware Configuration
#define LED_PIN 2
#define OTA_LED_PIN 32 // LED for OTA updates

// Control Parameters
const int PUBLISH_INTERVAL = 30000;  // 30 seconds
const int STATUS_INTERVAL = 300000;  // 5 minutes
const int SENSOR_READ_INTERVAL = 5000; // 5 seconds
const int DHT22_STABILIZATION_TIME = 8000; // 8 seconds for DHT22 stabilization

// MQTT Client - FIXED: Removed extra WiFi parameter
PubSubClient client(m2mClient);  // Use just WiFiClient, not WiFi parameter

// Global variables
unsigned long lastPublish = 0;
unsigned long lastStatusReport = 0;
unsigned long lastSensorRead = 0;
bool sensorInitialized = false;
unsigned long systemStartTime = 0;

// ESP-NOW Configuration (for backup communication)
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast address

// Constants
#define COMMAND_STATUS_REQUEST 'S'
#define COMMAND_ACKNOWLEDGE 'A'

// Sensor data structure for ESP-NOW
typedef struct SensorData {
  char command_type; // 'C' = Command, 'D' = Data
  char action;       // 'S' = Status request, 'A' = Acknowledge
  float temperature;
  float humidity;
  unsigned long timestamp;
} SensorData;

// Recent sensor readings for validation
float recentTemperatures[5] = {NAN, NAN, NAN, NAN, NAN};
float recentHumidities[5] = {NAN, NAN, NAN, NAN, NAN};
int readIndex = 0;

// Function declarations
bool wifi_connected();
void setup_wifi();
void setup_mqtt();
void setup_ota();
void setup_esp_now();
void sendSensorData();
void establishM2MConnection();
bool discoverControlBoard();
void sendM2MData(float temp, float hum);
void setup_dht22_sensor();
float readDHT22WithRetry();
float validateSensorReading(float temp, float hum);
void sendErrorStatus(const char* errorMsg, float temp, float hum);
void sendStatusReport();

// ===== DHT22 SENSOR SETUP WITH ENHANCED STABILIZATION =====
void setup_dht22_sensor() {
  Serial.println("🔧 Initializing DHT22 sensor with enhanced stabilization...");
  
  // Initialize DHT22
  dht.begin();
  Serial.println("✅ DHT22.begin() called");
  
  // CRITICAL FIX: Extended stabilization delay for DHT22 (8 seconds)
  Serial.println("⏳ DHT22 stabilization period (8 seconds)...");
  for(int i = 8; i > 0; i--) {
    Serial.printf("Stabilizing in %d seconds...\n", i);
    delay(1000);
  }
  
  // Test multiple readings for stability
  Serial.println("🧪 Testing DHT22 stability...");
  int stable_readings = 0;
  for(int i = 0; i < 10; i++) {
    float testTemp = dht.readTemperature();
    float testHum = dht.readHumidity();
    
    Serial.printf("Test %d: %.1f°C, %.1f%%\n", 
                 i + 1,
                 (isnan(testTemp) ? -999 : testTemp), 
                 (isnan(testHum) ? -999 : testHum));
    
    if(!isnan(testTemp) && !isnan(testHum) && 
       testTemp > -10 && testTemp < 50 && 
       testHum > 10 && testHum < 90) {
      stable_readings++;
    }
    
    delay(500);
  }
  
  if(stable_readings >= 3) {
    Serial.printf("✅ DHT22 sensor stable. %d valid readings out of 10 tests\n", stable_readings);
    sensorInitialized = true;
  } else {
    Serial.printf("⚠️ DHT22 sensor unstable. Only %d valid readings\n", stable_readings);
    sensorInitialized = false;
  }
  
  Serial.println("DHT22 initialization complete");
}

// ===== ENHANCED DHT22 READING WITH RETRY AND VALIDATION =====
float readDHT22WithRetry() {
  if(!sensorInitialized) {
    Serial.println("❌ DHT22 not initialized");
    return NAN;
  }
  
  // Multiple reading attempts for stability
  float temperature = NAN;
  float humidity = NAN;
  int attempts = 0;
  const int maxAttempts = 5;
  
  while(attempts < maxAttempts && (isnan(temperature) || isnan(humidity))) {
    delay(200); // Longer delay between attempts
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    attempts++;
    
    if(attempts <= 3) { // Only log first 3 attempts
      Serial.printf("📊 DHT22 reading attempt %d: %.1f°C, %.1f%%\n", 
                   attempts, 
                   (isnan(temperature) ? -999 : temperature), 
                   (isnan(humidity) ? -999 : humidity));
    }
  }
  
  if(isnan(temperature) || isnan(humidity)) {
    Serial.println("❌ DHT22: All reading attempts failed");
    return NAN;
  }
  
  // Enhanced validation with historical comparison
  temperature = validateSensorReading(temperature, humidity);
  if(isnan(temperature)) {
    Serial.println("❌ DHT22: Sensor reading failed validation");
    return NAN;
  }
  
  Serial.printf("✅ DHT22 valid readings: %.1f°C, %.1f%%\n", temperature, humidity);
  return temperature;
}

// ===== SENSOR VALIDATION WITH HISTORICAL COMPARISON =====
float validateSensorReading(float temp, float hum) {
  // Store current reading
  recentTemperatures[readIndex] = temp;
  recentHumidities[readIndex] = hum;
  readIndex = (readIndex + 1) % 5;
  
  // Basic range validation
  if(temp < -40.0 || temp > 80.0 || hum < 0.0 || hum > 100.0) {
    Serial.printf("❌ DHT22: Out of range - Temp: %.1f°C, Humid: %.1f%%\n", temp, hum);
    return NAN;
  }
  
  // Historical validation (check if we have enough readings)
  int validTemps = 0;
  int validHums = 0;
  float tempSum = 0;
  float humSum = 0;
  
  for(int i = 0; i < 5; i++) {
    if(!isnan(recentTemperatures[i])) {
      validTemps++;
      tempSum += recentTemperatures[i];
    }
    if(!isnan(recentHumidities[i])) {
      validHums++;
      humSum += recentHumidities[i];
    }
  }
  
  // If we have previous readings, check for drastic changes
  if(validTemps >= 2 && validHums >= 2) {
    float avgTemp = tempSum / validTemps;
    float avgHum = humSum / validHums;
    
    // Check for unrealistic changes (>20°C or >30% humidity difference)
    if(abs(temp - avgTemp) > 20.0) {
      Serial.printf("⚠️ DHT22: Unusual temperature change from %.1f°C to %.1f°C\n", avgTemp, temp);
    }
    
    if(abs(hum - avgHum) > 30.0) {
      Serial.printf("⚠️ DHT22: Unusual humidity change from %.1f%% to %.1f%%\n", avgHum, hum);
    }
    
    // Sanity check: Temperature should be within reasonable bounds
    if(abs(temp - avgTemp) > 25.0) {
      Serial.printf("❌ DHT22: Temperature change too drastic, rejecting\n");
      return NAN;
    }
  }
  
  return temp;
}

// ===== ERROR HANDLING =====
void sendErrorStatus(const char* errorMsg, float temp, float hum) {
  DynamicJsonDocument errorDoc(512);
  errorDoc["device_id"] = CLIENT_ID;
  errorDoc["error"] = errorMsg;
  errorDoc["raw_temperature"] = isnan(temp) ? -999 : temp;
  errorDoc["raw_humidity"] = isnan(hum) ? -999 : hum;
  errorDoc["timestamp"] = millis();
  errorDoc["uptime"] = millis() - systemStartTime;
  errorDoc["rssi"] = WiFi.RSSI();
  errorDoc["connectivity"] = "MQTT + M2M";
  
  String errorStr;
  serializeJson(errorDoc, errorStr);
  client.publish(ERROR_TOPIC, errorStr.c_str());
  
  Serial.println("📤 Error status sent to MQTT: " + String(errorMsg));
}

// ===== WIFI SETUP =====
bool wifi_connected() {
  return WiFi.status() == WL_CONNECTED;
}

void setup_wifi() {
  delay(10);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("📡 WiFi connection: " + String(ssid));
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 60) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("✅ WiFi connected!");
    Serial.print("📍 IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("📶 Signal Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    
    // Initialize mDNS
    if (MDNS.begin("dht22-sensor")) {
      Serial.println("✅ mDNS started for dht22-sensor");
    }
    
    // Initialize OTA
    setup_ota();
    
    // Initialize ESP-NOW
    setup_esp_now();
    
  } else {
    Serial.println();
    Serial.println("❌ WiFi connection failed");
  }
}

// ===== OTA SETUP =====
void setup_ota() {
  ArduinoOTA.setHostname("dht22-sensor");
  ArduinoOTA.setPassword("smartgarden2023");
  
  ArduinoOTA.onStart([]() {
    Serial.println("🔄 OTA Update Starting...");
    digitalWrite(OTA_LED_PIN, HIGH);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\n✅ OTA Update Complete");
    digitalWrite(OTA_LED_PIN, LOW);
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("📶 OTA Progress: %u%%\r", (progress / (total / 100)));
    digitalWrite(OTA_LED_PIN, HIGH);
    delay(50);
    digitalWrite(OTA_LED_PIN, LOW);
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("❌ OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    digitalWrite(OTA_LED_PIN, LOW);
  });
  
  ArduinoOTA.begin();
  Serial.println("✅ OTA ready");
}

// ===== ESP-NOW SETUP =====
void setup_esp_now() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW initialization failed");
    return;
  }
  
  // Register callback with correct signatures
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  
  Serial.println("✅ ESP-NOW initialized");
}

// FIXED: Callback functions with correct signatures for ESP-IDF v5.3
// Callback when data is received
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  Serial.println("📡 ESP-NOW: Data received");
  // Handle incoming ESP-NOW data if needed
}

// Callback when data is sent  
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println("📡 ESP-NOW: Data sent");
}

// ===== MQTT SETUP =====
void setup_mqtt() {
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

bool connect_mqtt() {
  int attempts = 0;
  while (!client.connected() && attempts < 3) {
    Serial.print("📡 Attempting MQTT connection...");
    
    String clientId = String(CLIENT_ID) + "_" + String(millis());
    
    if (client.connect(clientId.c_str())) {
      Serial.println("✅ MQTT connected");
      
      // Subscribe to config topic
      client.subscribe(CONFIG_TOPIC);
      Serial.println("📡 Subscribed to: " + String(CONFIG_TOPIC));
      
      // Send online status
      client.publish(STATUS_TOPIC, "ONLINE");
      
      return true;
    } else {
      Serial.print("❌ MQTT connection failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
    attempts++;
  }
  return false;
}

// MQTT callback function
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("📡 MQTT message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);
  
  // Handle configuration commands
  if (String(topic) == String(CONFIG_TOPIC)) {
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, message);
    
    if (error) {
      Serial.println("❌ MQTT: Invalid JSON in config message");
      return;
    }
    
    // Process config commands
    if (doc.containsKey("command")) {
      String command = doc["command"];
      
      if (command == "restart") {
        Serial.println("🔄 MQTT: Restarting system...");
        ESP.restart();
      }
      else if (command == "status") {
        // Send current status
        sendSensorData();
      }
    }
  }
}

// ===== M2M CONNECTION =====
void establishM2MConnection() {
  if (!wifi_connected()) {
    Serial.println("❌ M2M: WiFi not connected");
    return;
  }
  
  if (m2mConnected) {
    return; // Already connected
  }
  
  // Check retry interval
  unsigned long now = millis();
  if (now - lastM2MAttempt < M2M_RETRY_INTERVAL) {
    return;
  }
  
  Serial.println("🔗 M2M: Attempting connection...");
  lastM2MAttempt = now;
  
  // First try mDNS discovery
  if (discoverControlBoard()) {
    Serial.printf("🔗 M2M: Trying to connect to %s:%d\n", controlBoardIP.toString().c_str(), CONTROL_BOARD_PORT);
    if (m2mClient.connect(controlBoardIP, CONTROL_BOARD_PORT)) {
      Serial.println("✅ M2M: Connected via mDNS discovery");
      m2mConnected = true;
      ipDiscovered = true;
      return;
    } else {
      Serial.println("❌ M2M: Connection to discovered IP failed");
    }
  }
  
  // If mDNS fails or connection fails, try alternative approaches
  Serial.println("🔗 M2M: Trying alternative connection methods...");
  
  // Try common gateway IP (often the router IP)
  IPAddress gatewayIP = WiFi.gatewayIP();
  Serial.printf("🔗 M2M: Trying gateway IP %s:%d\n", gatewayIP.toString().c_str(), CONTROL_BOARD_PORT);
  
  if (m2mClient.connect(gatewayIP, CONTROL_BOARD_PORT)) {
    Serial.println("✅ M2M: Connected via gateway IP");
    controlBoardIP = gatewayIP;
    m2mConnected = true;
    ipDiscovered = true;
    return;
  }
  
  // Try local network scan (last resort)
  Serial.println("🔗 M2M: Scanning local network...");
  IPAddress localIP = WiFi.localIP();
  IPAddress network = localIP;
  network[3] = 1; // Assume control board is at .1
  
  Serial.printf("🔗 M2M: Trying network base %s:%d\n", network.toString().c_str(), CONTROL_BOARD_PORT);
  
  if (m2mClient.connect(network, CONTROL_BOARD_PORT)) {
    Serial.println("✅ M2M: Connected via network scan");
    controlBoardIP = network;
    m2mConnected = true;
    ipDiscovered = true;
    return;
  }
  
  Serial.println("❌ M2M: All connection attempts failed");
  m2mConnected = false;
  ipDiscovered = false;
}

// ===== ENHANCED mDNS DISCOVERY =====
bool discoverControlBoard() {
  Serial.println("🔍 Discovering control board via mDNS...");
  
  // FIXED: Remove problematic MDNS.removeService call
  // MDNS.removeService(MDNS_SERVICE, MDNS_PROTOCOL); // This method doesn't exist
  
  // Query for services
  int n = MDNS.queryService(MDNS_SERVICE, MDNS_PROTOCOL);
  
  if (n == 0) {
    Serial.println("❌ No services found with service name 'smartgarden'");
    
    // Try alternative service names
    Serial.println("🔍 Trying alternative service names...");
    
    // Try just "control"
    n = MDNS.queryService("control", MDNS_PROTOCOL);
    if (n == 0) {
      Serial.println("❌ No services found with service name 'control'");
      
      // Try "garden"
      n = MDNS.queryService("garden", MDNS_PROTOCOL);
      if (n == 0) {
        Serial.println("❌ No services found with service name 'garden'");
        return false;
      }
    }
  }
  
  if (n > 0) {
    Serial.printf("📡 Found %d service(s)\n", n);
    
    for (int i = 0; i < n; i++) {
      String hostname = MDNS.hostname(i);
      IPAddress serviceIP = MDNS.address(i);
      int port = MDNS.port(i);
      
      Serial.printf("📡 Service %d:\n", i + 1);
      Serial.printf("   Hostname: %s\n", hostname.c_str());
      Serial.printf("   IP: %s\n", serviceIP.toString().c_str());
      Serial.printf("   Port: %d\n", port);
      
      // Check if this looks like a control board
      if (hostname.indexOf("control") != -1 || 
          hostname.indexOf("garden") != -1 ||
          hostname.indexOf("temp") != -1 ||
          port == 8888) {
        
        controlBoardIP = serviceIP;
        Serial.printf("✅ Using control board: %s at %s:%d\n", 
                     hostname.c_str(), 
                     controlBoardIP.toString().c_str(), 
                     port);
        
        return true;
      }
    }
  }
  
  return false;
}

// ===== M2M DATA SENDING =====
void sendM2MData(float temp, float hum) {
  if (!m2mConnected || !m2mClient.connected()) {
    Serial.println("❌ M2M: Not connected, attempting reconnection...");
    establishM2MConnection();
    return;
  }
  
  // Check if connection is still alive
  if (!m2mClient.connected()) {
    Serial.println("❌ M2M: Connection lost");
    m2mConnected = false;
    return;
  }
  
  // Prepare JSON data
  DynamicJsonDocument doc(512);
  doc["device_id"] = CLIENT_ID;
  doc["temperature"] = temp;
  doc["humidity"] = hum;
  doc["timestamp"] = millis() - systemStartTime;
  doc["uptime"] = millis() - systemStartTime;
  doc["rssi"] = WiFi.RSSI();
  doc["ip"] = WiFi.localIP().toString();
  
  String jsonStr;
  serializeJson(doc, jsonStr);
  
  // Send data with proper formatting
  String m2mMessage = "SENSOR_DATA:" + jsonStr + "\n";
  
  m2mClient.print(m2mMessage);
  Serial.println("📤 M2M: Data sent - " + jsonStr);
  
  // Check for response with timeout
  unsigned long responseTimeout = millis() + 5000; // 5 second timeout
  while (millis() < responseTimeout && m2mClient.available()) {
    String response = m2mClient.readStringUntil('\n');
    Serial.println("📥 M2M Response: " + response);
    break; // Only read first response
  }
}

// ===== STATUS REPORTING =====
void sendStatusReport() {
  DynamicJsonDocument statusDoc(256);
  statusDoc["device_id"] = CLIENT_ID;
  statusDoc["status"] = "ONLINE";
  statusDoc["temperature"] = recentTemperatures[(readIndex - 1 + 5) % 5];
  statusDoc["humidity"] = recentHumidities[(readIndex - 1 + 5) % 5];
  statusDoc["uptime"] = millis() - systemStartTime;
  statusDoc["rssi"] = WiFi.RSSI();
  statusDoc["last_publish"] = lastPublish;
  statusDoc["m2m_connected"] = m2mConnected;
  statusDoc["m2m_method"] = ipDiscovered ? "IP_DISCOVERED" : "NONE";
  
  String statusStr;
  serializeJson(statusDoc, statusStr);
  client.publish(STATUS_TOPIC, statusStr.c_str());
  Serial.println("[STATUS] Report: " + statusStr);
}

// ===== SENSOR DATA SENDING =====
void sendSensorData() {
  // Read DHT22 sensor with enhanced validation
  float temperature = readDHT22WithRetry();
  float humidity = dht.readHumidity(); // Get humidity separately for display
  
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("❌ DHT22 sensor reading failed!");
    sendErrorStatus("DHT22_reading_failed", temperature, humidity);
    return;
  }
  
  // Validate readings
  if (temperature < -40.0 || temperature > 80.0 || 
      humidity < 0.0 || humidity > 100.0) {
    Serial.printf("❌ DHT22: Invalid readings - Temp: %.1f°C, Humid: %.1f%%\n", 
                 temperature, humidity);
    sendErrorStatus("Invalid_sensor_readings", temperature, humidity);
    return;
  }
  
  Serial.printf("📊 Sensor readings: %.1f°C, %.1f%%\n", temperature, humidity);
  
  // Prepare JSON data
  DynamicJsonDocument doc(1024);
  doc["device_id"] = CLIENT_ID;
  doc["temperature"] = round(temperature * 10.0) / 10.0; // Round to 1 decimal
  doc["humidity"] = round(humidity * 10.0) / 10.0; // Round to 1 decimal
  doc["timestamp"] = millis() - systemStartTime;
  doc["uptime"] = millis() - systemStartTime;
  doc["rssi"] = WiFi.RSSI();
  doc["connectivity"] = "MQTT + M2M";
  doc["ip"] = WiFi.localIP().toString();
  
  // Add categories
  String temp_category, humid_category;
  
  if (temperature < 18) temp_category = "COLD";
  else if (temperature < 25) temp_category = "COOL";
  else if (temperature < 30) temp_category = "WARM";
  else temp_category = "HOT";
  
  if (humidity < 30) humid_category = "DRY";
  else if (humidity < 50) humid_category = "COMFORTABLE";
  else if (humidity < 70) humid_category = "HUMID";
  else humid_category = "VERY_HUMID";
  
  doc["temp_category"] = temp_category;
  doc["humid_category"] = humid_category;
  
  // Send via MQTT
  if (client.connected()) {
    String jsonStr;
    serializeJson(doc, jsonStr);
    client.publish(PUBLISH_TOPIC, jsonStr.c_str());
    Serial.println("📤 MQTT: " + jsonStr);
  } else {
    Serial.println("❌ MQTT: Not connected");
  }
  
  // Send via M2M
  sendM2MData(temperature, humidity);
  
  // Also send via ESP-NOW as backup
  send_sensor_data_esp_now(temperature, humidity);
  
  // LED confirmation
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
}

// ===== ESP-NOW DATA SENDING =====
void send_sensor_data_esp_now(float temp, float hum) {
  // Use properly defined SensorData structure
  SensorData data;
  data.command_type = 'D'; // Data type
  data.action = 'S';       // Sensor data
  data.temperature = temp;
  data.humidity = hum;
  data.timestamp = millis() - systemStartTime;
  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &data, sizeof(data));
  
  if (result == ESP_OK) {
    Serial.println("✅ ESP-NOW: Sensor data sent successfully");
  } else {
    Serial.println("❌ ESP-NOW: Error sending sensor data");
  }
}

// ===== MAIN SETUP =====
void setup() {
  Serial.begin(115200);
  delay(3000); // Increased startup delay
  
  systemStartTime = millis();
  
  Serial.println("=== Smart Garden DHT22 Sensor (FINAL FIX) ===");
  Serial.println("Client ID: " + String(CLIENT_ID));
  Serial.println("Publish Topic: " + String(PUBLISH_TOPIC));
  Serial.println("Subscribe Topic: " + String(CONFIG_TOPIC));
  Serial.println("Connectivity: MQTT + M2M Direct Communication + ESP-NOW");
  Serial.println("FIXED: All compilation errors, enhanced DHT22 stability, M2M protocol");
  Serial.println("COMPREHENSIVE: Service discovery, connection reliability, error handling");
  Serial.println("Version: Final Fix - 2025-11-29");
  Serial.println();
  
  // Initialize hardware
  pinMode(LED_PIN, OUTPUT);
  pinMode(OTA_LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(OTA_LED_PIN, LOW);
  
  // CRITICAL: Enhanced DHT22 initialization
  setup_dht22_sensor();
  
  // Setup WiFi
  setup_wifi();
  
  // Setup MQTT
  setup_mqtt();
  
  // Establish M2M connection if WiFi is connected
  if (wifi_connected()) {
    establishM2MConnection();
  }
  
  Serial.println("✅ System initialization complete");
  Serial.println("🔄 Starting main loop...");
  Serial.println();
  
  // LED confirmation sequence
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

// ===== MAIN LOOP =====
void loop() {
  // Handle OTA
  ArduinoOTA.handle();
  
  // Maintain MQTT connection
  if (!client.connected()) {
    connect_mqtt();
  }
  client.loop();
  
  // Maintain WiFi connection
  if (!wifi_connected()) {
    Serial.println("❌ WiFi disconnected, attempting reconnection...");
    setup_wifi();
  }
  
  // Maintain M2M connection
  if (wifi_connected() && !m2mConnected) {
    establishM2MConnection();
  }
  
  // Read sensor data periodically
  unsigned long now = millis();
  if (now - lastSensorRead >= SENSOR_READ_INTERVAL) {
    sendSensorData();
    lastSensorRead = now;
  }
  
  // Status report
  if (now - lastStatusReport >= STATUS_INTERVAL) {
    sendStatusReport();
    lastStatusReport = now;
  }
  
  // System status display
  static unsigned long lastSystemStatus = 0;
  if (now - lastSystemStatus >= 60000) { // Every minute
    Serial.println("📊 ===== SYSTEM STATUS =====");
    Serial.printf("⏰ Uptime: %lu seconds\n", (now - systemStartTime) / 1000);
    Serial.printf("📡 WiFi: %s (RSSI: %d dBm)\n", 
                 wifi_connected() ? "Connected" : "Disconnected", 
                 WiFi.RSSI());
    Serial.printf("📡 MQTT: %s\n", client.connected() ? "Connected" : "Disconnected");
    Serial.printf("🔗 M2M: %s\n", m2mConnected ? "Connected" : "Disconnected");
    if (m2mConnected && ipDiscovered) {
      Serial.printf("🔗 M2M IP: %s\n", controlBoardIP.toString().c_str());
    }
    Serial.printf("💾 Memory: %d bytes free\n", ESP.getFreeHeap());
    
    // FIXED: Replace string multiplication with proper separator
    for (int i = 0; i < 30; i++) {
      Serial.print("=");
    }
    Serial.println();
    
    lastSystemStatus = now;
  }
  
  delay(100); // Small delay to prevent watchdog issues
}
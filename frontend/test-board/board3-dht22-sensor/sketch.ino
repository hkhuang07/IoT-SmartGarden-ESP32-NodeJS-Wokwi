/*
 * Smart Garden - Temperature & Humidity Sensor (ESP32)
 * Publish: garden/sensor/temp_humidity
 * Subscribe: garden/control/temp_config
 * Đo nhiệt độ và độ ẩm môi trường
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// WiFi Configuration
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// MQTT Configuration
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* CLIENT_ID = "ESP_SENSOR_DHT22_01";
const char* PUBLISH_TOPIC = "garden/sensor/temp_humidity";
const char* CONFIG_TOPIC = "garden/control/temp_config";

// Hardware Configuration
#define LED_PIN 2

// Control Parameters
const int PUBLISH_INTERVAL = 30000;  // 30 seconds
const int STATUS_INTERVAL = 300000;  // 5 minutes

// MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
long lastStatus = 0;

void setup_wifi() {
  delay(10);
  WiFi.begin(ssid, password);
  
  Serial.print("WiFi connection: " + String(ssid));
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 60) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("✅ WiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("❌ WiFi connection failed");
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println("[MQTT] Receive: " + String(topic) + " = " + message);
  
  // Process configuration commands if needed
  if (String(topic) == CONFIG_TOPIC) {
    // Add configuration handling here if needed
    Serial.println("[CONFIG] Processing configuration: " + message);
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT Connection...");
    
    if (client.connect(CLIENT_ID)) {
      Serial.println("✅ Connected");
      
      // Subscribe to config topic
      client.subscribe(CONFIG_TOPIC);
      
      // Send online status
      client.publish("garden/status/dht22_sensor", "ONLINE");
      
      // Send device info
      DynamicJsonDocument infoDoc(512);
      infoDoc["device_id"] = CLIENT_ID;
      infoDoc["type"] = "temperature_humidity_sensor";
      infoDoc["version"] = "2.0";
      infoDoc["sensor"] = "DHT22";
      infoDoc["ip"] = WiFi.localIP().toString();
      
      String infoStr;
      serializeJson(infoDoc, infoStr);
      client.publish("garden/system/device_info", infoStr.c_str());
      
    } else {
      Serial.print("Failure, rc=");
      Serial.print(client.state());
      Serial.println(" Try again in 3 second");
      delay(3000);
    }
  }
}

void sendSensorData() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("❌ DHT22 sensor reading error!");
    // Send error status
    DynamicJsonDocument errorDoc(256);
    errorDoc["device_id"] = CLIENT_ID;
    errorDoc["error"] = "DHT22_reading_failed";
    errorDoc["timestamp"] = millis();
    
    String errorStr;
    serializeJson(errorDoc, errorStr);
    client.publish("garden/sensor/error", errorStr.c_str());
    return;
  }
  
  // Create sensor data JSON
  DynamicJsonDocument doc(512);
  doc["device_id"] = CLIENT_ID;
  doc["temperature"] = round(temperature * 10.0) / 10.0; // Round to 1 decimal
  doc["humidity"] = round(humidity * 10.0) / 10.0; // Round to 1 decimal
  doc["timestamp"] = millis();
  doc["uptime"] = millis();
  doc["rssi"] = WiFi.RSSI();
  
  // Add temperature category
  String tempCategory;
  if (temperature < 18.0) tempCategory = "COOL";
  else if (temperature < 25.0) tempCategory = "OPTIMAL";
  else if (temperature < 30.0) tempCategory = "WARM";
  else tempCategory = "HOT";
  doc["temp_category"] = tempCategory;
  
  // Add humidity category
  String humidCategory;
  if (humidity < 40.0) humidCategory = "DRY";
  else if (humidity < 70.0) humidCategory = "NORMAL";
  else humidCategory = "HUMID";
  doc["humid_category"] = humidCategory;
  
  String payload;
  serializeJson(doc, payload);
  
  // Publish to MQTT
  client.publish(PUBLISH_TOPIC, payload.c_str());
  Serial.println("[SENSOR] Publish: " + payload);
  
  // Also publish to MongoDB topic
  String mongoPayload = "{";
  mongoPayload += "\"device_id\":\"" + String(CLIENT_ID) + "\",";
  mongoPayload += "\"sensor_type\":\"temperature_humidity\",";
  mongoPayload += "\"temperature\":" + String(temperature) + ",";
  mongoPayload += "\"humidity\":" + String(humidity) + ",";
  mongoPayload += "\"timestamp\":" + String(millis()) + ",";
  mongoPayload += "\"temp_category\":\"" + tempCategory + "\",";
  mongoPayload += "\"humid_category\":\"" + humidCategory + "\"";
  mongoPayload += "}";
  
  client.publish("garden/database/temp_humidity", mongoPayload.c_str());
  Serial.println("[MONGODB] Publish: " + mongoPayload);
}

void sendStatusReport() {
  DynamicJsonDocument statusDoc(256);
  statusDoc["device_id"] = CLIENT_ID;
  statusDoc["status"] = "ONLINE";
  statusDoc["temperature"] = dht.readTemperature();
  statusDoc["humidity"] = dht.readHumidity();
  statusDoc["uptime"] = millis();
  statusDoc["rssi"] = WiFi.RSSI();
  statusDoc["last_publish"] = lastMsg;
  
  String statusStr;
  serializeJson(statusDoc, statusStr);
  client.publish("garden/status/dht22_sensor", statusStr.c_str());
  Serial.println("[STATUS] Report: " + statusStr);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Smart Garden DHT22 Sensor ===");
  Serial.println("Client ID: " + String(CLIENT_ID));
  Serial.println("Publish Topic: " + String(PUBLISH_TOPIC));
  Serial.println("Subscribe Topic: " + String(CONFIG_TOPIC));
  
  // Initialize hardware
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize DHT22
  dht.begin();
  Serial.println("DHT22 initialized");
  
  // Setup WiFi
  setup_wifi();
  
  // Setup MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  Serial.println("Ready to go!");
  Serial.println("Publishing data every 30 seconds...");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  long now = millis();
  
  // Send sensor data every 30 seconds
  if (now - lastMsg > PUBLISH_INTERVAL) {
    lastMsg = now;
    sendSensorData();
    
    // LED blink to indicate data sent
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
  }
  
  // Send status report every 5 minutes
  if (now - lastStatus > STATUS_INTERVAL) {
    lastStatus = now;
    sendStatusReport();
  }
  
  delay(100);
}
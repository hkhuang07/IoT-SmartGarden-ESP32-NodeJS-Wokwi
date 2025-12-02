/*
 * Smart Garden - Board 1: Light Sensor Control (ESP32)
 * Chức năng: Đo ánh sáng, điều khiển đèn LED PWM, relay light, servo mái che, servo van nước
 * Áp dụng: C++, I2C/SPI, PWM, Interrupt
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
const char* CLIENT_ID = "ESP_BOARD1_LIGHT_01";
const char* PUBLISH_TOPIC = "garden/sensor/light";
const char* SUBSCRIBE_TOPIC = "garden/control/#";

// Hardware Configuration - Board 1
#define LDR_PIN 32
#define LED_PIN 2
#define RELAY_PIN 4
#define SERVO_ROOF_PIN 12
#define SERVO_VALVE_PIN 13
#define POTENTIOMETER_PIN 34
#define SWITCH1_PIN 25
#define SWITCH2_PIN 26
#define SWITCH3_PIN 27

// PWM Configuration
#define PWM_CHANNEL_LED 0
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8

// Servo Configuration
#include <ESP32Servo.h>
Servo roofServo;
Servo valveServo;

// I2C/SPI Configuration
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define SPI_MOSI_PIN 23
#define SPI_MISO_PIN 19
#define SPI_SCK_PIN 18
#define SPI_CS_PIN 5

// Control Parameters
const int LIGHT_LOW_THRESHOLD = 300;
const int LIGHT_HIGH_THRESHOLD = 900;
const int PUBLISH_INTERVAL = 10000;

// WatchDog Timer
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 30

// Interrupt Variables
volatile bool switch1Pressed = false;
volatile bool switch2Pressed = false;
volatile bool switch3Pressed = false;

// MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
bool autoLightControl = true;
bool autoRoofControl = true;
bool autoValveControl = true;

// Global Variables
int currentLEDValue = 0;
int currentRoofAngle = 0;
int currentValveAngle = 0;

// Interrupt Service Routines
void IRAM_ATTR switch1ISR() {
  switch1Pressed = true;
}

void IRAM_ATTR switch2ISR() {
  switch2Pressed = true;
}

void IRAM_ATTR switch3ISR() {
  switch3Pressed = true;
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

void setup_i2c_spi() {
  // I2C Setup
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  
  // SPI Setup
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN);
  pinMode(SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  
  Serial.println("✅ I2C/SPI initialized");
}

void setup_pwm() {
  // LED PWM Setup
  ledcSetup(PWM_CHANNEL_LED, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(LED_PIN, PWM_CHANNEL_LED);
  
  Serial.println("✅ PWM initialized");
}

void setup_servos() {
  roofServo.attach(SERVO_ROOF_PIN);
  valveServo.attach(SERVO_VALVE_PIN);
  
  // Set initial positions
  roofServo.write(0);  // Closed
  valveServo.write(0); // Closed
  currentRoofAngle = 0;
  currentValveAngle = 0;
  
  Serial.println("✅ Servos initialized");
}

void setup_interrupts() {
  pinMode(SWITCH1_PIN, INPUT_PULLUP);
  pinMode(SWITCH2_PIN, INPUT_PULLUP);
  pinMode(SWITCH3_PIN, INPUT_PULLUP);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(SWITCH1_PIN), switch1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(SWITCH2_PIN), switch2ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(SWITCH3_PIN), switch3ISR, FALLING);
  
  Serial.println("✅ Interrupts initialized");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println("[MQTT] Nhận: " + String(topic) + " = " + message);
  
  String topicStr = String(topic);
  
  // Xử lý cấu hình
  if (topicStr == "garden/control/light_config") {
    DynamicJsonDocument doc(512);
    deserializeJson(doc, message);
    
    if (doc.containsKey("auto_control")) {
      autoLightControl = doc["auto_control"];
      Serial.println("[CONFIG] Điều khiển đèn tự động: " + String(autoLightControl));
    }
    if (doc.containsKey("led_brightness")) {
      int brightness = doc["led_brightness"];
      controlLED(brightness);
    }
  }
  
  // Điều khiển đèn
  if (topicStr == "garden/control/light") {
    if (message == "ON") {
      controlLED(255);
    } else if (message == "OFF") {
      controlLED(0);
    } else if (message.toInt() > 0) {
      controlLED(message.toInt());
    }
  }
  
  // Điều khiển relay
  if (topicStr == "garden/control/relay_light") {
    digitalWrite(RELAY_PIN, message == "ON" ? HIGH : LOW);
    Serial.println("[RELAY] Light relay: " + message);
  }
  
  // Điều khiển mái che
  if (topicStr == "garden/control/roof_servo") {
    if (message == "OPEN") {
      controlRoofServo(180);
    } else if (message == "CLOSE") {
      controlRoofServo(0);
    }
  }
  
  // Điều khiển van nước
  if (topicStr == "garden/control/water_valve") {
    if (message == "OPEN") {
      controlValveServo(90);
    } else if (message == "CLOSE") {
      controlValveServo(0);
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT kết nối...");
    
    if (client.connect(CLIENT_ID)) {
      Serial.println("✅ Đã kết nối!");
      
      client.subscribe(SUBSCRIBE_TOPIC);
      
      client.publish("garden/status/board1", "ONLINE");
      
      DynamicJsonDocument infoDoc(512);
      infoDoc["device_id"] = CLIENT_ID;
      infoDoc["board"] = "BOARD1_LIGHT_CONTROL";
      infoDoc["version"] = "2.0";
      infoDoc["auto_control"] = autoLightControl;
      infoDoc["ip"] = WiFi.localIP().toString();
      infoDoc["features"] = "light_sensor,pwm_led,relay,servos,switches";
      
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
  // Đọc dữ liệu từ cảm biến
  int lightRaw = analogRead(LDR_PIN);
  int potValue = analogRead(POTENTIOMETER_PIN);
  float lightVoltage = (lightRaw / 4095.0) * 3.3;
  
  // Tạo JSON data
  DynamicJsonDocument doc(512);
  doc["l"] = lightRaw;
  doc["voltage"] = lightVoltage;
  doc["potentiometer"] = potValue;
  doc["led_value"] = currentLEDValue;
  doc["roof_angle"] = currentRoofAngle;
  doc["valve_angle"] = currentValveAngle;
  doc["auto_control"] = autoLightControl;
  doc["device_id"] = CLIENT_ID;
  doc["timestamp"] = millis();
  doc["rssi"] = WiFi.RSSI();
  doc["free_heap"] = ESP.getFreeHeap();
  
  String payload;
  serializeJson(doc, payload);
  
  client.publish(PUBLISH_TOPIC, payload.c_str());
  Serial.println("[SENSOR] Gửi: " + payload);
  
  if (autoLightControl) {
    handleAutoLightControl(lightRaw);
  }
}

void handleAutoLightControl(int lightValue) {
  static unsigned long lastAction = 0;
  
  if (millis() - lastAction < 30000) {
    return;
  }
  
  // Ánh sáng yếu - bật đèn và mở mái che
  if (lightValue < LIGHT_LOW_THRESHOLD) {
    controlLED(255);
    controlRoofServo(180);
    client.publish("garden/control/light", "ON");
    client.publish("garden/control/roof_servo", "OPEN");
    Serial.println("[AUTO] Ánh sáng yếu - Bật đèn và mở mái che");
  }
  // Ánh sáng đủ - tắt đèn và đóng mái che
  else if (lightValue >= LIGHT_LOW_THRESHOLD && lightValue < LIGHT_HIGH_THRESHOLD) {
    controlLED(128);
    controlRoofServo(90);
    client.publish("garden/control/light", "ON");
    client.publish("garden/control/roof_servo", "CLOSE");
    Serial.println("[AUTO] Ánh sáng vừa - Điều chỉnh đèn và mái che");
  }
  // Ánh sáng mạnh - tắt đèn và mở mái che
  else if (lightValue >= LIGHT_HIGH_THRESHOLD) {
    controlLED(0);
    controlRoofServo(180);
    client.publish("garden/control/light", "OFF");
    client.publish("garden/control/roof_servo", "OPEN");
    Serial.println("[AUTO] Ánh sáng mạnh - Tắt đèn và mở mái che");
  }
  
  lastAction = millis();
}

void controlLED(int value) {
  currentLEDValue = constrain(value, 0, 255);
  ledcWrite(PWM_CHANNEL_LED, currentLEDValue);
  Serial.println("[LED] Độ sáng: " + String(currentLEDValue));
}

void controlRoofServo(int angle) {
  angle = constrain(angle, 0, 180);
  roofServo.write(angle);
  currentRoofAngle = angle;
  Serial.println("[ROOF] Góc mái che: " + String(angle) + "°");
}

void controlValveServo(int angle) {
  angle = constrain(angle, 0, 180);
  valveServo.write(angle);
  currentValveAngle = angle;
  Serial.println("[VALVE] Góc van nước: " + String(angle) + "°");
}

void handleSwitches() {
  // Xử lý interrupt từ switches
  if (switch1Pressed) {
    switch1Pressed = false;
    
    // Switch 1: Toggle LED
    if (currentLEDValue > 0) {
      controlLED(0);
    } else {
      controlLED(255);
    }
    
    Serial.println("[SWITCH] Switch 1 - Toggle LED");
  }
  
  if (switch2Pressed) {
    switch2Pressed = false;
    
    // Switch 2: Toggle Roof
    if (currentRoofAngle < 90) {
      controlRoofServo(180);
    } else {
      controlRoofServo(0);
    }
    
    Serial.println("[SWITCH] Switch 2 - Toggle Roof");
  }
  
  if (switch3Pressed) {
    switch3Pressed = false;
    
    // Switch 3: Toggle Valve
    if (currentValveAngle < 90) {
      controlValveServo(180);
    } else {
      controlValveServo(0);
    }
    
    Serial.println("[SWITCH] Switch 3 - Toggle Valve");
  }
}

void sendStatusReport() {
  DynamicJsonDocument statusDoc(512);
  statusDoc["device_id"] = CLIENT_ID;
  statusDoc["status"] = "ONLINE";
  statusDoc["light_value"] = analogRead(LDR_PIN);
  statusDoc["potentiometer"] = analogRead(POTENTIOMETER_PIN);
  statusDoc["led_value"] = currentLEDValue;
  statusDoc["roof_angle"] = currentRoofAngle;
  statusDoc["valve_angle"] = currentValveAngle;
  statusDoc["auto_control"] = autoLightControl;
  statusDoc["uptime"] = millis();
  statusDoc["rssi"] = WiFi.RSSI();
  statusDoc["free_heap"] = ESP.getFreeHeap();
  
  String statusStr;
  serializeJson(statusDoc, statusStr);
  client.publish("garden/status/board1", statusStr.c_str());
}

void sendI2CSPIData() {
  // Gửi dữ liệu qua I2C/SPI (mô phỏng giao tiếp với thiết bị khác)
  DynamicJsonDocument commDoc(256);
  commDoc["board"] = CLIENT_ID;
  commDoc["type"] = "m2m_communication";
  commDoc["light"] = analogRead(LDR_PIN);
  commDoc["timestamp"] = millis();
  
  // I2C Communication - gửi dữ liệu đến thiết bị slave
  Wire.beginTransmission(8); // Địa chỉ thiết bị slave
  String jsonStr;
  serializeJson(commDoc, jsonStr);
  Wire.write(jsonStr.c_str());
  Wire.endTransmission();
  
  // SPI Communication - gửi dữ liệu qua SPI
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(jsonStr.c_str(), strlen(jsonStr.c_str()));
  digitalWrite(SPI_CS_PIN, HIGH);
  
  Serial.println("[I2C/SPI] Đã gửi dữ liệu M2M");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Smart Garden - Board 1: Light Control ===");
  Serial.println("Client ID: " + String(CLIENT_ID));
  Serial.println("Features: Light Sensor, PWM LED, Relay, Servos, Switches");
  
  // Initialize hardware
  setup_i2c_spi();
  setup_pwm();
  setup_servos();
  setup_interrupts();
  
  // Set pin modes
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(POTENTIOMETER_PIN, INPUT);
  
  // WatchDog Timer
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Kết nối WiFi và MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  Serial.println("✅ Board 1 sẵn sàng hoạt động!");
}

void loop() {
  // WatchDog reset
  esp_task_wdt_reset();
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  long now = millis();
  
  // Gửi dữ liệu cảm biến định kỳ
  if (now - lastMsg > PUBLISH_INTERVAL) {
    lastMsg = now;
    sendSensorData();
  }
  
  // Xử lý switches
  handleSwitches();
  
  // Gửi dữ liệu I2C/SPI mỗi 30 giây
  static unsigned long lastCommTime = 0;
  if (now - lastCommTime > 30000) {
    lastCommTime = now;
    sendI2CSPIData();
  }
  
  // Báo cáo trạng thái mỗi 5 phút
  static unsigned long lastStatusReport = 0;
  if (now - lastStatusReport > 300000) {
    lastStatusReport = now;
    sendStatusReport();
  }
  
  delay(100);
}
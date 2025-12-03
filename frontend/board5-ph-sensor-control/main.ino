/*
 * Smart Garden - Board 5: pH Sensor Control (ESP32)
 * Chức năng: Đo độ pH, tự động điều chỉnh bằng acid/alkaline dosing
 * Áp dụng: C++, I2C/SPI, Digital Analog, Servo, pH Control
 * Môi trường: Wokwi simulation
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>

// WiFi Configuration
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// MQTT Configuration
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* CLIENT_ID = "ESP_BOARD5_PH_01";
const char* PUBLISH_TOPIC = "garden/sensor/ph";
const char* SUBSCRIBE_TOPIC = "garden/control/ph_";

// Hardware Configuration - Board 5
#define PH_PIN_ANALOG 34
#define PH_PIN_DIGITAL 35
#define ACID_DOSING_SERVO_PIN 4
#define ALKALINE_DOSING_SERVO_PIN 5
#define ACID_LED_PIN 25
#define ALKALINE_LED_PIN 26
#define OPTIMAL_LED_PIN 27
#define STATUS_LED_PIN 2
#define CALIBRATE_BUTTON_PIN 23
#define PH_THRESHOLD_POT_PIN 32
#define AUTO_MODE_SWITCH_PIN 33
#define ALERT_BUZZER_PIN 13

// I2C/SPI Configuration
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define SPI_MOSI_PIN 23
#define SPI_MISO_PIN 19
#define SPI_SCK_PIN 18
#define SPI_CS_PIN 15

// Servo Configuration
Servo acidDosingServo;
Servo alkalineDosingServo;

// LCD Configuration
LiquidCrystal_I2C lcd(0x27, 16, 2);

// WatchDog Timer Configuration
#define WDT_TIMEOUT 30

// Global Variables
WiFiClient espClient;
PubSubClient client(espClient);
String clientId = String(CLIENT_ID) + "_" + String(random(0xffff), HEX);

// pH Control Variables
float currentPH = 7.0;
float phThreshold = 7.0;
float phCalibration = 0.0;
bool phDigitalValue = false;
bool autoMode = true;
unsigned long lastSensorRead = 0;
unsigned long lastControlAction = 0;
unsigned long lastMQTTUpdate = 0;
unsigned long lastLCDUpdate = 0;

// Control thresholds
#define PH_ACIDIC_MIN 6.0
#define PH_OPTIMAL_MIN 6.5
#define PH_OPTIMAL_MAX 7.5
#define PH_ALKALINE_MAX 8.0

// Dosing control
#define DOSING_DURATION 5000  // 5 seconds dosing
#define DOSING_COOLDOWN 30000 // 30 seconds cooldown
#define SERVO_ACID_POSITION 0      // Acid dosing position
#define SERVO_NEUTRAL_POSITION 90  // Neutral position
#define SERVO_ALKALINE_POSITION 180 // Alkaline dosing position

// Status variables
enum PHStatus {
  PH_STRONG_ACIDIC,
  PH_ACIDIC,
  PH_OPTIMAL,
  PH_ALKALINE,
  PH_STRONG_ALKALINE
};

PHStatus currentPHStatus = PH_OPTIMAL;
bool acidDosingActive = false;
bool alkalineDosingActive = false;
unsigned long dosingStartTime = 0;

// Timer Variables for replacing any potential setTimeout usage
unsigned long dosingTimer = 0;
bool dosingTimerActive = false;

// Setup WiFi
void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("WiFi kết nối: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

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
    
    // Turn on status LED when connected
    digitalWrite(STATUS_LED_PIN, HIGH);
  } else {
    Serial.println();
    Serial.println("❌ WiFi kết nối thất bại");
  }
}

// Setup I2C/SPI
void setupI2CSPI() {
  // I2C Setup
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  
  // SPI Setup
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN);
  pinMode(SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  
  Serial.println("✅ I2C/SPI initialized");
}

// Setup MQTT
void setupMQTT() {
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("MQTT kết nối...");
    
    if (client.connect(clientId.c_str())) {
      Serial.println("✅ Đã kết nối!");
      
      // Subscribe to control topics
      client.subscribe(SUBSCRIBE_TOPIC);
      client.subscribe("garden/control/all");
      client.subscribe("garden/control/ph_config");
      client.subscribe("garden/control/ph_manual");
      
      // Publish connection message
      //client.publish("garden/status/board5", "ONLINE");
      client.publish("garden/status/board5", "{\"status\":\"ONLINE\",\"device\":\"BOARD5_PH\"}");

      
      DynamicJsonDocument infoDoc(512);
      infoDoc["device_id"] = CLIENT_ID;
      infoDoc["board"] = "BOARD5_PH_CONTROL";
      infoDoc["version"] = "2.0";
      infoDoc["auto_mode"] = autoMode;
      infoDoc["ip"] = WiFi.localIP().toString();
      infoDoc["features"] = "ph_sensor,dual_servo_control,lcd_display,calibration,auto_manual_mode,led_indicators";
      
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

// MQTT Callback
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.println("[MQTT] Nhận: " + String(topic) + " = " + message);

  String topicStr = String(topic);
  
  if (topicStr == "garden/control/ph_config") {
    DynamicJsonDocument doc(512);
    deserializeJson(doc, message);
    
    if (doc.containsKey("auto_mode")) {
      autoMode = doc["auto_mode"];
      Serial.println("[CONFIG] Chế độ tự động: " + String(autoMode ? "ON" : "OFF"));
    }
    
    if (doc.containsKey("ph_threshold")) {
      phThreshold = doc["ph_threshold"];
      Serial.println("[CONFIG] Ngưỡng pH: " + String(phThreshold));
    }
  }
  
  if (topicStr == "garden/control/ph_manual") {
    if (message == "CALIBRATE") {
      calibratePH();
    } else if (message == "START_ACID") {
      startAcidDosing();
    } else if (message == "START_ALKALINE") {
      startAlkalineDosing();
    } else if (message == "STOP_ALL") {
      stopAllDosing();
    }
  }
}

// Read pH sensor
void readPHSensor() {
  // Read analog value (0-4095) and convert to pH
  int rawValue = analogRead(PH_PIN_ANALOG);
  phDigitalValue = digitalRead(PH_PIN_DIGITAL);
  
  // Convert ADC value to pH (calibrated)
  // ADC range: ~1200 (acidic) to ~2800 (alkaline), neutral ~2048
  float rawPH;
  if (rawValue <= 2048) {
    // Acidic range
    rawPH = map(rawValue, 1200, 2048, 4.0, 7.0);
  } else {
    // Alkaline range
    rawPH = map(rawValue, 2048, 2800, 7.0, 10.0);
  }
  
  // Apply calibration
  currentPH = rawPH + phCalibration;
  currentPH = constrain(currentPH, 0.0, 14.0);
  
  // Determine pH status
  updatePHStatus();
  
  // Read threshold potentiometer
  int potValue = analogRead(PH_THRESHOLD_POT_PIN);
  phThreshold = map(potValue, 0, 4095, 5.0, 9.0);
  
  // Read auto mode switch
  autoMode = digitalRead(AUTO_MODE_SWITCH_PIN);
}

// Update pH status
void updatePHStatus() {
  if (currentPH < 5.0) {
    currentPHStatus = PH_STRONG_ACIDIC;
  } else if (currentPH < PH_ACIDIC_MIN) {
    currentPHStatus = PH_ACIDIC;
  } else if (currentPH >= PH_OPTIMAL_MIN && currentPH <= PH_OPTIMAL_MAX) {
    currentPHStatus = PH_OPTIMAL;
  } else if (currentPH <= PH_ALKALINE_MAX) {
    currentPHStatus = PH_ALKALINE;
  } else {
    currentPHStatus = PH_STRONG_ALKALINE;
  }
}

// Perform pH control
void performPHControl() {
  if (!autoMode) return; // Manual mode - no automatic control
  
  unsigned long currentTime = millis();
  
  // Check if we're in cooldown period
  if (currentTime - lastControlAction < DOSING_COOLDOWN) {
    return;
  }
  
  bool needsAcid = false;
  bool needsAlkaline = false;
  
  // Determine if pH adjustment is needed
  if (currentPH < (phThreshold - 0.5)) {
    needsAlkaline = true; // pH too low, need to increase (make less acidic)
  } else if (currentPH > (phThreshold + 0.5)) {
    needsAcid = true; // pH too high, need to decrease (make less alkaline)
  }
  
  // Perform dosing if needed
  if (needsAlkaline && !alkalineDosingActive) {
    startAlkalineDosing();
  } else if (needsAcid && !acidDosingActive) {
    startAcidDosing();
  } else if (!needsAcid && !needsAlkaline) {
    stopAllDosing();
  }
}

// Start acid dosing
void startAcidDosing() {
  Serial.println("[ACID] Bắt đầu dosing acid...");
  acidDosingActive = true;
  dosingStartTime = millis();
  lastControlAction = millis();
  
  // Move servo to acid position
  acidDosingServo.write(SERVO_ACID_POSITION);
  
  // Turn on acid LED
  digitalWrite(ACID_LED_PIN, HIGH);
  
  // Turn off alkaline LED
  digitalWrite(ALKALINE_LED_PIN, LOW);
  
  // Sound alert
  digitalWrite(ALERT_BUZZER_PIN, HIGH);
  delay(500);
  digitalWrite(ALERT_BUZZER_PIN, LOW);
  
  // Use timer system instead of potential setTimeout
  dosingTimer = millis() + DOSING_DURATION;
  dosingTimerActive = true;
  
  client.publish("garden/log", "pH control: Starting acid dosing", false);
}

// Start alkaline dosing
void startAlkalineDosing() {
  Serial.println("[ALKALINE] Bắt đầu dosing alkaline...");
  alkalineDosingActive = true;
  dosingStartTime = millis();
  lastControlAction = millis();
  
  // Move servo to alkaline position
  alkalineDosingServo.write(SERVO_ALKALINE_POSITION);
  
  // Turn on alkaline LED
  digitalWrite(ALKALINE_LED_PIN, HIGH);
  
  // Turn off acid LED
  digitalWrite(ACID_LED_PIN, LOW);
  
  // Sound alert
  digitalWrite(ALERT_BUZZER_PIN, HIGH);
  delay(500);
  digitalWrite(ALERT_BUZZER_PIN, LOW);
  
  // Use timer system instead of potential setTimeout
  dosingTimer = millis() + DOSING_DURATION;
  dosingTimerActive = true;
  
  client.publish("garden/log", "pH control: Starting alkaline dosing", false);
}

// Stop all dosing
void stopAllDosing() {
  if (acidDosingActive || alkalineDosingActive) {
    Serial.println("[STOP] Dừng tất cả dosing...");
    
    // Return servos to neutral
    acidDosingServo.write(SERVO_NEUTRAL_POSITION);
    alkalineDosingServo.write(SERVO_NEUTRAL_POSITION);
    
    // Turn off LEDs
    digitalWrite(ACID_LED_PIN, LOW);
    digitalWrite(ALKALINE_LED_PIN, LOW);
    
    // Turn on optimal LED if pH is in optimal range
    if (currentPHStatus == PH_OPTIMAL) {
      digitalWrite(OPTIMAL_LED_PIN, HIGH);
    } else {
      digitalWrite(OPTIMAL_LED_PIN, LOW);
    }
    
    acidDosingActive = false;
    alkalineDosingActive = false;
    dosingTimerActive = false;
    
    client.publish("garden/log", "pH control: Stopped dosing", false);
  }
}

// Check dosing timeout (replacement for setTimeout)
void checkDosingTimeout() {
  if (dosingTimerActive && millis() >= dosingTimer) {
    stopAllDosing();
  }
}

// Perform manual dosing
void performManualDosing(String type, int duration) {
  Serial.println("Manual dosing: " + type + " for " + String(duration) + "ms");
  
  if (type.equals("acid")) {
    acidDosingServo.write(SERVO_ACID_POSITION);
    digitalWrite(ACID_LED_PIN, HIGH);
  } else if (type.equals("alkaline")) {
    alkalineDosingServo.write(SERVO_ALKALINE_POSITION);
    digitalWrite(ALKALINE_LED_PIN, HIGH);
  }
  
  delay(duration);
  
  // Stop dosing
  acidDosingServo.write(SERVO_NEUTRAL_POSITION);
  alkalineDosingServo.write(SERVO_NEUTRAL_POSITION);
  digitalWrite(ACID_LED_PIN, LOW);
  digitalWrite(ALKALINE_LED_PIN, LOW);
  
  // Fixed MQTT publish - convert String to const char*
  String logMsg = "Manual dosing completed: " + type;
  client.publish("garden/log", logMsg.c_str(), false);
}

// Calibrate pH sensor
void calibratePH() {
  Serial.println("Calibrating pH sensor...");
  
  // Take multiple readings for calibration
  float sum = 0;
  int readings = 10;
  
  for (int i = 0; i < readings; i++) {
    int rawValue = analogRead(PH_PIN_ANALOG);
    float tempPH;
    
    if (rawValue <= 2048) {
      tempPH = map(rawValue, 1200, 2048, 4.0, 7.0);
    } else {
      tempPH = map(rawValue, 2048, 2800, 7.0, 10.0);
    }
    
    sum += tempPH;
    delay(100);
  }
  
  float averagePH = sum / readings;
  
  // Assume we're calibrating with pH 7.0 buffer solution
  phCalibration = 7.0 - averagePH;
  
  Serial.println("Calibration completed. Offset: " + String(phCalibration));
  
  // Fixed MQTT publish - convert String to const char*
  String calibMsg = "pH sensor calibrated. Offset: " + String(phCalibration);
  client.publish("garden/log", calibMsg.c_str(), false);
  
  // Blink status LED to indicate calibration
  for (int i = 0; i < 5; i++) {
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(200);
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(200);
  }
}

// Update LCD display
void updateLCD() {
  if (millis() - lastLCDUpdate < 500) {
    return;
  }
  lastLCDUpdate = millis();
  
  lcd.clear();
  
  // Line 1: pH value and status
  lcd.setCursor(0, 0);
  lcd.print("pH: ");
  lcd.print(currentPH, 1);
  
  // Show status
  lcd.setCursor(8, 0);
  switch (currentPHStatus) {
    case PH_STRONG_ACIDIC: lcd.print("S-ACID"); break;
    case PH_ACIDIC: lcd.print("ACID"); break;
    case PH_OPTIMAL: lcd.print("OPTIMAL"); break;
    case PH_ALKALINE: lcd.print("ALKALINE"); break;
    case PH_STRONG_ALKALINE: lcd.print("S-ALK"); break;
  }
  
  // Line 2: Threshold and mode
  lcd.setCursor(0, 1);
  lcd.print("Thr: ");
  lcd.print(phThreshold, 1);
  
  lcd.setCursor(8, 1);
  lcd.print(autoMode ? "AUTO" : "MAN");
  
  // Show dosing status
  if (acidDosingActive) {
    lcd.setCursor(13, 1);
    lcd.print("AC");
  } else if (alkalineDosingActive) {
    lcd.setCursor(13, 1);
    lcd.print("AL");
  }
}

// Send MQTT data
void sendMQTTData() {
  DynamicJsonDocument doc(1024);
  
  doc["device_id"] = CLIENT_ID;
  doc["ph_value"] = currentPH;
  doc["ph_status"] = getPHStatusString();
  doc["digital_value"] = phDigitalValue;
  doc["analog_raw"] = analogRead(PH_PIN_ANALOG);
  doc["threshold"] = phThreshold;
  doc["auto_mode"] = autoMode;
  doc["calibration"] = phCalibration;
  doc["acid_dosing_active"] = acidDosingActive;
  doc["alkaline_dosing_active"] = alkalineDosingActive;
  doc["timestamp"] = millis();
  doc["rssi"] = WiFi.RSSI();
  doc["free_heap"] = ESP.getFreeHeap();
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  client.publish(PUBLISH_TOPIC, jsonString.c_str());
  Serial.println("[SENSOR] Gửi: " + jsonString);
}

// Get pH status string
String getPHStatusString() {
  switch (currentPHStatus) {
    case PH_STRONG_ACIDIC: return "strong_acidic";
    case PH_ACIDIC: return "acidic";
    case PH_OPTIMAL: return "optimal";
    case PH_ALKALINE: return "alkaline";
    case PH_STRONG_ALKALINE: return "strong_alkaline";
    default: return "unknown";
  }
}

// Send I2C/SPI data (M2M communication)
void sendI2CSPIData() {
  DynamicJsonDocument commDoc(256);
  commDoc["board"] = CLIENT_ID;
  commDoc["type"] = "ph_data";
  commDoc["ph_value"] = currentPH;
  commDoc["ph_status"] = getPHStatusString();
  commDoc["auto_mode"] = autoMode;
  commDoc["acid_dosing"] = acidDosingActive;
  commDoc["alkaline_dosing"] = alkalineDosingActive;
  commDoc["timestamp"] = millis();
  
  // I2C Communication
  Wire.beginTransmission(9); // Địa chỉ thiết bị nhận
  String jsonStr;
  serializeJson(commDoc, jsonStr);
  
  // Chuyển String thành uint8_t array
  uint8_t data[jsonStr.length() + 1];
  jsonStr.getBytes(data, sizeof(data));
  Wire.write(data, jsonStr.length());
  Wire.endTransmission();
  
  // SPI Communication
  digitalWrite(SPI_CS_PIN, LOW);
  uint8_t spiData[jsonStr.length() + 1];
  jsonStr.getBytes(spiData, sizeof(spiData));
  SPI.transfer(spiData, jsonStr.length());
  digitalWrite(SPI_CS_PIN, HIGH);
  
  Serial.println("[I2C/SPI] Đã gửi dữ liệu pH");
}

// Send status report
void sendStatusReport() {
  DynamicJsonDocument statusDoc(512);
  statusDoc["device_id"] = CLIENT_ID;
  statusDoc["status"] = "ONLINE";
  statusDoc["ph_value"] = currentPH;
  statusDoc["ph_status"] = getPHStatusString();
  statusDoc["auto_mode"] = autoMode;
  statusDoc["acid_dosing"] = acidDosingActive;
  statusDoc["alkaline_dosing"] = alkalineDosingActive;
  statusDoc["threshold"] = phThreshold;
  statusDoc["calibration"] = phCalibration;
  statusDoc["uptime"] = millis();
  statusDoc["rssi"] = WiFi.RSSI();
  statusDoc["free_heap"] = ESP.getFreeHeap();
  
  String statusStr;
  serializeJson(statusDoc, statusStr);
  //client.publish("garden/status/board5", statusStr.c_str());
  client.publish("garden/status/board5", "{\"status\":\"ONLINE\",\"device\":\"BOARD5_PH\"}");

}

// Setup function
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Smart Garden - Board 5: pH Sensor Control ===");
  Serial.println("Client ID: " + String(CLIENT_ID));
  Serial.println("Features: pH Sensor, Dual Servo Control, Auto/Manual Mode, LCD Display, Calibration");
  
  // Initialize hardware
  setupI2CSPI();
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart Garden Board5");
  lcd.setCursor(0, 1);
  lcd.print("pH Control System");
  delay(2000);
  
  // Initialize servos
  acidDosingServo.attach(ACID_DOSING_SERVO_PIN);
  alkalineDosingServo.attach(ALKALINE_DOSING_SERVO_PIN);
  acidDosingServo.write(SERVO_NEUTRAL_POSITION);
  alkalineDosingServo.write(SERVO_NEUTRAL_POSITION);
  
  // Initialize hardware pins
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ACID_LED_PIN, OUTPUT);
  pinMode(ALKALINE_LED_PIN, OUTPUT);
  pinMode(OPTIMAL_LED_PIN, OUTPUT);
  pinMode(CALIBRATE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(AUTO_MODE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(ALERT_BUZZER_PIN, OUTPUT);
  
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(ACID_LED_PIN, LOW);
  digitalWrite(ALKALINE_LED_PIN, LOW);
  digitalWrite(OPTIMAL_LED_PIN, LOW);
  
  // WatchDog Timer - sửa lỗi ESP-IDF v5.x
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  
  // Setup WiFi and MQTT
  setupWiFi();
  setupMQTT();
  
  Serial.println("✅ Board 5 sẵn sàng hoạt động!");
  
  // Initial readings
  readPHSensor();
  updateLCD();
}

// Main loop
void loop() {
  // WatchDog reset
  esp_task_wdt_reset();
  
  // Reconnect MQTT if needed
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();
  
  unsigned long currentTime = millis();
  
  // Read sensor every 1 second
  if (currentTime - lastSensorRead > 1000) {
    readPHSensor();
    lastSensorRead = currentTime;
  }
  
  // Perform pH control
  performPHControl();
  
  // Check dosing timeout
  checkDosingTimeout();
  
  // Update LCD
  updateLCD();
  
  // Send MQTT data every 5 seconds
  if (currentTime - lastMQTTUpdate > 5000) {
    lastMQTTUpdate = currentTime;
    sendMQTTData();
  }
  
  // Handle calibration button
  if (digitalRead(CALIBRATE_BUTTON_PIN) == LOW) {
    delay(50); // Debounce
    if (digitalRead(CALIBRATE_BUTTON_PIN) == LOW) {
      calibratePH();
    }
  }
  
  // Send I2C/SPI data every 45 seconds
  static unsigned long lastCommTime = 0;
  if (currentTime - lastCommTime > 45000) {
    lastCommTime = currentTime;
    sendI2CSPIData();
  }
  
  // Send status report every 5 minutes
  static unsigned long lastStatusReport = 0;
  if (currentTime - lastStatusReport > 300000) {
    lastStatusReport = currentTime;
    sendStatusReport();
  }
  
  delay(100);
}
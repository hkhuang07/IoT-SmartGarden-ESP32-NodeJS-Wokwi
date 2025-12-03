/*
 * Smart Garden - Board 4: NPK Nutrient Sensor + LCD + Valve Control (ESP32)
 * Chức năng: Đo hàm lượng dinh dưỡng NPK, hiển thị LCD, điều khiển van
 * Áp dụng: Digital Analog, LCD, Servo Valve
 * Môi trường: Wokwi simulation
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <esp_task_wdt.h>

// ==================== WIFI CONFIGURATION ====================
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// ==================== MQTT CONFIGURATION ====================
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* CLIENT_ID = "ESP_BOARD4_NPK_01";

// MQTT Topics
const char* PUBLISH_TOPIC = "garden/sensor/npk";
const char* SUBSCRIBE_TOPIC = "garden/control/npk_";
const char* CONFIG_TOPIC = "garden/control/npk_config";
const char* VALVE_CONTROL_TOPIC = "garden/control/water_valve";
const char* CALIBRATION_TOPIC = "garden/control/npk_calibration";
const char* STATUS_TOPIC = "garden/status/board4";

// ==================== HARDWARE CONFIGURATION ====================
// Board 4 - NPK Sensor + LCD + Servo Valve
#define N_PIN 34     // Nitrogen analog input
#define P_PIN 35     // Phosphorus analog input
#define K_PIN 32     // Potassium analog input
#define VALVE_SERVO_PIN 4    // Servo valve control
#define BUTTON_PIN 25        // Manual control button
#define LED_PIN 2            // Status LED
#define POTENTIOMETER_PIN 33 // Calibration adjustment

// ==================== LCD CONFIGURATION ====================
LiquidCrystal_I2C lcd(0x27, 20, 4); // 20x4 LCD with I2C

// ==================== SERVO CONFIGURATION ====================
Servo valveServo;
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define VALVE_OPEN_ANGLE 90
#define VALVE_CLOSED_ANGLE 0

// ==================== WATCHDOG TIMER CONFIGURATION ====================
#define WDT_TIMEOUT 30  // 30 seconds timeout

// ==================== ADC CONFIGURATION ====================
const int ADC_RESOLUTION = 4096;
const float ADC_VOLTAGE = 3.3;

// ==================== NPK CALIBRATION VALUES ====================
float n_calibration = 0.015;
float p_calibration = 0.012;
float k_calibration = 0.016;

// ==================== NPK THRESHOLDS ====================
const float N_MIN = 10.0, N_MAX = 50.0;
const float P_MIN = 5.0, P_MAX = 30.0;
const float K_MIN = 8.0, K_MAX = 40.0;

// ==================== TIMING CONFIGURATION ====================
#define SENSOR_READ_INTERVAL 2500  // 25 seconds
#define STATUS_REPORT_INTERVAL 30000  // 5 minutes

// ==================== INTERRUPT VARIABLES ====================
volatile bool buttonPressed = false;
volatile bool calibrationMode = false;

// ==================== MQTT CLIENT ====================
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
unsigned long lastStatusReport = 0;
unsigned long valveCloseTimer = 0;
bool valveTimerActive = false;

// ==================== CONTROL MODES ====================
bool autoFertilization = true;

// ==================== GLOBAL VARIABLES ====================
float nitrogenValue = 0.0;
float phosphorusValue = 0.0;
float potassiumValue = 0.0;
int currentValveAngle = 0;
int lcdUpdateCounter = 0;
String lastAction = "INIT";

// ==================== INTERRUPT SERVICE ROUTINES ====================
void IRAM_ATTR buttonISR() {
  buttonPressed = true;
}

// ==================== SETUP FUNCTIONS ====================

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

void setup_lcd() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart Garden v2.0");
  lcd.setCursor(0, 1);
  lcd.print("Board 4: NPK Sensor");
  lcd.setCursor(0, 2);
  lcd.print("NPK + LCD + Valve");
  delay(2000);
  
  Serial.println("✅ LCD initialized - 20x4 I2C");
}

void setup_servo() {
  valveServo.attach(VALVE_SERVO_PIN);
  controlValveServo(VALVE_CLOSED_ANGLE);
  
  Serial.println("✅ Servo valve initialized - GPIO" + String(VALVE_SERVO_PIN));
}

void setup_adc() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  Serial.println("✅ ADC initialized - 12-bit resolution");
}

void setup_interrupts() {
  // Button with INPUT_PULLUP - Logic đảo (nhấn = LOW)
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
  
  Serial.println("✅ Interrupts initialized - Button: GPIO" + String(BUTTON_PIN));
}

// ==================== MQTT CALLBACK ====================

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  String topicStr = String(topic);
  Serial.println("[MQTT] Nhận: " + topicStr + " = " + message);
  
  // Configuration commands
  if (topicStr == CONFIG_TOPIC) {
    DynamicJsonDocument doc(512);
    deserializeJson(doc, message);
    
    if (doc.containsKey("auto_fertilization")) {
      autoFertilization = doc["auto_fertilization"];
      lastAction = "AUTO_MODE_" + String(autoFertilization ? "ON" : "OFF");
      Serial.println("[CONFIG] Tưới phân tự động: " + String(autoFertilization));
    }
    
    if (doc.containsKey("n_calibration")) {
      n_calibration = doc["n_calibration"];
      Serial.println("[CONFIG] Nitrogen calibration: " + String(n_calibration));
      lastAction = "N_CALIBRATION_UPDATED";
    }
    
    if (doc.containsKey("p_calibration")) {
      p_calibration = doc["p_calibration"];
      Serial.println("[CONFIG] Phosphorus calibration: " + String(p_calibration));
      lastAction = "P_CALIBRATION_UPDATED";
    }
    
    if (doc.containsKey("k_calibration")) {
      k_calibration = doc["k_calibration"];
      Serial.println("[CONFIG] Potassium calibration: " + String(k_calibration));
      lastAction = "K_CALIBRATION_UPDATED";
    }
  }
  
  // Valve control commands
  else if (topicStr == VALVE_CONTROL_TOPIC) {
    if (message == "OPEN") {
      controlValveServo(VALVE_OPEN_ANGLE);
      client.publish("garden/status/npk_valve", "OPEN");
      lastAction = "VALVE_OPEN_MANUAL";
    } else if (message == "CLOSE") {
      controlValveServo(VALVE_CLOSED_ANGLE);
      client.publish("garden/status/npk_valve", "CLOSED");
      lastAction = "VALVE_CLOSE_MANUAL";
    } else if (message.toInt() > 0 && message.toInt() <= 180) {
      controlValveServo(message.toInt());
      lastAction = "VALVE_POSITION_" + message;
    }
  }
  
  // Calibration commands
  else if (topicStr == CALIBRATION_TOPIC) {
    handleCalibrationCommand(message);
  }
}

void handleCalibrationCommand(String command) {
  if (command == "CALIBRATE_HIGH") {
    calibrateNPK("HIGH");
  } else if (command == "CALIBRATE_MEDIUM") {
    calibrateNPK("MEDIUM");
  } else if (command == "CALIBRATE_LOW") {
    calibrateNPK("LOW");
  } else if (command == "RESET_CALIBRATION") {
    resetCalibration();
  }
}

void calibrateNPK(String level) {
  int n_raw = analogRead(N_PIN);
  int p_raw = analogRead(P_PIN);
  int k_raw = analogRead(K_PIN);
  
  Serial.println("[CALIBRATION] " + level + " - N:" + String(n_raw) + 
                 " P:" + String(p_raw) + " K:" + String(k_raw));
  
  // Store calibration values
  // In real implementation, would save to EEPROM
  
  lcd.setCursor(0, 3);
  lcd.print("Calibrated: " + level);
  
  lastAction = "CALIBRATION_" + level;
  delay(2000);
}

void resetCalibration() {
  Serial.println("[CALIBRATION] Reset hiệu chỉnh");
  
  lcd.setCursor(0, 3);
  lcd.print("Calibration Reset");
  delay(2000);
  
  lastAction = "CALIBRATION_RESET";
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT kết nối...");
    
    if (client.connect(CLIENT_ID)) {
      Serial.println("✅ MQTT kết nối thành công!");
      
      // Subscribe to control topics
      client.subscribe(SUBSCRIBE_TOPIC);
      client.subscribe(CONFIG_TOPIC);
      client.subscribe(VALVE_CONTROL_TOPIC);
      client.subscribe(CALIBRATION_TOPIC);
      
      // Send online status
      client.publish("garden/status/board4", "{\"status\":\"ONLINE\",\"device\":\"BOARD4_NPK\"}");
      
      // Send device info
      sendDeviceInfo();
      
    } else {
      Serial.print("Thất bại, rc=");
      Serial.print(client.state());
      Serial.println(" Thử lại sau 3 giây");
      delay(3000);
    }
  }
}

void sendDeviceInfo() {
  DynamicJsonDocument infoDoc(512);
  infoDoc["device_id"] = CLIENT_ID;
  infoDoc["board"] = "BOARD4_NPK_CONTROL";
  infoDoc["version"] = "2.0";
  infoDoc["auto_fertilization"] = autoFertilization;
  infoDoc["ip"] = WiFi.localIP().toString();
  infoDoc["features"] = "npk_sensor,lcd,servo_valve,calibration,auto_fertilization,digital_analog";
  
  JsonArray sensors = infoDoc.createNestedArray("sensors");
  sensors.add("NPK_triple_sensor");
  sensors.add("potentiometer_calibration");
  
  JsonArray actuators = infoDoc.createNestedArray("actuators");
  actuators.add("servo_valve");
  actuators.add("lcd_display");
  actuators.add("status_led");
  
  JsonArray interfaces = infoDoc.createNestedArray("interfaces");
  interfaces.add("analog_digital");
  interfaces.add("i2c_lcd");
  interfaces.add("pwm_servo");
  
  String infoStr;
  serializeJson(infoDoc, infoStr);
  client.publish("garden/system/device_info", infoStr.c_str());
  Serial.println("[INFO] Gửi thông tin thiết bị");
}

// ==================== SENSOR READING & AUTO CONTROL ====================

void sendSensorData() {
  // Read raw values from NPK sensors
  int n_raw = analogRead(N_PIN);
  int p_raw = analogRead(P_PIN);
  int k_raw = analogRead(K_PIN);
  
  // Convert to ppm (parts per million)
  nitrogenValue = (n_raw * ADC_VOLTAGE / ADC_RESOLUTION) / n_calibration;
  phosphorusValue = (p_raw * ADC_VOLTAGE / ADC_RESOLUTION) / p_calibration;
  potassiumValue = (k_raw * ADC_VOLTAGE / ADC_RESOLUTION) / k_calibration;
  
  // Read potentiometer value
  int potValue = analogRead(POTENTIOMETER_PIN);
  
  // Create JSON data for MongoDB
  DynamicJsonDocument doc(1024);
  doc["device_id"] = CLIENT_ID;
  doc["timestamp"] = millis();
  
  // Raw sensor data
  JsonObject raw_data = doc.createNestedObject("raw_data");
  raw_data["n_raw"] = n_raw;
  raw_data["p_raw"] = p_raw;
  raw_data["k_raw"] = k_raw;
  
  // Processed values (ppm)
  JsonObject npk_values = doc.createNestedObject("npk_values");
  npk_values["nitrogen_ppm"] = nitrogenValue;
  npk_values["phosphorus_ppm"] = phosphorusValue;
  npk_values["potassium_ppm"] = potassiumValue;
  
  // System status
  JsonObject system = doc.createNestedObject("system");
  system["auto_fertilization"] = autoFertilization;
  system["valve_angle"] = currentValveAngle;
  system["potentiometer"] = potValue;
  system["last_action"] = lastAction;
  system["wifi_rssi"] = WiFi.RSSI();
  system["free_heap"] = ESP.getFreeHeap();
  
  // Status assessment
  JsonObject nutrient_status = doc.createNestedObject("nutrient_status");
  nutrient_status["nitrogen"] = getNutrientStatus(nitrogenValue, N_MIN, N_MAX);
  nutrient_status["phosphorus"] = getNutrientStatus(phosphorusValue, P_MIN, P_MAX);
  nutrient_status["potassium"] = getNutrientStatus(potassiumValue, K_MIN, K_MAX);
  
  String payload;
  serializeJson(doc, payload);
  
  // Publish to MQTT
  client.publish(PUBLISH_TOPIC, payload.c_str());
  Serial.println("[SENSOR] NPK: N=" + String(nitrogenValue, 1) + "ppm, " +
                 "P=" + String(phosphorusValue, 1) + "ppm, " +
                 "K=" + String(potassiumValue, 1) + "ppm");
  
  // Auto fertilization control
  if (autoFertilization) {
    handleAutoFertilization();
  }
  
  // Update LCD display
  updateLCD();
}

String getNutrientStatus(float value, float min_val, float max_val) {
  if (value < min_val) return "LOW";
  if (value > max_val) return "HIGH";
  return "NORMAL";
}

// Logic điều khiển tưới phân tự động - Theo yêu cầu phân tích
void handleAutoFertilization() {
  bool needsFertilization = false;
  String deficiency = "";
  
  // Check Nitrogen deficiency
  if (nitrogenValue < N_MIN) {
    needsFertilization = true;
    deficiency += "N ";
  }
  
  // Check Phosphorus deficiency
  if (phosphorusValue < P_MIN) {
    needsFertilization = true;
    deficiency += "P ";
  }
  
  // Check Potassium deficiency
  if (potassiumValue < K_MIN) {
    needsFertilization = true;
    deficiency += "K ";
  }
  
  if (needsFertilization) {
    Serial.println("[AUTO] Thiếu dinh dưỡng: " + deficiency + "- Bật tưới phân");
    
    // Open valve for fertilization
    controlValveServo(VALVE_OPEN_ANGLE);
    
    // Send alert
    client.publish("garden/alerts/npk_deficiency", deficiency.c_str());
    
    // Auto close after 30 seconds (using timer instead of setTimeout)
    valveCloseTimer = millis() + 30000;  // 30 seconds
    valveTimerActive = true;
    
    lastAction = "AUTO_FERTILIZATION_" + deficiency;
  }
}

void controlValveServo(int angle) {
  currentValveAngle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  valveServo.write(currentValveAngle);
  Serial.println("[VALVE] Van nước: " + String(currentValveAngle) + "° (" + 
                 (currentValveAngle > 45 ? "Mở" : "Đóng") + ")");
}

void updateLCD() {
  lcdUpdateCounter++;
  if (lcdUpdateCounter % 5 == 0) { // Update every 5 cycles
    lcd.clear();
    
    // Row 1: Title
    lcd.setCursor(0, 0);
    lcd.print("Smart Garden v2.0");
    
    // Row 2: Nitrogen and Phosphorus
    lcd.setCursor(0, 1);
    lcd.print("N:");
    lcd.print(nitrogenValue, 0);
    lcd.print("ppm P:");
    lcd.print(phosphorusValue, 0);
    lcd.print("ppm");
    
    // Row 3: Potassium and Status
    lcd.setCursor(0, 2);
    lcd.print("K:");
    lcd.print(potassiumValue, 0);
    lcd.print("ppm");
    
    // Row 4: Valve status and nutrient level
    lcd.setCursor(0, 3);
    lcd.print("Valve:");
    lcd.print(currentValveAngle > 0 ? "ON " : "OFF");
    
    // Show nutrient level
    String overall_status = "NORMAL";
    if (nitrogenValue < N_MIN || phosphorusValue < P_MIN || potassiumValue < K_MIN) {
      overall_status = "LOW";
    } else if (nitrogenValue > N_MAX || phosphorusValue > P_MAX || potassiumValue > K_MAX) {
      overall_status = "HIGH";
    }
    lcd.print(" " + overall_status);
  }
}

void handleButton() {
  if (buttonPressed) {
    buttonPressed = false;
    
    // Toggle auto fertilization mode
    autoFertilization = !autoFertilization;
    
    Serial.println("[BUTTON] Chế độ tưới phân tự động: " + String(autoFertilization ? "ON" : "OFF"));
    
    // Flash LED indicator
    digitalWrite(LED_PIN, autoFertilization ? HIGH : LOW);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    
    // Show notification on LCD
    lcd.setCursor(0, 3);
    lcd.print("Auto Mode: " + String(autoFertilization ? "ON " : "OFF"));
    delay(1000);
    
    lastAction = "BUTTON_TOGGLE_AUTO_" + String(autoFertilization ? "ON" : "OFF");
  }
}

void sendStatusReport() {
  DynamicJsonDocument statusDoc(512);
  statusDoc["device_id"] = CLIENT_ID;
  statusDoc["status"] = "ONLINE";
  statusDoc["uptime_seconds"] = millis() / 1000;
  
  // Current nutrient levels
  statusDoc["nitrogen"] = nitrogenValue;
  statusDoc["phosphorus"] = phosphorusValue;
  statusDoc["potassium"] = potassiumValue;
  
  // System status
  statusDoc["auto_fertilization"] = autoFertilization;
  statusDoc["valve_angle"] = currentValveAngle;
  statusDoc["last_action"] = lastAction;
  statusDoc["wifi_rssi"] = WiFi.RSSI();
  statusDoc["free_heap"] = ESP.getFreeHeap();
  
  // Fix ArduinoJson serialization - correct parameter order
  String statusStr;
  serializeJson(statusDoc, statusStr);
  client.publish(STATUS_TOPIC, statusStr.c_str());
  Serial.println("[STATUS] Gửi báo cáo trạng thái");
}

// ==================== SETUP ====================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Smart Garden - Board 4: NPK Nutrient Control System ===");
  Serial.println("🔬 Chức năng: Cảm biến NPK + LCD + Servo Valve + Auto Fertilization");
  Serial.println("🔧 Công nghệ: Digital Analog, I2C LCD, PWM Servo");
  Serial.println("🌐 Môi trường: Wokwi simulation");
  Serial.println();
  
  // Initialize hardware subsystems
  setup_lcd();
  setup_servo();
  setup_adc();
  setup_interrupts();
  
  // Set pin modes
  pinMode(LED_PIN, OUTPUT);
  pinMode(N_PIN, INPUT);
  pinMode(P_PIN, INPUT);
  pinMode(K_PIN, INPUT);
  pinMode(POTENTIOMETER_PIN, INPUT);
  
  // Initialize LED
  digitalWrite(LED_PIN, LOW);
  
  // WatchDog Timer Setup - Fixed for ESP32 SDK
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  Serial.println("✅ WatchDog Timer initialized (" + String(WDT_TIMEOUT) + "s)");
  
  Serial.println("✅ Hardware initialization complete");
  Serial.println();
  
  // Network connections
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  Serial.println();
  Serial.println("🌱 Smart Garden Board 4 NPK Control Ready!");
  Serial.println("📡 MQTT: " + String(mqtt_server) + ":" + String(mqtt_port));
  Serial.println("🏠 Client ID: " + String(CLIENT_ID));
  Serial.println("📊 Publish: " + String(PUBLISH_TOPIC));
  Serial.println("🎛️ Auto Fertilization: " + String(autoFertilization ? "ON" : "OFF"));
}

// ==================== MAIN LOOP ====================

void loop() {
  // WatchDog reset
  esp_task_wdt_reset();
  
  // Handle valve timer (replacement for setTimeout)
  if (valveTimerActive && millis() >= valveCloseTimer) {
    controlValveServo(VALVE_CLOSED_ANGLE);
    valveTimerActive = false;
    lastAction = "VALVE_AUTO_CLOSE";
    Serial.println("[AUTO] Tắt tưới phân tự động");
  }
  
  // MQTT connection management
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  // Timing variables
  long now = millis();
  
  // Send sensor data periodically (25 seconds)
  if (now - lastMsg > SENSOR_READ_INTERVAL) {
    lastMsg = now;
    sendSensorData();
  } 
  
  // Handle manual button
  handleButton();
  
  // Send status report every 5 minutes
  if (now - lastStatusReport > STATUS_REPORT_INTERVAL) {
    lastStatusReport = now;
    sendStatusReport();
  }
  
  // Small delay for system stability
  delay(100);
}
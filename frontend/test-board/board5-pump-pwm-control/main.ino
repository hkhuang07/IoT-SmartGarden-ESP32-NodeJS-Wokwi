/*
 * Smart Garden - Board 5: Water Pump Control with PWM & WatchDog (ESP32)
 * Chức năng: Điều khiển bơm nước với PWM, WatchDog timer, Multiple relays
 * Áp dụng: PWM, WatchDog, Multiple I/O, Digital Control
 * Môi trường: Wokwi simulation
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi Configuration
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// MQTT Configuration
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* CLIENT_ID = "ESP_BOARD5_PUMP_01";
const char* PUBLISH_TOPIC = "garden/sensor/pump_status";
const char* SUBSCRIBE_TOPIC = "garden/control/pump_";

// Hardware Configuration - Board 5
#define PUMP_RELAY_PIN 4
#define MAIN_VALVE_RELAY_PIN 5
#define EMERGENCY_VALVE_RELAY_PIN 18
#define FLOW_SENSOR_PIN 19
#define PRESSURE_SENSOR_PIN 34
#define BUTTON_START_PIN 25
#define BUTTON_STOP_PIN 26
#define BUTTON_EMERGENCY_PIN 27
#define LED_STATUS_PIN 2
#define LED_ALARM_PIN 12

// PWM Configuration
#define PWM_CHANNEL_PUMP 0
#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION 8

// WatchDog Timer Configuration
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 15  // 15 seconds timeout

// Global Variables
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;

// Pump Control Variables
bool pumpActive = false;
bool mainValveOpen = false;
bool emergencyMode = false;
int pumpSpeed = 0;
float flowRate = 0.0;
float pressureValue = 0.0;
unsigned long pumpStartTime = 0;
unsigned long lastMaintenanceTime = 0;

// Flow Sensor Calibration
const float FLOW_CALIBRATION = 0.5; // pulses per second to L/min

// Pressure Sensor Calibration
const float PRESSURE_OFFSET = 0.5;
const float PRESSURE_SCALE = 0.1;

// Maintenance Settings
const unsigned long MAINTENANCE_INTERVAL = 86400000; // 24 hours
const unsigned long MAX_CONTINUOUS_PUMPING = 1800000; // 30 minutes
const unsigned long MIN_PUMP_COOLDOWN = 300000; // 5 minutes

// State Variables
volatile bool buttonStartPressed = false;
volatile bool buttonStopPressed = false;
volatile bool buttonEmergencyPressed = false;
volatile unsigned long lastFlowPulse = 0;
volatile unsigned long flowPulseCount = 0;

// ISR Functions
void IRAM_ATTR buttonStartISR() {
  buttonStartPressed = true;
}

void IRAM_ATTR buttonStopISR() {
  buttonStopPressed = true;
}

void IRAM_ATTR buttonEmergencyISR() {
  buttonEmergencyPressed = true;
}

void IRAM_ATTR flowSensorISR() {
  flowPulseCount++;
  lastFlowPulse = millis();
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

void setup_pwm() {
  ledcSetup(PWM_CHANNEL_PUMP, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PUMP_RELAY_PIN, PWM_CHANNEL_PUMP);
  ledcWrite(PWM_CHANNEL_PUMP, 0);
  
  Serial.println("✅ PWM initialized");
}

void setup_interrupts() {
  pinMode(BUTTON_START_PIN, INPUT_PULLUP);
  pinMode(BUTTON_STOP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_EMERGENCY_PIN, INPUT_PULLUP);
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(BUTTON_START_PIN), buttonStartISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_STOP_PIN), buttonStopISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_EMERGENCY_PIN), buttonEmergencyISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowSensorISR, RISING);
  
  Serial.println("✅ Interrupts initialized");
}

void setup_hardware() {
  pinMode(MAIN_VALVE_RELAY_PIN, OUTPUT);
  pinMode(EMERGENCY_VALVE_RELAY_PIN, OUTPUT);
  pinMode(PRESSURE_SENSOR_PIN, INPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(LED_ALARM_PIN, OUTPUT);
  
  // Initial state
  digitalWrite(MAIN_VALVE_RELAY_PIN, LOW);
  digitalWrite(EMERGENCY_VALVE_RELAY_PIN, LOW);
  digitalWrite(LED_STATUS_PIN, LOW);
  digitalWrite(LED_ALARM_PIN, LOW);
  
  Serial.println("✅ Hardware initialized");
}

void setup_watchdog() {
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  Serial.println("✅ WatchDog Timer initialized");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println("[MQTT] Nhận: " + String(topic) + " = " + message);
  
  String topicStr = String(topic);
  
  if (topicStr == "garden/control/pump_start") {
    if (message == "ON" || message == "START") {
      startPump();
    }
  }
  
  if (topicStr == "garden/control/pump_stop") {
    if (message == "OFF" || message == "STOP") {
      stopPump();
    }
  }
  
  if (topicStr == "garden/control/pump_speed") {
    int speed = message.toInt();
    if (speed >= 0 && speed <= 255) {
      setPumpSpeed(speed);
    }
  }
  
  if (topicStr == "garden/control/pump_emergency") {
    if (message == "EMERGENCY_STOP") {
      emergencyStop();
    }
  }
  
  if (topicStr == "garden/control/pump_config") {
    DynamicJsonDocument doc(512);
    deserializeJson(doc, message);
    
    // Có thể cập nhật các tham số cấu hình
    Serial.println("[CONFIG] Pump configuration updated");
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT kết nối...");
    
    if (client.connect(CLIENT_ID)) {
      Serial.println("✅ Đã kết nối!");
      
      client.subscribe(SUBSCRIBE_TOPIC);
      client.subscribe("garden/control/pump_start");
      client.subscribe("garden/control/pump_stop");
      client.subscribe("garden/control/pump_speed");
      client.subscribe("garden/control/pump_emergency");
      client.subscribe("garden/control/pump_config");
      
      client.publish("garden/status/board5", "ONLINE");
      
      DynamicJsonDocument infoDoc(512);
      infoDoc["device_id"] = CLIENT_ID;
      infoDoc["board"] = "BOARD5_PUMP_CONTROL";
      infoDoc["version"] = "2.0";
      infoDoc["pump_active"] = pumpActive;
      infoDoc["ip"] = WiFi.localIP().toString();
      infoDoc["features"] = "pwm_control,watchdog,flow_sensor,pressure_sensor,emergency_stop";
      
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

void startPump() {
  if (emergencyMode) {
    Serial.println("[PUMP] Không thể khởi động: Chế độ khẩn cấp");
    return;
  }
  
  if (pumpActive) {
    Serial.println("[PUMP] Bơm đang hoạt động");
    return;
  }
  
  // Kiểm tra cooldown
  if (millis() - pumpStartTime < MIN_PUMP_COOLDOWN && pumpStartTime > 0) {
    Serial.println("[PUMP] Đang trong thời gian nghỉ");
    return;
  }
  
  pumpActive = true;
  pumpStartTime = millis();
  setPumpSpeed(200); // Tốc độ mặc định
  
  digitalWrite(MAIN_VALVE_RELAY_PIN, HIGH);
  mainValveOpen = true;
  
  digitalWrite(LED_STATUS_PIN, HIGH);
  
  Serial.println("[PUMP] Khởi động bơm với tốc độ: " + String(pumpSpeed));
  
  client.publish("garden/status/pump", "ON");
}

void stopPump() {
  if (!pumpActive) {
    Serial.println("[PUMP] Bơm đã dừng");
    return;
  }
  
  pumpActive = false;
  setPumpSpeed(0);
  
  digitalWrite(MAIN_VALVE_RELAY_PIN, LOW);
  mainValveOpen = false;
  
  digitalWrite(LED_STATUS_PIN, LOW);
  
  Serial.println("[PUMP] Dừng bơm");
  
  client.publish("garden/status/pump", "OFF");
}

void emergencyStop() {
  emergencyMode = true;
  pumpActive = false;
  
  setPumpSpeed(0);
  
  digitalWrite(MAIN_VALVE_RELAY_PIN, LOW);
  digitalWrite(EMERGENCY_VALVE_RELAY_PIN, HIGH);
  mainValveOpen = false;
  
  digitalWrite(LED_STATUS_PIN, LOW);
  digitalWrite(LED_ALARM_PIN, HIGH);
  
  Serial.println("[EMERGENCY] Dừng khẩn cấp hệ thống bơm!");
  
  client.publish("garden/alerts/pump_emergency", "EMERGENCY_STOP");
  
  // Reset emergency mode sau 10 giây
  setTimeout([]() {
    emergencyMode = false;
    digitalWrite(EMERGENCY_VALVE_RELAY_PIN, LOW);
    digitalWrite(LED_ALARM_PIN, LOW);
    Serial.println("[EMERGENCY] Reset chế độ khẩn cấp");
  }, 10000);
}

void setPumpSpeed(int speed) {
  pumpSpeed = constrain(speed, 0, 255);
  
  if (pumpActive) {
    ledcWrite(PWM_CHANNEL_PUMP, pumpSpeed);
    Serial.println("[PUMP] Tốc độ: " + String(pumpSpeed) + "/255");
  }
}

void checkFlowSensor() {
  // Tính toán lưu lượng nước
  unsigned long currentTime = millis();
  unsigned long timeDiff = currentTime - lastFlowPulse;
  
  if (timeDiff > 1000) { // Cập nhật mỗi giây
    float pulsesPerSecond = (float)flowPulseCount / (timeDiff / 1000.0);
    flowRate = pulsesPerSecond * FLOW_CALIBRATION;
    flowPulseCount = 0;
    lastFlowPulse = currentTime;
  }
}

void readPressureSensor() {
  int pressureRaw = analogRead(PRESSURE_SENSOR_PIN);
  float voltage = (pressureRaw * ADC_VOLTAGE) / 4095.0;
  pressureValue = (voltage - PRESSURE_OFFSET) / PRESSURE_SCALE;
  pressureValue = max(0.0, pressureValue); // Đảm bảo không âm
}

void checkMaintenance() {
  unsigned long currentTime = millis();
  
  // Kiểm tra bảo trì định kỳ
  if (currentTime - lastMaintenanceTime > MAINTENANCE_INTERVAL) {
    Serial.println("[MAINTENANCE] Thời gian bảo trì định kỳ");
    
    client.publish("garden/alerts/maintenance", "SCHEDULED_MAINTENANCE");
    
    lastMaintenanceTime = currentTime;
  }
  
  // Kiểm tra thời gian bơm liên tục tối đa
  if (pumpActive && (currentTime - pumpStartTime > MAX_CONTINUOUS_PUMPING)) {
    Serial.println("[PUMP] Tự động dừng do chạy quá lâu");
    stopPump();
    
    client.publish("garden/alerts/pump_timeout", "MAX_CONTINUOUS_TIME");
  }
}

void handleButtons() {
  if (buttonStartPressed) {
    buttonStartPressed = false;
    startPump();
  }
  
  if (buttonStopPressed) {
    buttonStopPressed = false;
    stopPump();
  }
  
  if (buttonEmergencyPressed) {
    buttonEmergencyPressed = false;
    emergencyStop();
  }
}

void sendSensorData() {
  checkFlowSensor();
  readPressureSensor();
  
  DynamicJsonDocument doc(512);
  doc["pump_active"] = pumpActive;
  doc["pump_speed"] = pumpSpeed;
  doc["flow_rate"] = flowRate;
  doc["pressure"] = pressureValue;
  doc["main_valve"] = mainValveOpen;
  doc["emergency_mode"] = emergencyMode;
  doc["uptime"] = millis() - pumpStartTime;
  doc["device_id"] = CLIENT_ID;
  doc["timestamp"] = millis();
  doc["rssi"] = WiFi.RSSI();
  doc["free_heap"] = ESP.getFreeHeap();
  
  String payload;
  serializeJson(doc, payload);
  
  client.publish(PUBLISH_TOPIC, payload.c_str());
  Serial.println("[SENSOR] Gửi: " + payload);
}

void sendStatusReport() {
  DynamicJsonDocument statusDoc(512);
  statusDoc["device_id"] = CLIENT_ID;
  statusDoc["status"] = "ONLINE";
  statusDoc["pump_active"] = pumpActive;
  statusDoc["pump_speed"] = pumpSpeed;
  statusDoc["flow_rate"] = flowRate;
  statusDoc["pressure"] = pressureValue;
  statusDoc["maintenance_due"] = (millis() - lastMaintenanceTime > MAINTENANCE_INTERVAL * 0.9);
  statusDoc["uptime"] = millis();
  statusDoc["rssi"] = WiFi.RSSI();
  statusDoc["free_heap"] = ESP.getFreeHeap();
  
  String statusStr;
  serializeJson(statusDoc, statusStr);
  client.publish("garden/status/board5", statusStr.c_str());
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Smart Garden - Board 5: Pump Control with PWM & WatchDog ===");
  Serial.println("Client ID: " + String(CLIENT_ID));
  Serial.println("Features: PWM Control, WatchDog Timer, Flow Sensor, Pressure Sensor");
  
  lastMaintenanceTime = millis();
  
  // Initialize hardware
  setup_pwm();
  setup_interrupts();
  setup_hardware();
  setup_watchdog();
  
  // Kết nối WiFi và MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  Serial.println("✅ Board 5 sẵn sàng hoạt động!");
  Serial.println("Sử dụng các nút: START, STOP, EMERGENCY để điều khiển");
}

void loop() {
  // WatchDog reset - phải được gọi thường xuyên
  esp_task_wdt_reset();
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  long now = millis();
  
  // Gửi dữ liệu cảm biến định kỳ
  if (now - lastMsg > 5000) {
    lastMsg = now;
    sendSensorData();
  }
  
  // Xử lý buttons
  handleButtons();
  
  // Kiểm tra bảo trì và an toàn
  checkMaintenance();
  
  // Báo cáo trạng thái mỗi 2 phút
  static unsigned long lastStatusReport = 0;
  if (now - lastStatusReport > 120000) {
    lastStatusReport = now;
    sendStatusReport();
  }
  
  delay(100);
}
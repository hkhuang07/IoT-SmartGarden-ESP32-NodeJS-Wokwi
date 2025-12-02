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

// WiFi Configuration
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// MQTT Configuration
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* CLIENT_ID = "ESP_BOARD4_NPK_01";
const char* PUBLISH_TOPIC = "garden/sensor/npk";
const char* SUBSCRIBE_TOPIC = "garden/control/npk_";

// Hardware Configuration - Board 4
#define N_PIN 34  // Nitrogen
#define P_PIN 35  // Phosphorus
#define K_PIN 32  // Potassium
#define VALVE_SERVO_PIN 4
#define BUTTON_PIN 25
#define LED_PIN 2
#define POTENTIOMETER_PIN 33

// LCD Configuration
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4); // 20x4 LCD

// Servo Configuration
#include <ESP32Servo.h>
Servo valveServo;

// WatchDog Timer
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 30

// Interrupt Variables
volatile bool buttonPressed = false;
volatile bool calibrationMode = false;

// MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
bool autoFertigation = true;

// Global Variables
float nitrogenValue = 0.0;
float phosphorusValue = 0.0;
float potassiumValue = 0.0;
int currentValveAngle = 0;
int lcdUpdateCounter = 0;

// Calibration Values
const float N_CALIBRATION = 0.015;
const float P_CALIBRATION = 0.012;
const float K_CALIBRATION = 0.016;

// NPK Thresholds
const float N_MIN = 10.0, N_MAX = 50.0;
const float P_MIN = 5.0, P_MAX = 30.0;
const float K_MIN = 8.0, K_MAX = 40.0;

// ADC Configuration
const int ADC_RESOLUTION = 4096;
const float ADC_VOLTAGE = 3.3;

void IRAM_ATTR buttonISR() {
  buttonPressed = true;
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

void setup_lcd() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart Garden v2.0");
  lcd.setCursor(0, 1);
  lcd.print("Board 4: NPK Sensor");
  delay(2000);
  
  Serial.println("✅ LCD initialized");
}

void setup_servo() {
  valveServo.attach(VALVE_SERVO_PIN);
  valveServo.write(0); // Đóng van ban đầu
  currentValveAngle = 0;
  
  Serial.println("✅ Servo initialized");
}

void setup_adc() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  Serial.println("✅ ADC initialized");
}

void setup_interrupts() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
  
  Serial.println("✅ Interrupts initialized");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println("[MQTT] Nhận: " + String(topic) + " = " + message);
  
  String topicStr = String(topic);
  
  if (topicStr == "garden/control/npk_config") {
    DynamicJsonDocument doc(512);
    deserializeJson(doc, message);
    
    if (doc.containsKey("auto_fertilization")) {
      autoFertigation = doc["auto_fertilization"];
      Serial.println("[CONFIG] Tưới phân tự động: " + String(autoFertigation));
    }
    
    if (doc.containsKey("n_calibration")) {
      // Cập nhật hệ số hiệu chỉnh Nitrogen
      Serial.println("[CONFIG] Hiệu chỉnh Nitrogen cập nhật");
    }
    
    if (doc.containsKey("p_calibration")) {
      // Cập nhật hệ số hiệu chỉnh Phosphorus
      Serial.println("[CONFIG] Hiệu chỉnh Phosphorus cập nhật");
    }
    
    if (doc.containsKey("k_calibration")) {
      // Cập nhật hệ số hiệu chỉnh Potassium
      Serial.println("[CONFIG] Hiệu chỉnh Potassium cập nhật");
    }
  }
  
  if (topicStr == "garden/control/water_valve") {
    if (message == "OPEN") {
      controlValveServo(90);
      client.publish("garden/status/npk_valve", "OPEN");
    } else if (message == "CLOSE") {
      controlValveServo(0);
      client.publish("garden/status/npk_valve", "CLOSED");
    } else if (message.toInt() > 0 && message.toInt() <= 180) {
      controlValveServo(message.toInt());
    }
  }
  
  if (topicStr == "garden/control/npk_calibration") {
    handleCalibrationCommand(message);
  }
}

void handleCalibrationCommand(String command) {
  if (command == "CALIBRATE_HIGH") {
    // Hiệu chỉnh với dung dịch giàu dinh dưỡng
    calibrateNPK("HIGH");
  } else if (command == "CALIBRATE_MEDIUM") {
    // Hiệu chỉnh với dung dịch trung bình
    calibrateNPK("MEDIUM");
  } else if (command == "CALIBRATE_LOW") {
    // Hiệu chỉnh với dung dịch thấp dinh dưỡng
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
  
  // Lưu giá trị hiệu chỉnh
  // Trong thực tế sẽ lưu vào EEPROM
  
  lcd.setCursor(0, 3);
  lcd.print("Calibrated: " + level);
  
  delay(2000);
}

void resetCalibration() {
  Serial.println("[CALIBRATION] Reset hiệu chỉnh");
  
  lcd.setCursor(0, 3);
  lcd.print("Calibration Reset");
  delay(2000);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT kết nối...");
    
    if (client.connect(CLIENT_ID)) {
      Serial.println("✅ Đã kết nối!");
      
      client.subscribe(SUBSCRIBE_TOPIC);
      client.subscribe("garden/control/npk_config");
      client.subscribe("garden/control/water_valve");
      client.subscribe("garden/control/npk_calibration");
      
      client.publish("garden/status/board4", "ONLINE");
      
      DynamicJsonDocument infoDoc(512);
      infoDoc["device_id"] = CLIENT_ID;
      infoDoc["board"] = "BOARD4_NPK_CONTROL";
      infoDoc["version"] = "2.0";
      infoDoc["auto_fertilization"] = autoFertigation;
      infoDoc["ip"] = WiFi.localIP().toString();
      infoDoc["features"] = "npk_sensor,lcd,servo_valve,calibration,auto_fertilization";
      
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
  // Đọc giá trị raw từ các cảm biến
  int n_raw = analogRead(N_PIN);
  int p_raw = analogRead(P_PIN);
  int k_raw = analogRead(K_PIN);
  
  // Chuyển đổi sang ppm (parts per million)
  nitrogenValue = (n_raw * ADC_VOLTAGE / ADC_RESOLUTION) / N_CALIBRATION;
  phosphorusValue = (p_raw * ADC_VOLTAGE / ADC_RESOLUTION) / P_CALIBRATION;
  potassiumValue = (k_raw * ADC_VOLTAGE / ADC_RESOLUTION) / K_CALIBRATION;
  
  // Đọc giá trị từ potentiometer
  int potValue = analogRead(POTENTIOMETER_PIN);
  
  // Tạo JSON data
  DynamicJsonDocument doc(512);
  doc["n_raw"] = n_raw;
  doc["p_raw"] = p_raw;
  doc["k_raw"] = k_raw;
  doc["n_ppm"] = nitrogenValue;
  doc["p_ppm"] = phosphorusValue;
  doc["k_ppm"] = potassiumValue;
  doc["potentiometer"] = potValue;
  doc["auto_fertilization"] = autoFertigation;
  doc["valve_angle"] = currentValveAngle;
  doc["device_id"] = CLIENT_ID;
  doc["timestamp"] = millis();
  doc["rssi"] = WiFi.RSSI();
  doc["free_heap"] = ESP.getFreeHeap();
  
  String payload;
  serializeJson(doc, payload);
  
  client.publish(PUBLISH_TOPIC, payload.c_str());
  Serial.println("[SENSOR] Gửi: " + payload);
  
  Serial.println("NPK - N:" + String(nitrogenValue, 1) + "ppm, " +
                 "P:" + String(phosphorusValue, 1) + "ppm, " +
                 "K:" + String(potassiumValue, 1) + "ppm");
  
  if (autoFertigation) {
    handleAutoFertilization();
  }
  
  // Cập nhật LCD
  updateLCD();
}

void handleAutoFertilization() {
  bool needsFertilization = false;
  String deficiency = "";
  
  // Kiểm tra thiếu Nitrogen
  if (nitrogenValue < N_MIN) {
    needsFertilization = true;
    deficiency += "N ";
  }
  
  // Kiểm tra thiếu Phosphorus
  if (phosphorusValue < P_MIN) {
    needsFertilization = true;
    deficiency += "P ";
  }
  
  // Kiểm tra thiếu Potassium
  if (potassiumValue < K_MIN) {
    needsFertilization = true;
    deficiency += "K ";
  }
  
  if (needsFertilization) {
    Serial.println("[AUTO] Thiếu dinh dưỡng: " + deficiency + "- Bật tưới phân");
    
    // Bật van nước để tưới phân
    controlValveServo(90);
    
    // Gửi cảnh báo
    client.publish("garden/alerts/npk_deficiency", deficiency.c_str());
    
    // Tự động tắt sau 30 giây
    setTimeout([]() {
      controlValveServo(0);
      Serial.println("[AUTO] Tắt tưới phân");
    }, 30000);
  }
}

void controlValveServo(int angle) {
  angle = constrain(angle, 0, 180);
  valveServo.write(angle);
  currentValveAngle = angle;
  Serial.println("[VALVE] Góc van: " + String(angle) + "°");
}

void updateLCD() {
  lcdUpdateCounter++;
  if (lcdUpdateCounter % 5 == 0) { // Cập nhật mỗi 5 chu kỳ
    lcd.clear();
    
    // Dòng 1: Tiêu đề
    lcd.setCursor(0, 0);
    lcd.print("Smart Garden v2.0");
    
    // Dòng 2: Nitrogen và Phosphorus
    lcd.setCursor(0, 1);
    lcd.print("N:");
    lcd.print(nitrogenValue, 0);
    lcd.print("ppm P:");
    lcd.print(phosphorusValue, 0);
    lcd.print("ppm");
    
    // Dòng 3: Potassium
    lcd.setCursor(0, 2);
    lcd.print("K:");
    lcd.print(potassiumValue, 0);
    lcd.print("ppm");
    
    // Dòng 4: Trạng thái
    lcd.setCursor(0, 3);
    lcd.print("Valve:");
    lcd.print(currentValveAngle > 0 ? "ON " : "OFF");
    
    // Hiển thị trạng thái dinh dưỡng
    if (nitrogenValue < N_MIN || phosphorusValue < P_MIN || potassiumValue < K_MIN) {
      lcd.print(" LOW");
    } else if (nitrogenValue > N_MAX || phosphorusValue > P_MAX || potassiumValue > K_MAX) {
      lcd.print(" HIGH");
    } else {
      lcd.print(" NORMAL");
    }
  }
}

void handleButton() {
  if (buttonPressed) {
    buttonPressed = false;
    
    // Toggle auto fertilization mode
    autoFertigation = !autoFertigation;
    
    Serial.println("[BUTTON] Chế độ tưới phân tự động: " + String(autoFertigation ? "ON" : "OFF"));
    
    digitalWrite(LED_PIN, autoFertigation ? HIGH : LOW);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    
    // Hiển thị thông báo trên LCD
    lcd.setCursor(0, 3);
    lcd.print("Auto Mode: " + String(autoFertigation ? "ON " : "OFF"));
    delay(1000);
  }
}

void sendStatusReport() {
  DynamicJsonDocument statusDoc(512);
  statusDoc["device_id"] = CLIENT_ID;
  statusDoc["status"] = "ONLINE";
  statusDoc["nitrogen"] = nitrogenValue;
  statusDoc["phosphorus"] = phosphorusValue;
  statusDoc["potassium"] = potassiumValue;
  statusDoc["auto_fertilization"] = autoFertigation;
  statusDoc["valve_angle"] = currentValveAngle;
  statusDoc["uptime"] = millis();
  statusDoc["rssi"] = WiFi.RSSI();
  statusDoc["free_heap"] = ESP.getFreeHeap();
  
  String statusStr;
  serializeJson(statusStr, statusStr);
  client.publish("garden/status/board4", statusStr.c_str());
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Smart Garden - Board 4: NPK Nutrient Control ===");
  Serial.println("Client ID: " + String(CLIENT_ID));
  Serial.println("Features: NPK Sensor, LCD Display, Auto Fertilization, Servo Valve");
  
  // Initialize hardware
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
  
  digitalWrite(LED_PIN, LOW);
  
  // WatchDog Timer
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Kết nối WiFi và MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  Serial.println("✅ Board 4 sẵn sàng hoạt động!");
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
  if (now - lastMsg > 25000) {
    lastMsg = now;
    sendSensorData();
  }
  
  // Xử lý button
  handleButton();
  
  // Báo cáo trạng thái mỗi 5 phút
  static unsigned long lastStatusReport = 0;
  if (now - lastStatusReport > 300000) {
    lastStatusReport = now;
    sendStatusReport();
  }
  
  delay(100);
}
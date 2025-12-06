/*
 * Smart Garden - Board 2: Soil Moisture Sensor Control (ESP32)
 * Chức năng: Đo độ ẩm đất, điều khiển van nước tự động
 * Áp dụng: C++, I2C/SPI, Digital Analog, Servo
 * Môi trường: Wokwi simulation
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>
#include <LiquidCrystal_I2C.h>

// WiFi Configuration
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// MQTT Configuration
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* CLIENT_ID = "ESP_BOARD2_SOIL_01";
const char* PUBLISH_TOPIC = "garden/sensor/soil_moisture";
const char* SUBSCRIBE_TOPIC = "garden/control/soil_";

// Hardware Configuration - Board 2
#define SOIL_PIN 35
#define SOIL_PIN_ANALOG 34
#define VALVE_SERVO_PIN 4
#define RELAY_PUMP_PIN 5
#define LED_PIN 2
#define BUTTON_PIN 25
#define POTENTIOMETER_PIN 32
#define CALIBRATION_PIN 33

// I2C/SPI Configuration
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define SPI_MOSI_PIN 23
#define SPI_MISO_PIN 19
#define SPI_SCK_PIN 18
#define SPI_CS_PIN 15

// Servo Configuration
Servo valveServo;

// LCD Configuration
LiquidCrystal_I2C lcd(0x27, 20, 4); // I2C address 0x27, 20x4 display

// WatchDog Timer Configuration
#define WDT_TIMEOUT 30

// Interrupt Variables
volatile bool buttonPressed = false;
volatile bool calibrationMode = false;

// MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
bool autoWatering = true;
bool pumpActive = false;

// Global Variables
int currentValveAngle = 0;
int soilMoistureValue = 0;
float soilMoisturePercent = 0.0;
int calibrationOffset = 0;

// Control Parameters
const int MOISTURE_LOW_THRESHOLD = 1500;
const int MOISTURE_HIGH_THRESHOLD = 2500;
const int PUBLISH_INTERVAL = 15000;
const int WATERING_DURATION = 5000;
const int CHECK_INTERVAL = 30000;
const int VALVE_OPEN_ANGLE = 90;
const int VALVE_CLOSED_ANGLE = 0;

// Calibration values
int drySoilValue = 3000;  // Giá trị độ ẩm đất khô
int wetSoilValue = 1000;  // Giá trị độ ẩm đất ướt

// Timer Variables for setTimeout replacement
unsigned long valveCloseTimer = 0;
bool valveTimerActive = false;
unsigned long lastWatering = 0;
bool isWatering = false;

// LCD Update Timer
unsigned long lastLCDUpdate = 0;
const int LCD_UPDATE_INTERVAL = 1000;

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

void setup_i2c_spi() {
  // I2C Setup
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  
  // SPI Setup
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN);
  pinMode(SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  
  Serial.println("✅ I2C/SPI initialized");
}

void setup_lcd() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Smart Garden Board2");
  lcd.setCursor(0, 1);
  lcd.print("Soil Moisture System");
  delay(2000);
  lcd.clear();
  
  Serial.println("✅ LCD initialized");
}

void setup_servo() {
  valveServo.attach(VALVE_SERVO_PIN);
  valveServo.write(VALVE_CLOSED_ANGLE);
  currentValveAngle = VALVE_CLOSED_ANGLE;
  
  Serial.println("✅ Servo initialized");
}

void setup_interrupts() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
  
  Serial.println("✅ Interrupts initialized");
}

void setup_adc() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  Serial.println("✅ ADC initialized");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println("[MQTT] Nhận: " + String(topic) + " = " + message);
  
  String topicStr = String(topic);
  
  if (topicStr == "garden/control/soil_config") {
    DynamicJsonDocument doc(512);
    deserializeJson(doc, message);
    
    if (doc.containsKey("auto_watering")) {
      autoWatering = doc["auto_watering"];
      Serial.println("[CONFIG] Tưới tự động: " + String(autoWatering));
    }
    
    if (doc.containsKey("watering_duration")) {
      // Cập nhật thời gian tưới
      Serial.println("[CONFIG] Thời gian tưới cập nhật");
    }
  }
  
  if (topicStr == "garden/control/water_valve") {
    if (message == "OPEN") {
      controlValve(true);
    } else if (message == "CLOSE") {
      controlValve(false);
    } else if (message.toInt() > 0 && message.toInt() <= 180) {
      controlValveServo(message.toInt());
    }
  }
  
  if (topicStr == "garden/control/soil_calibration") {
    if (message == "CALIBRATE_DRY") {
      calibrateDrySoil();
    } else if (message == "CALIBRATE_WET") {
      calibrateWetSoil();
    } else if (message == "RESET_CALIBRATION") {
      resetCalibration();
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT kết nối...");
    
    if (client.connect(CLIENT_ID)) {
      Serial.println("✅ Đã kết nối!");
      
      client.subscribe(SUBSCRIBE_TOPIC);
      client.subscribe("garden/control/soil_config");
      client.subscribe("garden/control/water_valve");
      client.subscribe("garden/control/soil_calibration");
      
      //client.publish("garden/status/board2", "ONLINE");
      client.publish("garden/status/board2", "{\"status\":\"ONLINE\",\"device\":\"BOARD2_SOIL\"}");

      
      DynamicJsonDocument infoDoc(512);
      infoDoc["device_id"] = CLIENT_ID;
      infoDoc["board"] = "BOARD2_SOIL_CONTROL";
      infoDoc["version"] = "2.0";
      infoDoc["auto_watering"] = autoWatering;
      infoDoc["ip"] = WiFi.localIP().toString();
      infoDoc["features"] = "soil_sensor,servo_valve,relay_pump,calibration,auto_watering,lcd_display";
      
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
  // Đọc dữ liệu từ cảm biến độ ẩm đất
  int soilRaw = analogRead(SOIL_PIN);
  int soilAnalog = analogRead(SOIL_PIN_ANALOG);
  
  // Chuyển đổi sang phần trăm
  soilMoisturePercent = map(soilRaw, drySoilValue, wetSoilValue, 0, 100);
  soilMoisturePercent = constrain(soilMoisturePercent, 0, 100);
  
  soilMoistureValue = soilRaw;
  
  // Đọc các giá trị khác
  int potValue = analogRead(POTENTIOMETER_PIN);
  int calibrationValue = analogRead(CALIBRATION_PIN);
  
  // Tạo JSON data
  DynamicJsonDocument doc(512);
  doc["moisture_value"] = soilMoistureValue;
  doc["analog"] = soilAnalog;
  doc["percentage"] = soilMoisturePercent;
  doc["potentiometer"] = potValue;
  doc["calibration"] = calibrationValue;
  doc["auto_watering"] = autoWatering;
  doc["pump_active"] = pumpActive;
  doc["valve_angle"] = currentValveAngle;
  doc["device_id"] = CLIENT_ID;
  doc["timestamp"] = millis();
  doc["rssi"] = WiFi.RSSI();
  doc["free_heap"] = ESP.getFreeHeap();
  
  String payload;
  serializeJson(doc, payload);
  
  client.publish(PUBLISH_TOPIC, payload.c_str());
  Serial.println("[SENSOR] Gửi: " + payload);
  
  // Cập nhật LCD
  updateLCDDisplay();
  
  if (autoWatering) {
    handleAutoWatering(soilMoistureValue);
  }
}

void handleAutoWatering(int moistureValue) {
  // Kiểm tra thời gian tối thiểu giữa các lần tưới
  if (millis() - lastWatering < 60000) {
    return;
  }
  
  // Độ ẩm thấp - bật bơm
  if (moistureValue > MOISTURE_LOW_THRESHOLD && !isWatering) {
    controlValve(true);
    isWatering = true;
    lastWatering = millis();
    
    Serial.println("[AUTO] Bật tưới nước (độ ẩm thấp: " + String(moistureValue) + ")");
    
    // Tự động tắt sau thời gian quy định - dùng timer system thay vì setTimeout
    valveCloseTimer = millis() + WATERING_DURATION;
    valveTimerActive = true;
  }
  // Độ ẩm cao - đóng van
  else if (moistureValue <= MOISTURE_HIGH_THRESHOLD && isWatering) {
    controlValve(false);
    isWatering = false;
    lastWatering = millis();
    
    Serial.println("[AUTO] Đóng van nước (độ ẩm cao: " + String(moistureValue) + ")");
  }
}

void controlValve(bool open) {
  if (open) {
    controlValveServo(VALVE_OPEN_ANGLE);
    digitalWrite(RELAY_PUMP_PIN, HIGH);
    pumpActive = true;
    Serial.println("[VALVE] Mở van nước");
  } else {
    controlValveServo(VALVE_CLOSED_ANGLE);
    digitalWrite(RELAY_PUMP_PIN, LOW);
    pumpActive = false;
    Serial.println("[VALVE] Đóng van nước");
  }
}

void controlValveServo(int angle) {
  angle = constrain(angle, 0, 180);
  valveServo.write(angle);
  currentValveAngle = angle;
  Serial.println("[SERVO] Góc van: " + String(angle) + "°");
}

void updateLCDDisplay() {
  if (millis() - lastLCDUpdate < LCD_UPDATE_INTERVAL) {
    return;
  }
  lastLCDUpdate = millis();
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Soil Moisture Board");
  lcd.setCursor(0, 1);
  lcd.print("Value: " + String(soilMoistureValue));
  lcd.setCursor(0, 2);
  lcd.print("Percent: " + String((int)soilMoisturePercent) + "%");
  
  lcd.setCursor(0, 3);
  if (pumpActive) {
    lcd.print("Status: PUMP ON ");
  } else {
    lcd.print("Status: PUMP OFF");
  }
  
  // Hiển thị chế độ tưới
  lcd.setCursor(15, 0);
  lcd.print(autoWatering ? "AUTO" : "MANU");
  
  // Hiển thị góc van
  lcd.setCursor(15, 3);
  lcd.print("V" + String(currentValveAngle));
}

void calibrateDrySoil() {
  drySoilValue = analogRead(SOIL_PIN);
  Serial.println("[CALIBRATION] Hiệu chỉnh đất khô: " + String(drySoilValue));
  
  // Lưu giá trị vào EEPROM
  saveCalibrationToMemory();
}

void calibrateWetSoil() {
  wetSoilValue = analogRead(SOIL_PIN);
  Serial.println("[CALIBRATION] Hiệu chỉnh đất ướt: " + String(wetSoilValue));
  
  // Lưu giá trị vào EEPROM
  saveCalibrationToMemory();
}

void resetCalibration() {
  drySoilValue = 3000;
  wetSoilValue = 1000;
  Serial.println("[CALIBRATION] Reset hiệu chỉnh");
  
  saveCalibrationToMemory();
}

void saveCalibrationToMemory() {
  // Mô phỏng lưu vào EEPROM
  Serial.println("[MEMORY] Lưu hiệu chỉnh: Dry=" + String(drySoilValue) + ", Wet=" + String(wetSoilValue));
}

void loadCalibrationFromMemory() {
  // Mô phỏng đọc từ EEPROM
  Serial.println("[MEMORY] Đọc hiệu chỉnh: Dry=" + String(drySoilValue) + ", Wet=" + String(wetSoilValue));
}

void handleButton() {
  if (buttonPressed) {
    buttonPressed = false;
    
    // Bật/tắt chế độ tưới tự động
    autoWatering = !autoWatering;
    
    Serial.println("[BUTTON] Chế độ tưới tự động: " + String(autoWatering ? "ON" : "OFF"));
    
    // Báo hiệu LED
    digitalWrite(LED_PIN, autoWatering ? HIGH : LOW);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    
    // Cập nhật LCD ngay lập tức
    lastLCDUpdate = 0;
  }
}

void sendI2CSPIData() {
  // Gửi dữ liệu qua I2C đến thiết bị khác (M2M communication)
  DynamicJsonDocument commDoc(256);
  commDoc["board"] = CLIENT_ID;
  commDoc["type"] = "soil_data";
  commDoc["moisture"] = soilMoistureValue;
  commDoc["percentage"] = soilMoisturePercent;
  commDoc["auto_watering"] = autoWatering;
  commDoc["pump_active"] = pumpActive;
  commDoc["timestamp"] = millis();
  
  // I2C Communication - sửa lỗi conversion
  Wire.beginTransmission(9); // Địa chỉ thiết bị nhận
  String jsonStr;
  serializeJson(commDoc, jsonStr);
  
  // Chuyển String thành uint8_t array để gửi qua I2C
  uint8_t data[jsonStr.length() + 1];
  jsonStr.getBytes(data, sizeof(data));
  Wire.write(data, jsonStr.length());
  Wire.endTransmission();
  
  // SPI Communication - sửa lỗi const void* to void*
  digitalWrite(SPI_CS_PIN, LOW);
  uint8_t spiData[jsonStr.length() + 1];
  jsonStr.getBytes(spiData, sizeof(spiData));
  SPI.transfer(spiData, jsonStr.length());
  digitalWrite(SPI_CS_PIN, HIGH);
  
  Serial.println("[I2C/SPI] Đã gửi dữ liệu đất");
}

void sendStatusReport() {
  DynamicJsonDocument statusDoc(512);
  statusDoc["device_id"] = CLIENT_ID;
  statusDoc["status"] = "ONLINE";
  statusDoc["moisture_value"] = soilMoistureValue;
  statusDoc["moisture_percent"] = soilMoisturePercent;
  statusDoc["auto_watering"] = autoWatering;
  statusDoc["pump_active"] = pumpActive;
  statusDoc["valve_angle"] = currentValveAngle;
  statusDoc["calibration"] = "Dry:" + String(drySoilValue) + ",Wet:" + String(wetSoilValue);
  statusDoc["uptime"] = millis();
  statusDoc["rssi"] = WiFi.RSSI();
  statusDoc["free_heap"] = ESP.getFreeHeap();
  
  String statusStr;
  serializeJson(statusDoc, statusStr);
  //client.publish("garden/status/board2", statusStr.c_str());
  client.publish("garden/status/board2", "{\"status\":\"ONLINE\",\"device\":\"BOARD2_SOIL\"}");
}

void checkValveTimer() {
  // Kiểm tra timer thay thế cho setTimeout
  if (valveTimerActive && millis() >= valveCloseTimer) {
    controlValve(false);
    isWatering = false;
    valveTimerActive = false;
    lastWatering = millis();
    Serial.println("[AUTO] Tắt tưới nước");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Smart Garden - Board 2: Soil Moisture Control ===");
  Serial.println("Client ID: " + String(CLIENT_ID));
  Serial.println("Features: Soil Sensor, Auto Watering, Servo Valve, Calibration, LCD Display");
  
  // Load calibration từ memory
  loadCalibrationFromMemory();
  
  // Initialize hardware
  setup_i2c_spi();
  setup_lcd();
  setup_servo();
  setup_interrupts();
  setup_adc();
  
  // Set pin modes
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_PUMP_PIN, OUTPUT);
  pinMode(SOIL_PIN, INPUT);
  pinMode(SOIL_PIN_ANALOG, INPUT);
  pinMode(POTENTIOMETER_PIN, INPUT);
  pinMode(CALIBRATION_PIN, INPUT);
  
  digitalWrite(RELAY_PUMP_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  
  // WatchDog Timer - sửa lỗi ESP-IDF v5.x
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  
  // Kết nối WiFi và MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  Serial.println("✅ Board 2 sẵn sàng hoạt động!");
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
  
  // Kiểm tra nút bấm
  handleButton();
  
  // Kiểm tra valve timer (thay thế setTimeout)
  checkValveTimer();
  
  // Gửi dữ liệu I2C/SPI mỗi 45 giây
  static unsigned long lastCommTime = 0;
  if (now - lastCommTime > 45000) {
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
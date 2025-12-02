/*
 * Smart Garden - Board 3: Temperature/Humidity + M2M + LCD Control (ESP32)
 * Chức năng: Cảm biến nhiệt độ/độ ẩm, giao tiếp M2M, LCD, điều khiển
 * Áp dụng: C++ cho cảm biến, MicroPython cho điều khiển, M2M, LCD, Servo
 * Môi trường: Wokwi simulation
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <DHT.h>

// WiFi Configuration
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// MQTT Configuration
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* CLIENT_ID = "ESP_BOARD3_TEMP_M2M_01";
const char* PUBLISH_TOPIC = "garden/sensor/temp_humidity";
const char* SUBSCRIBE_TOPIC = "garden/control/#";

// Hardware Configuration - Board 3
#define DHT_PIN 4
#define DHT_TYPE DHT22
#define LCD_SDA_PIN 21
#define LCD_SCL_PIN 22
#define VALVE_SERVO_PIN 12
#define ROOF_SERVO_PIN 13
#define BUTTON1_PIN 25
#define BUTTON2_PIN 26
#define BUTTON3_PIN 27
#define LED_PIN 2
#define POTENTIOMETER_PIN 32

// I2C/SPI Configuration (for M2M communication)
#define I2C_SDA_PIN 19
#define I2C_SCL_PIN 18
#define SPI_MOSI_PIN 23
#define SPI_MISO_PIN 19
#define SPI_SCK_PIN 18
#define SPI_CS_PIN 5

// LCD Configuration
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4); // 20x4 LCD

// Servo Configuration
#include <ESP32Servo.h>
Servo valveServo;
Servo roofServo;

// DHT Configuration
DHT dht(DHT_PIN, DHT_TYPE);

// WatchDog Timer
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 30

// Interrupt Variables
volatile bool button1Pressed = false;
volatile bool button2Pressed = false;
volatile bool button3Pressed = false;

// MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;

// Global Variables
float currentTemperature = 0.0;
float currentHumidity = 0.0;
int currentValveAngle = 0;
int currentRoofAngle = 0;
int lcdUpdateCounter = 0;
bool autoMode = true;

// M2M Communication
String lastReceivedM2MData = "";
unsigned long lastM2MTime = 0;

// Temperature/Humidity thresholds
const float TEMP_MIN = 18.0;
const float TEMP_MAX = 35.0;
const float HUMIDITY_MIN = 40.0;
const float HUMIDITY_MAX = 80.0;

void IRAM_ATTR button1ISR() {
  button1Pressed = true;
}

void IRAM_ATTR button2ISR() {
  button2Pressed = true;
}

void IRAM_ATTR button3ISR() {
  button3Pressed = true;
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
  lcd.print("Board 3: Temp/M2M");
  delay(2000);
  
  Serial.println("✅ LCD initialized");
}

void setup_servos() {
  valveServo.attach(VALVE_SERVO_PIN);
  roofServo.attach(ROOF_SERVO_PIN);
  
  valveServo.write(0);
  roofServo.write(0);
  
  currentValveAngle = 0;
  currentRoofAngle = 0;
  
  Serial.println("✅ Servos initialized");
}

void setup_i2c_spi() {
  // I2C Setup for M2M communication
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  
  // SPI Setup
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN);
  pinMode(SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  
  Serial.println("✅ I2C/SPI initialized");
}

void setup_interrupts() {
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(BUTTON1_PIN), button1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON2_PIN), button2ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON3_PIN), button3ISR, FALLING);
  
  Serial.println("✅ Interrupts initialized");
}

void setup_dht() {
  dht.begin();
  Serial.println("✅ DHT22 initialized");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println("[MQTT] Nhận: " + String(topic) + " = " + message);
  
  String topicStr = String(topic);
  
  if (topicStr == "garden/control/temp_config") {
    DynamicJsonDocument doc(512);
    deserializeJson(doc, message);
    
    if (doc.containsKey("auto_mode")) {
      autoMode = doc["auto_mode"];
      Serial.println("[CONFIG] Chế độ tự động: " + String(autoMode));
    }
  }
  
  if (topicStr == "garden/control/water_valve") {
    if (message == "OPEN") {
      controlValveServo(90);
      client.publish("garden/status/valve", "OPEN");
    } else if (message == "CLOSE") {
      controlValveServo(0);
      client.publish("garden/status/valve", "CLOSED");
    } else if (message.toInt() > 0 && message.toInt() <= 180) {
      controlValveServo(message.toInt());
    }
  }
  
  if (topicStr == "garden/control/roof_servo") {
    if (message == "OPEN") {
      controlRoofServo(180);
      client.publish("garden/status/roof", "OPEN");
    } else if (message == "CLOSE") {
      controlRoofServo(0);
      client.publish("garden/status/roof", "CLOSED");
    } else if (message.toInt() >= 0 && message.toInt() <= 180) {
      controlRoofServo(message.toInt());
    }
  }
  
  if (topicStr == "garden/control/m2m_data") {
    // Xử lý dữ liệu từ thiết bị khác (M2M)
    handleM2MData(message);
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT kết nối...");
    
    if (client.connect(CLIENT_ID)) {
      Serial.println("✅ Đã kết nối!");
      
      client.subscribe(SUBSCRIBE_TOPIC);
      client.subscribe("garden/control/temp_config");
      client.subscribe("garden/control/water_valve");
      client.subscribe("garden/control/roof_servo");
      client.subscribe("garden/control/m2m_data");
      
      client.publish("garden/status/board3", "ONLINE");
      
      DynamicJsonDocument infoDoc(512);
      infoDoc["device_id"] = CLIENT_ID;
      infoDoc["board"] = "BOARD3_TEMP_M2M";
      infoDoc["version"] = "2.0";
      infoDoc["auto_mode"] = autoMode;
      infoDoc["ip"] = WiFi.localIP().toString();
      infoDoc["features"] = "temp_sensor,humidity_sensor,lcd,m2m_communication,servos,buttons";
      
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
  // Đọc dữ liệu từ DHT22
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("❌ Lỗi đọc cảm biến DHT22!");
    return;
  }
  
  currentTemperature = temperature;
  currentHumidity = humidity;
  
  // Tạo JSON data
  DynamicJsonDocument doc(512);
  doc["t"] = temperature;
  doc["h"] = humidity;
  doc["valve_angle"] = currentValveAngle;
  doc["roof_angle"] = currentRoofAngle;
  doc["auto_mode"] = autoMode;
  doc["device_id"] = CLIENT_ID;
  doc["timestamp"] = millis();
  doc["rssi"] = WiFi.RSSI();
  doc["free_heap"] = ESP.getFreeHeap();
  
  String payload;
  serializeJson(doc, payload);
  
  client.publish(PUBLISH_TOPIC, payload.c_str());
  Serial.println("[SENSOR] Gửi: " + payload);
  
  if (autoMode) {
    handleAutoControl(temperature, humidity);
  }
  
  // Cập nhật LCD
  updateLCD(temperature, humidity);
}

void handleAutoControl(float temperature, float humidity) {
  // Điều khiển mái che dựa trên nhiệt độ
  if (temperature > 32.0) {
    if (currentRoofAngle < 180) {
      controlRoofServo(180);
      Serial.println("[AUTO] Nhiệt độ cao - Mở mái che");
    }
  } else if (temperature < 20.0) {
    if (currentRoofAngle > 0) {
      controlRoofServo(0);
      Serial.println("[AUTO] Nhiệt độ thấp - Đóng mái che");
    }
  }
  
  // Điều khiển van nước dựa trên độ ẩm
  if (humidity < 50.0) {
    if (currentValveAngle == 0) {
      controlValveServo(90);
      Serial.println("[AUTO] Độ ẩm thấp - Mở van nước");
    }
  } else if (humidity > 70.0) {
    if (currentValveAngle > 0) {
      controlValveServo(0);
      Serial.println("[AUTO] Độ ẩm cao - Đóng van nước");
    }
  }
}

void controlValveServo(int angle) {
  angle = constrain(angle, 0, 180);
  valveServo.write(angle);
  currentValveAngle = angle;
  Serial.println("[VALVE] Góc van: " + String(angle) + "°");
}

void controlRoofServo(int angle) {
  angle = constrain(angle, 0, 180);
  roofServo.write(angle);
  currentRoofAngle = angle;
  Serial.println("[ROOF] Góc mái che: " + String(angle) + "°");
}

void updateLCD(float temperature, float humidity) {
  lcdUpdateCounter++;
  if (lcdUpdateCounter % 10 == 0) { // Cập nhật mỗi 10 chu kỳ
    lcd.clear();
    
    // Dòng 1: Tiêu đề
    lcd.setCursor(0, 0);
    lcd.print("Smart Garden v2.0");
    
    // Dòng 2: Nhiệt độ
    lcd.setCursor(0, 1);
    lcd.print("Temp: ");
    lcd.print(temperature, 1);
    lcd.print("C");
    
    // Dòng 3: Độ ẩm
    lcd.setCursor(0, 2);
    lcd.print("Humidity: ");
    lcd.print(humidity, 1);
    lcd.print("%");
    
    // Dòng 4: Trạng thái
    lcd.setCursor(0, 3);
    lcd.print("Valve:");
    lcd.print(currentValveAngle > 0 ? "ON " : "OFF");
    lcd.print(" Roof:");
    lcd.print(currentRoofAngle > 90 ? "ON " : "OFF");
  }
}

void handleM2MData(String data) {
  Serial.println("[M2M] Nhận dữ liệu từ thiết bị khác: " + data);
  
  // Parse JSON data từ thiết bị khác
  DynamicJsonDocument doc(256);
  deserializeJson(doc, data);
  
  // Xử lý dữ liệu nhận được
  if (doc.containsKey("sensor_type")) {
    String sensorType = doc["sensor_type"];
    
    if (sensorType == "soil_moisture") {
      // Đồng bộ hóa với dữ liệu độ ẩm đất
      if (doc.containsKey("percentage")) {
        float soilMoisture = doc["percentage"];
        Serial.println("[M2M] Đồng bộ độ ẩm đất: " + String(soilMoisture) + "%");
        
        // Có thể điều chỉnh các tham số dựa trên dữ liệu này
        if (soilMoisture < 30) {
          controlValveServo(90);
          updateLCD(currentTemperature, currentHumidity);
          lcd.setCursor(0, 3);
          lcd.print("Soil DRY - Watering");
        }
      }
    }
  }
  
  lastReceivedM2MData = data;
  lastM2MTime = millis();
}

void sendM2MData() {
  // Gửi dữ liệu đến thiết bị khác (M2M communication)
  DynamicJsonDocument commDoc(256);
  commDoc["board"] = CLIENT_ID;
  commDoc["sensor_type"] = "temp_humidity";
  commDoc["temperature"] = currentTemperature;
  commDoc["humidity"] = currentHumidity;
  commDoc["valve_angle"] = currentValveAngle;
  commDoc["roof_angle"] = currentRoofAngle;
  commDoc["auto_mode"] = autoMode;
  commDoc["timestamp"] = millis();
  
  // I2C Communication
  Wire.beginTransmission(8); // Địa chỉ thiết bị nhận
  String jsonStr;
  serializeJson(commDoc, jsonStr);
  Wire.write(jsonStr.c_str());
  Wire.endTransmission();
  
  // SPI Communication
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(jsonStr.c_str(), strlen(jsonStr.c_str()));
  digitalWrite(SPI_CS_PIN, HIGH);
  
  Serial.println("[M2M] Đã gửi dữ liệu nhiệt độ/độ ẩm");
}

void handleButtons() {
  if (button1Pressed) {
    button1Pressed = false;
    
    // Button 1: Toggle auto mode
    autoMode = !autoMode;
    
    Serial.println("[BUTTON] Chế độ tự động: " + String(autoMode ? "ON" : "OFF"));
    
    digitalWrite(LED_PIN, autoMode ? HIGH : LOW);
    delay(200);
    digitalWrite(LED_PIN, LOW);
  }
  
  if (button2Pressed) {
    button2Pressed = false;
    
    // Button 2: Manual valve control
    if (currentValveAngle > 0) {
      controlValveServo(0);
    } else {
      controlValveServo(90);
    }
    
    Serial.println("[BUTTON] Điều khiển thủ công van nước");
  }
  
  if (button3Pressed) {
    button3Pressed = false;
    
    // Button 3: Manual roof control
    if (currentRoofAngle < 90) {
      controlRoofServo(180);
    } else {
      controlRoofServo(0);
    }
    
    Serial.println("[BUTTON] Điều khiển thủ công mái che");
  }
}

void sendStatusReport() {
  DynamicJsonDocument statusDoc(512);
  statusDoc["device_id"] = CLIENT_ID;
  statusDoc["status"] = "ONLINE";
  statusDoc["temperature"] = currentTemperature;
  statusDoc["humidity"] = currentHumidity;
  statusDoc["valve_angle"] = currentValveAngle;
  statusDoc["roof_angle"] = currentRoofAngle;
  statusDoc["auto_mode"] = autoMode;
  statusDoc["uptime"] = millis();
  statusDoc["rssi"] = WiFi.RSSI();
  statusDoc["free_heap"] = ESP.getFreeHeap();
  statusDoc["last_m2m"] = lastReceivedM2MData;
  
  String statusStr;
  serializeJson(statusDoc, statusStr);
  client.publish("garden/status/board3", statusStr.c_str());
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Smart Garden - Board 3: Temperature/Humidity + M2M ===");
  Serial.println("Client ID: " + String(CLIENT_ID));
  Serial.println("Features: Temp/Hum Sensor, LCD, M2M Communication, Servos");
  
  // Initialize hardware
  setup_lcd();
  setup_servos();
  setup_i2c_spi();
  setup_interrupts();
  setup_dht();
  
  // Set pin modes
  pinMode(LED_PIN, OUTPUT);
  pinMode(POTENTIOMETER_PIN, INPUT);
  digitalWrite(LED_PIN, LOW);
  
  // WatchDog Timer
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // Kết nối WiFi và MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  Serial.println("✅ Board 3 sẵn sàng hoạt động!");
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
  if (now - lastMsg > 20000) {
    lastMsg = now;
    sendSensorData();
  }
  
  // Xử lý buttons
  handleButtons();
  
  // Gửi dữ liệu M2M mỗi 60 giây
  static unsigned long lastM2MTime = 0;
  if (now - lastM2MTime > 60000) {
    lastM2MTime = now;
    sendM2MData();
  }
  
  // Báo cáo trạng thái mỗi 5 phút
  static unsigned long lastStatusReport = 0;
  if (now - lastStatusReport > 300000) {
    lastStatusReport = now;
    sendStatusReport();
  }
  
  delay(100);
}
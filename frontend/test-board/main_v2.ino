/*
 * Smart Garden - Board 1: Light Sensor Control (ESP32)
 * Chức năng: 
 * - Cảm biến ánh sáng: Thu thập dữ liệu ánh sáng môi trường
 * - Điều khiển đèn LED PWM và Relay light cho vườn
 * - Điều khiển servo mái che và servo van nước dựa trên ánh sáng
 * - Điều khiển thủ công bằng công tắc
 * - Lưu thông tin vào MongoDB qua MQTT
 * 
 * Áp dụng: C++, I2C/SPI, PWM, Interrupt
 * Môi trường: Wokwi simulation
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>
#include "driver/ledc.h"  // ESP32 LEDC PWM driver
#include "esp_timer.h"     // ESP32 timer functions

// ==================== WIFI CONFIGURATION ====================
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// ==================== MQTT CONFIGURATION ====================
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* CLIENT_ID = "ESP_BOARD1_LIGHT_01";

// MQTT Topics - Theo yêu cầu phân tích
const char* PUBLISH_TOPIC = "garden/sensor/light";     // Dữ liệu cảm biến ánh sáng
const char* CONTROL_SUBSCRIBE_TOPIC = "garden/control/light"; // Điều khiển đèn
const char* ROOF_SUBSCRIBE_TOPIC = "garden/control/roof";     // Điều khiển mái che
const char* VALVE_SUBSCRIBE_TOPIC = "garden/control/valve";   // Điều khiển van nước
const char* STATUS_TOPIC = "garden/status/board1";     // Trạng thái board
const char* CONFIG_TOPIC = "garden/system/device_info"; // Thông tin thiết bị

// ==================== HARDWARE CONFIGURATION ====================
// Pin assignments - Theo diagram.json
#define LDR_PIN 32                    // Cảm biến ánh sáng LDR
#define LED_PIN 2                     // Đèn LED chiếu (PWM)
#define RELAY_PIN 4                   // Relay điều khiển đèn vườn
#define SERVO_ROOF_PIN 12             // Servo mái che
#define SERVO_VALVE_PIN 13            // Servo van nước
#define POTENTIOMETER_PIN 34          // Biến trở điều chỉnh ngưỡng
#define SWITCH1_PIN 25                // Công tắc 1 (LED)
#define SWITCH2_PIN 26                // Công tắc 2 (Mái che)
#define SWITCH3_PIN 27                // Công tắc 3 (Van nước)

// ==================== PWM CONFIGURATION ====================
// Điều khiển PWM cho đèn LED chiếu sáng
#define PWM_CHANNEL_LED 0
#define PWM_FREQUENCY 5000            // 5kHz frequency
#define PWM_RESOLUTION 8              // 8-bit resolution (0-255)
#define LED_MIN_BRIGHTNESS 0
#define LED_MAX_BRIGHTNESS 255

// LEDC Constants for ESP32
#define LEDC_SPEED_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_0 LEDC_TIMER_0
#define LEDC_TIMER_8_BIT LEDC_TIMER_8_BIT
#define LEDC_CHANNEL_0 LEDC_CHANNEL_0

// ==================== SERVO CONFIGURATION ====================
Servo roofServo;                     // Servo mái che (0-180°)
Servo valveServo;                    // Servo van nước (0-180°)
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define ROOF_OPEN_ANGLE 180           // Góc mở mái che
#define ROOF_CLOSED_ANGLE 0           // Góc đóng mái che
#define VALVE_OPEN_ANGLE 90           // Góc mở van nước
#define VALVE_CLOSED_ANGLE 0          // Góc đóng van nước

// ==================== I2C/SPI CONFIGURATION ====================
// Áp dụng I2C/SPI theo yêu cầu
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define SPI_MOSI_PIN 23
#define SPI_MISO_PIN 19
#define SPI_SCK_PIN 18
#define SPI_CS_PIN 5

// ==================== CONTROL PARAMETERS ====================
// Ngưỡng điều khiển dựa trên ánh sáng
#define LIGHT_CRITICAL_LOW 200        // Ánh sáng quá thấp - bật đèn tối đa
#define LIGHT_LOW 400                 // Ánh sáng thấp - bật đèn
#define LIGHT_OPTIMAL 800             // Ánh sáng tối ưu - đèn trung bình
#define LIGHT_HIGH 1200               // Ánh sáng cao - giảm đèn
#define LIGHT_CRITICAL_HIGH 2000      // Ánh sáng quá cao - đóng mái che

// Timing
#define PUBLISH_INTERVAL 5000         // 5 giây gửi dữ liệu
#define STATUS_INTERVAL 30000         // 30 giây gửi trạng thái
#define AUTO_CONTROL_INTERVAL 30000   // 30 giây kiểm tra điều khiển tự động

// ==================== WATCHDOG TIMER ====================
#define WDT_TIMEOUT 30                // 30 giây timeout

// ==================== INTERRUPT VARIABLES ====================
volatile bool switch1Pressed = false;    // LED toggle
volatile bool switch2Pressed = false;    // Roof toggle  
volatile bool switch3Pressed = false;    // Valve toggle

// ==================== MQTT CLIENT ====================
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
long lastStatus = 0;
long lastAutoControl = 0;

// ==================== CONTROL MODES ====================
bool autoLightControl = true;        // Điều khiển đèn tự động
bool autoRoofControl = true;         // Điều khiển mái che tự động  
bool autoValveControl = true;        // Điều khiển van nước tự động

// ==================== TIMER VARIABLES ====================
// Thay thế setTimeout bằng simple timer flags
unsigned long valveCloseTimer = 0;
bool valveTimerActive = false;
const unsigned long VALVE_CLOSE_DELAY = 30000;  // 30 giây

// ==================== AUTO CONTROL FLAGS ====================
// Flags cho auto valve control
unsigned long lastWaterTime = 0;
bool wateredRecently = false;

// ==================== CURRENT STATES ====================
int currentLEDValue = 0;             // Giá trị PWM hiện tại (0-255)
int currentRoofAngle = 0;            // Góc mái che hiện tại (0-180)
int currentValveAngle = 0;           // Góc van nước hiện tại (0-180)
bool relayLightState = false;        // Trạng thái relay đèn vườn
String lastAction = "INIT";          // Hành động cuối cùng

// ==================== INTERRUPT SERVICE ROUTINES ====================
void IRAM_ATTR switch1ISR() { switch1Pressed = true; }
void IRAM_ATTR switch2ISR() { switch2Pressed = true; }
void IRAM_ATTR switch3ISR() { switch3Pressed = true; }

// ==================== CONTROL FUNCTIONS ====================

// Điều khiển đèn LED PWM theo ánh sáng
void controlLEDPWM(int brightness) {
  currentLEDValue = constrain(brightness, LED_MIN_BRIGHTNESS, LED_MAX_BRIGHTNESS);
  ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_0, currentLEDValue);
  ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_0);
  Serial.println("[LED] Độ sáng PWM: " + String(currentLEDValue) + "/255");
}

// Điều khiển relay đèn vườn
void controlRelayLight(bool state) {
  relayLightState = state;
  // Relay thường Active-Low: LOW = ON, HIGH = OFF
  digitalWrite(RELAY_PIN, state ? LOW : HIGH);
  Serial.println("[RELAY] Đèn vườn: " + String(state ? "ON" : "OFF"));
}

// Điều khiển servo mái che
void controlRoofServo(int angle) {
  currentRoofAngle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  roofServo.write(currentRoofAngle);
  Serial.println("[ROOF] Mái che: " + String(currentRoofAngle) + "° (" + 
                 (currentRoofAngle > 90 ? "Mở" : "Đóng") + ")");
}

// Điều khiển servo van nước
void controlValveServo(int angle) {
  currentValveAngle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  valveServo.write(currentValveAngle);
  Serial.println("[VALVE] Van nước: " + String(currentValveAngle) + "° (" + 
                 (currentValveAngle > 45 ? "Mở" : "Đóng") + ")");
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
    Serial.println("\n✅ WiFi kết nối thành công!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ WiFi kết nối thất bại");
  }
}

void setup_i2c_spi() {
  // I2C Setup - Áp dụng theo yêu cầu
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.println("[I2C] SDA: GPIO" + String(I2C_SDA_PIN) + ", SCL: GPIO" + String(I2C_SCL_PIN));
  
  // SPI Setup - Áp dụng theo yêu cầu  
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN);
  pinMode(SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  Serial.println("[SPI] MOSI: GPIO" + String(SPI_MOSI_PIN) + 
                 ", MISO: GPIO" + String(SPI_MISO_PIN) + 
                 ", SCK: GPIO" + String(SPI_SCK_PIN));
                 
  Serial.println("✅ I2C/SPI initialized");
}

void setup_pwm() {
  // PWM Setup cho đèn LED chiếu sáng - Fixed struct field order
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = PWM_FREQUENCY,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);
  
  ledc_channel_config_t ledc_channel = {
    .gpio_num = LED_PIN,
    .speed_mode = LEDC_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channel);
  
  // Khởi tạo đèn ở trạng thái tắt
  controlLEDPWM(0);
  
  Serial.println("✅ PWM initialized - LED: GPIO" + String(LED_PIN));
}

void setup_servos() {
  roofServo.attach(SERVO_ROOF_PIN);
  valveServo.attach(SERVO_VALVE_PIN);
  
  // Khởi tạo servo ở trạng thái đóng
  controlRoofServo(ROOF_CLOSED_ANGLE);
  controlValveServo(VALVE_CLOSED_ANGLE);
  
  Serial.println("✅ Servos initialized - Roof: GPIO" + String(SERVO_ROOF_PIN) + 
                 ", Valve: GPIO" + String(SERVO_VALVE_PIN));
}

void setup_interrupts() {
  // Công tắc với INPUT_PULLUP - Logic đảo (nhấn = LOW)
  pinMode(SWITCH1_PIN, INPUT_PULLUP);
  pinMode(SWITCH2_PIN, INPUT_PULLUP);  
  pinMode(SWITCH3_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(SWITCH1_PIN), switch1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(SWITCH2_PIN), switch2ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(SWITCH3_PIN), switch3ISR, FALLING);
  
  Serial.println("✅ Interrupts initialized - Switch1: GPIO" + String(SWITCH1_PIN) + 
                 ", Switch2: GPIO" + String(SWITCH2_PIN) + 
                 ", Switch3: GPIO" + String(SWITCH3_PIN));
}

// ==================== MQTT CALLBACK ====================

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  String topicStr = String(topic);
  Serial.println("[MQTT] Nhận: " + topicStr + " = " + message);
  
  // Điều khiển đèn LED PWM
  if (topicStr == CONTROL_SUBSCRIBE_TOPIC) {
    if (message == "ON") {
      controlLEDPWM(255);
      lastAction = "LED_ON_MAX";
    } else if (message == "OFF") {
      controlLEDPWM(0);
      lastAction = "LED_OFF";
    } else if (message == "AUTO") {
      autoLightControl = true;
      lastAction = "LED_AUTO_ON";
      Serial.println("[AUTO] Điều khiển đèn tự động: ON");
    } else if (message.toInt() >= 0 && message.toInt() <= 255) {
      controlLEDPWM(message.toInt());
      lastAction = "LED_PWM_" + message;
    }
  }
  
  // Điều khiển mái che
  else if (topicStr == ROOF_SUBSCRIBE_TOPIC) {
    if (message == "OPEN") {
      controlRoofServo(ROOF_OPEN_ANGLE);
      lastAction = "ROOF_OPEN";
    } else if (message == "CLOSE") {
      controlRoofServo(ROOF_CLOSED_ANGLE);
      lastAction = "ROOF_CLOSE";
    } else if (message == "AUTO") {
      autoRoofControl = true;
      lastAction = "ROOF_AUTO_ON";
      Serial.println("[AUTO] Điều khiển mái che tự động: ON");
    }
  }
  
  // Điều khiển van nước
  else if (topicStr == VALVE_SUBSCRIBE_TOPIC) {
    if (message == "OPEN") {
      controlValveServo(VALVE_OPEN_ANGLE);
      lastAction = "VALVE_OPEN";
    } else if (message == "CLOSE") {
      controlValveServo(VALVE_CLOSED_ANGLE);
      lastAction = "VALVE_CLOSE";
    } else if (message == "AUTO") {
      autoValveControl = true;
      lastAction = "VALVE_AUTO_ON";
      Serial.println("[AUTO] Điều khiển van nước tự động: ON");
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT kết nối...");
    
    if (client.connect(CLIENT_ID)) {
      Serial.println("✅ MQTT kết nối thành công!");
      
      // Subscribe các topics điều khiển
      client.subscribe(CONTROL_SUBSCRIBE_TOPIC);
      client.subscribe(ROOF_SUBSCRIBE_TOPIC);
      client.subscribe(VALVE_SUBSCRIBE_TOPIC);
      
      // Gửi thông tin thiết bị
      sendDeviceInfo();
      
      // Gửi trạng thái online
      client.publish(STATUS_TOPIC, "{\"status\":\"ONLINE\",\"device\":\"BOARD1_LIGHT\"}");
      
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
  infoDoc["board"] = "BOARD1_LIGHT_CONTROL";
  infoDoc["version"] = "3.0";
  infoDoc["features"] = "light_sensor,pwm_led,relay_light,servos,switches,i2c_spi";
  infoDoc["auto_control"] = autoLightControl && autoRoofControl && autoValveControl;
  infoDoc["ip"] = WiFi.localIP().toString();
  
  JsonArray sensors = infoDoc.createNestedArray("sensors");
  sensors.add("LDR_photoresistor");
  
  JsonArray actuators = infoDoc.createNestedArray("actuators");
  actuators.add("PWM_LED");
  actuators.add("Relay_Light");
  actuators.add("Roof_Servo");
  actuators.add("Valve_Servo");
  
  JsonArray interfaces = infoDoc.createNestedArray("interfaces");
  interfaces.add("I2C");
  interfaces.add("SPI");
  interfaces.add("PWM");
  interfaces.add("Interrupts");
  
  String infoStr;
  serializeJson(infoDoc, infoStr);
  client.publish(CONFIG_TOPIC, infoStr.c_str());
  Serial.println("[INFO] Gửi thông tin thiết bị");
}

// ==================== SENSOR READING & AUTO CONTROL ====================

void sendSensorData() {
  // Đọc dữ liệu từ cảm biến
  int lightRaw = analogRead(LDR_PIN);
  float lightVoltage = (lightRaw / 4095.0) * 3.3;
  int potentiometerValue = analogRead(POTENTIOMETER_PIN);
  
  // Tạo JSON data để gửi về MongoDB
  DynamicJsonDocument doc(1024);
  doc["device_id"] = CLIENT_ID;
  doc["timestamp"] = millis();
  doc["light_raw"] = lightRaw;
  doc["light_voltage"] = lightVoltage;
  doc["light_percentage"] = map(lightRaw, 0, 4095, 0, 100);
  doc["potentiometer"] = potentiometerValue;
  
  // Thông tin điều khiển hiện tại
  JsonObject controls = doc.createNestedObject("controls");
  controls["led_pwm_value"] = currentLEDValue;
  controls["led_brightness"] = String((currentLEDValue * 100 / 255)) + "%";
  controls["relay_light_state"] = relayLightState ? "ON" : "OFF";
  controls["roof_angle"] = currentRoofAngle;
  controls["roof_position"] = currentRoofAngle > 90 ? "OPEN" : "CLOSED";
  controls["valve_angle"] = currentValveAngle;
  controls["valve_position"] = currentValveAngle > 45 ? "OPEN" : "CLOSED";
  
  // Thông tin auto control
  JsonObject autoModes = doc.createNestedObject("auto_modes");
  autoModes["light_auto"] = autoLightControl;
  autoModes["roof_auto"] = autoRoofControl;
  autoModes["valve_auto"] = autoValveControl;
  autoModes["overall_auto"] = autoLightControl || autoRoofControl || autoValveControl;
  
  // Thông tin hệ thống
  JsonObject system = doc.createNestedObject("system");
  system["wifi_rssi"] = WiFi.RSSI();
  system["free_heap"] = ESP.getFreeHeap();
  system["uptime_seconds"] = millis() / 1000;
  system["last_action"] = lastAction;
  
  String payload;
  serializeJson(doc, payload);
  
  // Publish đến MongoDB qua MQTT
  client.publish(PUBLISH_TOPIC, payload.c_str());
  Serial.println("[SENSOR] Gửi dữ liệu: Light=" + String(lightRaw) + 
                ", LED PWM=" + String(currentLEDValue) + 
                ", Roof=" + String(currentRoofAngle) + "°");
}

// Logic điều khiển ánh sáng tự động - Theo yêu cầu phân tích
void handleAutoLightControl(int lightValue) {
  if (!autoLightControl) return;
  
  // Điều chỉnh ngưỡng dựa trên biến trở
  int potValue = analogRead(POTENTIOMETER_PIN);
  int lightThreshold = map(potValue, 0, 4095, 300, 1000);
  
  String action = "";
  
  if (lightValue < LIGHT_CRITICAL_LOW) {
    // Ánh sáng quá thấp - bật đèn tối đa và relay vườn
    controlLEDPWM(255);
    controlRelayLight(true);
    action = "CRITICAL_LOW_LIGHT";
  }
  else if (lightValue < LIGHT_LOW) {
    // Ánh sáng thấp - bật đèn cao và relay vườn
    controlLEDPWM(200);
    controlRelayLight(true);
    action = "LOW_LIGHT_BOOST";
  }
  else if (lightValue < LIGHT_OPTIMAL) {
    // Ánh sáng trung bình - đèn vừa phải
    controlLEDPWM(128);
    controlRelayLight(true);
    action = "MEDIUM_LIGHT_SUPPLEMENT";
  }
  else if (lightValue < LIGHT_HIGH) {
    // Ánh sáng tối ưu - đèn thấp
    controlLEDPWM(64);
    controlRelayLight(false);
    action = "OPTIMAL_LIGHT_MINIMAL";
  }
  else {
    // Ánh sáng cao - tắt đèn
    controlLEDPWM(0);
    controlRelayLight(false);
    action = "HIGH_LIGHT_NO_LED";
  }
  
  if (action != "") {
    lastAction = action;
    Serial.println("[AUTO-LIGHT] " + action + " (Light: " + String(lightValue) + ")");
  }
}

// Logic điều khiển mái che tự động
void handleAutoRoofControl(int lightValue) {
  if (!autoRoofControl) return;
  
  String action = "";
  
  if (lightValue > LIGHT_CRITICAL_HIGH) {
    // Ánh sáng quá cao - đóng mái che để che nắng
    controlRoofServo(ROOF_CLOSED_ANGLE);
    action = "CRITICAL_HIGH_LIGHT_SHADE";
  }
  else if (lightValue > LIGHT_HIGH) {
    // Ánh sáng cao - đóng mái che một phần
    controlRoofServo(135);
    action = "HIGH_LIGHT_PARTIAL_SHADE";
  }
  else if (lightValue > LIGHT_OPTIMAL) {
    // Ánh sáng tối ưu - mở mái che một phần
    controlRoofServo(90);
    action = "OPTIMAL_LIGHT_PARTIAL_OPEN";
  }
  else {
    // Ánh sáng thấp - mở mái che hoàn toàn
    controlRoofServo(ROOF_OPEN_ANGLE);
    action = "LOW_LIGHT_FULL_OPEN";
  }
  
  if (action != "") {
    lastAction = action;
    Serial.println("[AUTO-ROOF] " + action);
  }
}

// Logic điều khiển van nước dựa trên ánh sáng và thời gian
void handleAutoValveControl(int lightValue) {
  if (!autoValveControl) return;
  
  // Logic đơn giản: Khi ánh sáng đủ cao và đã qua 1 tiếng từ lần tưới cuối
  String action = "";
  
  if (lightValue > LIGHT_OPTIMAL && !wateredRecently && 
      (millis() - lastWaterTime > 3600000)) { // 1 tiếng
    // Tưới nước khi đủ ánh sáng và chưa tưới gần đây
    controlValveServo(VALVE_OPEN_ANGLE);
    lastWaterTime = millis();
    wateredRecently = true;
    action = "SCHEDULED_WATERING";
    
    // Tự động đóng van sau 30 giây
    valveCloseTimer = millis() + VALVE_CLOSE_DELAY;
    valveTimerActive = true;
  }
  
  if (action != "") { 
    lastAction = action;
    Serial.println("[AUTO-VALVE] " + action);
  }
}

void handleAutoControl() {
  if (millis() - lastAutoControl < AUTO_CONTROL_INTERVAL) return;
  lastAutoControl = millis();
  
  int lightValue = analogRead(LDR_PIN);
  Serial.println("[AUTO] Kiểm tra điều khiển tự động - Light: " + String(lightValue));
  
  // Thực hiện các logic điều khiển tự động
  if (autoLightControl) handleAutoLightControl(lightValue);
  if (autoRoofControl) handleAutoRoofControl(lightValue);
  if (autoValveControl) handleAutoValveControl(lightValue);
}

// ==================== MANUAL CONTROL ====================

void handleSwitches() {
  if (switch1Pressed) {
    switch1Pressed = false;
    
    // Switch 1: Toggle LED (tắt/bật đèn LED)
    if (currentLEDValue > 0) {
      controlLEDPWM(0);
      autoLightControl = false; // Chuyển sang chế độ thủ công
      lastAction = "MANUAL_LED_TOGGLE_OFF";
    } else {
      controlLEDPWM(255);
      autoLightControl = false; // Chuyển sang chế độ thủ công
      lastAction = "MANUAL_LED_TOGGLE_ON";
    }
    
    Serial.println("[SWITCH] Switch 1 - Toggle LED (Manual)");
  }
  
  if (switch2Pressed) {
    switch2Pressed = false;
    
    // Switch 2: Toggle Roof (mở/đóng mái che)
    if (currentRoofAngle < 90) {
      controlRoofServo(ROOF_OPEN_ANGLE);
      autoRoofControl = false; // Chuyển sang chế độ thủ công
      lastAction = "MANUAL_ROOF_TOGGLE_OPEN";
    } else {
      controlRoofServo(ROOF_CLOSED_ANGLE);
      autoRoofControl = false; // Chuyển sang chế độ thủ công
      lastAction = "MANUAL_ROOF_TOGGLE_CLOSE";
    }
    
    Serial.println("[SWITCH] Switch 2 - Toggle Roof (Manual)");
  }
  
  if (switch3Pressed) {
    switch3Pressed = false;
    
    // Switch 3: Toggle Valve (mở/đóng van nước)
    if (currentValveAngle < 45) {
      controlValveServo(VALVE_OPEN_ANGLE);
      autoValveControl = false; // Chuyển sang chế độ thủ công
      lastAction = "MANUAL_VALVE_TOGGLE_OPEN";
    } else {
      controlValveServo(VALVE_CLOSED_ANGLE);
      autoValveControl = false; // Chuyển sang chế độ thủ công
      lastAction = "MANUAL_VALVE_TOGGLE_CLOSE";
    }
    
    Serial.println("[SWITCH] Switch 3 - Toggle Valve (Manual)");
  }
}

// ==================== STATUS REPORTING ====================

void sendStatusReport() {
  if (millis() - lastStatus < STATUS_INTERVAL) return;
  lastStatus = millis();
  
  DynamicJsonDocument statusDoc(512);
  statusDoc["device_id"] = CLIENT_ID;
  statusDoc["status"] = "ONLINE";
  statusDoc["uptime_seconds"] = millis() / 1000;
  statusDoc["current_light"] = analogRead(LDR_PIN);
  
  // Tạo nested objects thay vì inline object creation
  JsonObject autoModes = statusDoc.createNestedObject("auto_modes");
  autoModes["light"] = autoLightControl;
  autoModes["roof"] = autoRoofControl;
  autoModes["valve"] = autoValveControl;
  
  JsonObject controls = statusDoc.createNestedObject("controls");
  controls["led_pwm"] = currentLEDValue;
  controls["relay_light"] = relayLightState;
  controls["roof_angle"] = currentRoofAngle;
  controls["valve_angle"] = currentValveAngle;
  
  statusDoc["last_action"] = lastAction;
  
  String statusStr;
  serializeJson(statusDoc, statusStr);
  client.publish(STATUS_TOPIC, statusStr.c_str());
  Serial.println("[STATUS] Gửi báo cáo trạng thái");
}

// ==================== I2C/SPI COMMUNICATION ====================

void sendI2CSPIData() {
  // Gửi dữ liệu qua I2C/SPI theo yêu cầu
  DynamicJsonDocument commDoc(256);
  commDoc["board"] = CLIENT_ID;
  commDoc["type"] = "m2m_communication";
  commDoc["light_reading"] = analogRead(LDR_PIN);
  commDoc["timestamp"] = millis();
  commDoc["status"] = "ACTIVE";
  
  String jsonStr;
  serializeJson(commDoc, jsonStr);

  // I2C Communication
  Wire.beginTransmission(8); // Gửi đến slave address 8
  Wire.write((const uint8_t*)jsonStr.c_str(), jsonStr.length());
  Wire.endTransmission();
  
  // SPI Communication
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer((void*)jsonStr.c_str(), jsonStr.length());
  digitalWrite(SPI_CS_PIN, HIGH);
  
  Serial.println("[I2C/SPI] Đã gửi dữ liệu M2M: " + jsonStr);
}

// ==================== SETUP ====================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Smart Garden - Board 1: Light Control System ===");
  Serial.println("🔬 Chức năng: Cảm biến ánh sáng + Điều khiển đèn LED PWM + Relay + Servo");
  Serial.println("🔧 Công nghệ: C++, I2C/SPI, PWM, Interrupt");
  Serial.println("🌐 Môi trường: Wokwi simulation");
  Serial.println();
  
  // WatchDog Timer Setup - Fixed for ESP32 SDK
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  Serial.println("✅ WatchDog Timer initialized (" + String(WDT_TIMEOUT) + "s)");
  
  // Initialize hardware subsystems
  setup_i2c_spi();
  setup_pwm();
  setup_servos();
  setup_interrupts();
  
  // Set pin modes
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(POTENTIOMETER_PIN, INPUT);
  
  // Khởi tạo relay ở trạng thái tắt
  controlRelayLight(false);
  
  Serial.println("✅ Hardware initialization complete");
  Serial.println();
  
  // Network connections
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  Serial.println();
  Serial.println("🌱 Smart Garden Board 1 Light Control Ready!");
  Serial.println("📡 MQTT: " + String(mqtt_server) + ":" + String(mqtt_port));
  Serial.println("🏠 Client ID: " + String(CLIENT_ID));
  Serial.println("📊 Publish: " + String(PUBLISH_TOPIC));
  Serial.println("🎛️ Auto Modes - Light: " + String(autoLightControl ? "ON" : "OFF") + 
                ", Roof: " + String(autoRoofControl ? "ON" : "OFF") + 
                ", Valve: " + String(autoValveControl ? "ON" : "OFF"));
}

// ==================== MAIN LOOP ====================

void loop() {
  // WatchDog reset
  esp_task_wdt_reset();
  
  // Handle valve timer
  if (valveTimerActive && millis() >= valveCloseTimer) {
    controlValveServo(VALVE_CLOSED_ANGLE);
    valveTimerActive = false;
    wateredRecently = false;  // Reset flag sau khi tưới xong
    Serial.println("[AUTO-VALVE] Đã tưới xong, đóng van");
  }
  
  // MQTT connection management
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  // Timing variables
  long now = millis();
  
  // Gửi dữ liệu cảm biến định kỳ (5 giây)
  if (now - lastMsg > PUBLISH_INTERVAL) {
    lastMsg = now;
    sendSensorData();
  }
  
  // Xử lý điều khiển tự động (30 giây)
  handleAutoControl();
  
  // Xử lý công tắc thủ công
  handleSwitches();
  
  // Gửi báo cáo trạng thái (30 giây)
  sendStatusReport();
  
  // Gửi dữ liệu I2C/SPI mỗi 60 giây
  static unsigned long lastCommTime = 0;
  if (now - lastCommTime > 60000) {
    lastCommTime = now;
    sendI2CSPIData();
  }
  
  // Small delay để hệ thống ổn định
  delay(100);
}
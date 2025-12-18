/*
 * ESP32 Soil Moisture Sensor+Control Board - SERVO WORKING VERSION
 * Author: Huynh Quoc Huy (Servo Working)
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <ArduinoJson.h>

// WiFi Configuration
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

// MQTT Configuration
const char* MQTT_BROKER = "broker.hivemq.com";
const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "ESP_SOIL_MOISTURE_01";
const char* MQTT_TOPIC_SENSOR = "garden/sensor/soil_moisture_data";
const char* MQTT_TOPIC_STATUS = "garden/status/soil_board";
const char* MQTT_TOPIC_ALERTS = "garden/alerts/soil_board";
const char* MQTT_TOPIC_COMMANDS = "garden/control/soil_commands";

// Hardware Pin Definitions
const int SOIL_MOISTURE_PIN = 34;
const int VALVE_SERVO_PIN = 12;
const int LED_PIN = 2;
const int BUTTON_PIN = 25;
const int POTENTIOMETER_PIN = 32;
const int LCD_SDA_PIN = 21;
const int LCD_SCL_PIN = 22;
const int LCD_I2C_ADDRESS = 0x27;

// Soil Moisture Thresholds
const int MOISTURE_LOW = 45;
const int MOISTURE_OPTIMAL = 60;
// ADC Configuration 
const int MOISTURE_MAX_RAW = 4095;

// Timing Configuration
const unsigned long AUTOMATION_INTERVAL = 3000;
const unsigned long STATUS_INTERVAL = 5000;

// Servo Angles - SAME AS LIGHT BOARD
const int SERVO_CLOSED_ANGLE = 0;
const int SERVO_MID_ANGLE = 90;
const int SERVO_OPEN_ANGLE = 180;

// System Variables
bool system_initialized = false;
bool mqtt_connected = false;
bool auto_mode = true;
bool valve_servo_active = false;

// Timing Variables
unsigned long last_automation_check = 0;
unsigned long last_status_publish = 0;
unsigned long last_sensor_read = 0;
unsigned long system_start_time = 0;

// PWM Variables
int led_brightness = 0;

// Sensor Variables
float current_moisture = 0.0;
int current_potentiometer = 0;
int moisture_threshold = MOISTURE_OPTIMAL;

// Servo state tracking
int current_servo_angle = -1;
String current_state = "INITIALIZING";

// Global Objects
WiFiClient espClient;
PubSubClient client(espClient);
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 16, 2);
DynamicJsonDocument json_doc(512);
Servo valveServo;

// === SERVO CONTROL ===
void activateValveServo(int angle) {
    angle = constrain(angle, 0, 180); //giới hạn giá trị góc xoay
    valveServo.write(angle);          //xoay góc với giá trị góc angle
    valve_servo_active = (angle != 0);//giá trị valve servo đang hoạt động
    current_servo_angle = angle;      //góc hiện tại bằng angle
    Serial.println("💧 Valve servo: " + String(angle) + "°");
    Serial.println("💧 Valve servo active: " + String(valve_servo_active) + "°");
    Serial.println("💧 Valve servo angle: " + String(current_servo_angle) + "°");
}
void deactivateValveServo() {
    if (valve_servo_active) {
        valveServo.write(0);          //xoay góc với giá trị góc 0
        valve_servo_active = false;   //giá trị valve servo đang tắt
        current_servo_angle = 0;      //góc hiện tại bằng 0
        Serial.println("💧 Valve servo DEACTIVATED: 0 °");
        Serial.println("💧 Valve servo active: " + String(valve_servo_active) + "°");
        Serial.println("💧 Valve servo angle: " + String(current_servo_angle) + "°");
    }
}

// === LED CONTROL ===
void setLEDBrightness(int brightness) { 
    brightness = constrain(brightness, 0, 255);
    if (led_brightness != brightness) {
        led_brightness = brightness;
        analogWrite(LED_PIN, brightness);
        Serial.println("💡 LED: " + String(brightness));
    }
}

// === SENSOR READING ===
float readSoilMoisture() {
    int raw_value = analogRead(SOIL_MOISTURE_PIN);
    int constrained_raw = constrain(raw_value, 0, MOISTURE_MAX_RAW);
    
    // Map ADC (0-4095) sang moisture (100-0%)
    int moisture_int = map(constrained_raw, 0, MOISTURE_MAX_RAW, 100, 0);
    float moisture = constrain((float)moisture_int, 0.0, 100.0);
    
    Serial.print("📊 ADC: "); Serial.print(raw_value);
    Serial.print(" → Moisture: "); Serial.print(moisture, 1); Serial.println("%");
    return moisture;
}
//=== READ POTENTIOMETER ===
int readPotentiometer() {
    int reading = analogRead(POTENTIOMETER_PIN);  
    current_potentiometer = reading;
    moisture_threshold = map(reading, 0, 4095, 20, 80);
    return reading;
}

// === AUTO CONTROL LOGIC ===
void auto_control_logic() {
    current_moisture = readSoilMoisture();

    String target_state = ""; //Trạng thái
    int target_angle = 0;     //Góc xoay 0 độ
    int target_brightness = 0;//Độ sáng 0
    
    if (current_moisture > MOISTURE_LOW) {
        // Độ ẩm 30-60%: Mở van một phần
        target_state = "LOW";               //Cập nhật trạng thái valve thành thấp       
        valveServo.write(90);              //Xoay góc 90 độ
        target_brightness = (float)225/2;  //Độ sáng 127
        
    } else if (current_moisture > MOISTURE_OPTIMAL ) {
         // Độ ẩm >= 60%: Đóng van
        target_state = "OPTIMAL";   //Cập nhật trạng thái valve thành đủ
        valveServo.write(0);        //Xoay góc 0 độ
        target_brightness = 0;      //Độ sáng 0
        
    } else {
        // Độ ẩm < 30%: Mở van hoàn toàn
        target_state = "CRITICAL";  
        valveServo.write(180);        //Xoay góc 0 độ
        target_brightness = 255;      //Độ sáng 0
    }
    
    // Kiểm tra thay đổi
    bool state_changed = false;
    if (target_state != current_state) {
        state_changed = true;
        Serial.println("💧 State changed: " + String(current_state) + " → " + String(target_state));
    }
    bool angle_changed = false;
    if (target_angle != current_servo_angle) {
        angle_changed = true;
        Serial.println("💧 Angle changed: " + String(current_servo_angle) + "° → " + String(target_angle) + "°");
    }
    
    if (state_changed || angle_changed) {
        Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        Serial.print("💧 State: "); Serial.print(current_state); Serial.print(" → "); Serial.println(target_state);
        Serial.print("   Moisture: ");  Serial.print(current_moisture, 1); Serial.println("%");
        
        String status_msg = "";
        if (target_state == "OPTIMAL") {
            status_msg = "OPTIMAL (≥60%) - Valve CLOSED";
        } else if (target_state == "LOW") {
            status_msg = "LOW (30-60%) - Valve PARTIAL";
        } else if (target_state == "CRITICAL") {
            status_msg = "CRITICAL (<30%) - Valve OPEN";
        }
        Serial.println("   " + status_msg);
        Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        // Cập nhật servo và LED
        Serial.println("💧 Activating valve servo to " + String(target_angle) + "°")    ;
        activateValveServo(target_angle);
        Serial.println("💡 Setting LED brightness to " + String(target_brightness)) ;
        setLEDBrightness(target_brightness);
        
        // Lưu trạng thái
        Serial.println("✅ State updated to " + target_state);
        current_state = target_state;

    } else {
        Serial.println("⏸️  State: " + current_state + " (stable)");
    }
}

void checkAutomation() {
    unsigned long current_time = millis();
    if (current_time - last_automation_check < AUTOMATION_INTERVAL) {
        return;
    }
    last_automation_check = current_time;

    if (!auto_mode) {
        Serial.println("⏸️  Auto mode disabled");
        return;
    }
    
    Serial.println("\n🔄 Automation check...");
    auto_control_logic();
}

void updateLCD() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("M:");
    lcd.print((int)current_moisture);
    lcd.print("% T:");
    lcd.print(moisture_threshold);
    lcd.print("%");
    
    lcd.setCursor(0, 1);
    lcd.print(auto_mode ? "AUTO" : "MAN ");
    
    // Hiển thị trạng thái
    if (current_state == "OPTIMAL") {
        lcd.print(" CLOSE");
    } else if (current_state == "LOW") {
        lcd.print(" MID  ");
    } else if (current_state == "CRITICAL") {
        lcd.print(" OPEN ");
    } else {
        lcd.print(" ");
        lcd.print(current_servo_angle);
        lcd.print("d");
    }
}

void publishSensorData() {
    if (!client.connected()) {
        return;
    }
    json_doc.clear();
    json_doc["device_id"] = "soil_moisture_sensor";
    json_doc["moisture_value"] = current_moisture;
    json_doc["moisture_threshold"] = moisture_threshold;
    json_doc["potentiometer_value"] = current_potentiometer;
    json_doc["led_brightness"] = led_brightness;    
    json_doc["auto_mode"] = auto_mode;
    json_doc["valve_servo_angle"] = current_servo_angle;
    json_doc["valve_state"] = current_state;
    json_doc["valve_open"] = current_servo_angle > SERVO_CLOSED_ANGLE;
    json_doc["uptime"] = millis() - system_start_time;
    
    String message;
    serializeJson(json_doc, message);
    
    client.publish(MQTT_TOPIC_SENSOR, message.c_str());
    Serial.println("📤 Data published");
}

void publishSystemStatus() {
    if (!client.connected()) {
        return;
    }
    json_doc.clear();
    json_doc["device_id"] = "soil_moisture_board";
    json_doc["status"] = "online";
    json_doc["valve_state"] = current_state;
    json_doc["valve_angle"] = current_servo_angle;
    json_doc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
    json_doc["mqtt_connected"] = mqtt_connected;
    json_doc["auto_mode"] = auto_mode;
    json_doc["system_uptime"] = millis() - system_start_time;
    json_doc["firmware_version"] = "1.0.8_working";
    json_doc["timestamp"] = millis();
    
    String message;
    serializeJson(json_doc, message);
    
    client.publish(MQTT_TOPIC_STATUS, message.c_str());
    Serial.println("📤 Status published");
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.println("📨 MQTT: " + String(topic));
    
    payload[length] = '\0';
    String message = String((char*)payload);
    
    DeserializationError error = deserializeJson(json_doc, message);
    if (error) {
        Serial.println("❌ JSON error: " + String(error.c_str()));
        return;
    }

    String command = json_doc["command"];
    
    if (command == "set_auto_mode") {
        auto_mode = json_doc["auto_mode"];
        Serial.println("💧 Auto: " + String(auto_mode ? "ON" : "OFF"));
        if (auto_mode) {
            last_automation_check = 0;
        }
        updateLCD();
        
    } else if (command == "control_pump") {
        bool activate = json_doc["activate"];
        auto_mode = false;
        if (activate) {
            Serial.println("🔧 Manual: OPEN");
            activateValveServo(SERVO_OPEN_ANGLE);
            setLEDBrightness(255);
            current_state = "CRITICAL";
        } else {
            Serial.println("🔧 Manual: CLOSE");
            activateValveServo(SERVO_CLOSED_ANGLE);
            setLEDBrightness(0);
            current_state = "OPTIMAL";
        }
        updateLCD();
        
    } else if (command == "set_threshold") {
        moisture_threshold = constrain((int)json_doc["threshold"], 10, 90);
        Serial.println("🎯 Threshold: " + String(moisture_threshold) + "%");
        updateLCD();
        
    } else if (command == "get_status") {
        publishSystemStatus();
        publishSensorData();
    }
}

bool connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    Serial.println("📡 WiFi connecting...");
    unsigned long start_time = millis();
    
    while (WiFi.status() != WL_CONNECTED && (millis() - start_time) < 20000) {
        delay(500);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✓ WiFi OK");
        Serial.println("IP: " + WiFi.localIP().toString());
        return true;
    }
    Serial.println("\n❌ WiFi failed");
    return false;
}

bool connectMQTT() {
    client.setServer(MQTT_BROKER, MQTT_PORT);
    client.setCallback(mqttCallback);
    
    Serial.println("📡 MQTT connecting...");
    
    if (client.connect(MQTT_CLIENT_ID)) {
        Serial.println("✓ MQTT OK");
        mqtt_connected = true;
        client.subscribe(MQTT_TOPIC_COMMANDS);
        publishSystemStatus();
        return true;
    }
    Serial.println("❌ MQTT failed");
    return false;
}

void checkWiFiConnection() {
    unsigned long current_time = millis();
    static unsigned long last_check = 0;
    if (current_time - last_check < 10000) {
        return;
    }
    last_check = current_time;
    
    if (!client.connected() && WiFi.status() == WL_CONNECTED) {
        Serial.println("🔄 MQTT reconnect...");
        mqtt_connected = false;
        connectMQTT();
    }
    
    client.loop();
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n╔════════════════════════════════════╗");
    Serial.println("║  ESP32 Soil Moisture Control      ║");
    Serial.println("║  Version: 1.0.8 (Servo Working)   ║");
    Serial.println("║  Author: Huynh Quoc Huy           ║");
    Serial.println("╚════════════════════════════════════╝\n");
    
    system_start_time = millis();

    // Initialize pins
    pinMode(SOIL_MOISTURE_PIN, INPUT);
    pinMode(POTENTIOMETER_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // SERVO INIT - SAME AS LIGHT BOARD (NO min/max pulse)
    Serial.println("🔧 Attaching servo...");
    Serial.println("   Pin: " + String(VALVE_SERVO_PIN));
    
    valveServo.attach(VALVE_SERVO_PIN);  // ← Simple attach, no PWM params
    
    // Test servo qua 3 vị trí
    Serial.println("🧪 Servo test...");
    
    Serial.println("   Test 1/3: 90° (neutral)");
    valveServo.write(90);
    delay(1000);
    
    Serial.println("   Test 2/3: 0° (closed)");
    valveServo.write(0);
    delay(1000);
    
    Serial.println("   Test 3/3: 180° (open)");
    valveServo.write(180);
    delay(1000);
    
    Serial.println("   → Reset to 0° (init)");
    activateValveServo(0);
    current_state = "OPTIMAL";
    delay(500);
    
    Serial.println("✓ Servo OK\n");

    // Initialize LCD
    Serial.println("🖥️  LCD init...");
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Smart Garden");
    lcd.setCursor(0, 1);
    lcd.print("Starting...");
    delay(2000);
    
    // Connect WiFi
    connectWiFi();
    
    // Connect MQTT
    connectMQTT();
    
    // Read initial sensors
    Serial.println("\n📊 Initial reading...");
    current_moisture = readSoilMoisture();
    readPotentiometer();

    setLEDBrightness(0);
    system_initialized = true;
    updateLCD();
    
    Serial.println("\n╔════════════════════════════════════╗");
    Serial.println("║       SYSTEM READY                 ║");
    Serial.println("╚════════════════════════════════════╝");
    Serial.println("Moisture: " + String(current_moisture, 1) + "%");
    Serial.println("Threshold: " + String(moisture_threshold) + "%");
    Serial.println("Auto Mode: ON");
    Serial.println("Servo: " + String(current_servo_angle) + "°");
    Serial.println("\n🔄 Loop starting...\n");
    
    // Force first automation check
    last_automation_check = 0;
}

void loop() {
    unsigned long current_time = millis();
    
    checkWiFiConnection();
    checkAutomation();
    
    // Publish sensor data every 5 seconds
    if (current_time - last_sensor_read >= 5000) {
        current_moisture = readSoilMoisture();
        readPotentiometer();
        publishSensorData();
        last_sensor_read = current_time;
    }
    
    // Publish status every 5 seconds
    if (current_time - last_status_publish >= STATUS_INTERVAL) {
        publishSystemStatus();
        publishSensorData();
        last_status_publish = current_time;
    }
    
    // Update LCD every 2 seconds
    static unsigned long last_lcd_update = 0;
    if (current_time - last_lcd_update >= 2000) {
        updateLCD();
        last_lcd_update = current_time;
    }
    
    delay(50);
}
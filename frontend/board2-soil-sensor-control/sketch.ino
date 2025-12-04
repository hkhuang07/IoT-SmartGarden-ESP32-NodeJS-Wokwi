/*
 * ESP32 Soil Moisture Control Board
 * C++/Arduino Version for Smart Garden System
 * * Logic Cập Nhật: Điều khiển Servo Van Nước dựa trên độ ẩm đất.
 * Đã cập nhật JSON gửi trạng thái Servo và Độ ẩm lên MQTT.
 * Dùng Servo 90 độ (Mở) và 0 độ (Đóng).
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
const int SOIL_MOISTURE_PIN = 32;
const int VALVE_SERVO_PIN = 4;
const int LED_PIN = 2;
const int RELAY_PIN = 5;
const int BUTTON_PIN = 25;
const int POTENTIOMETER_PIN = 33;
const int LCD_SDA_PIN = 21;
const int LCD_SCL_PIN = 22;
const int LCD_I2C_ADDRESS = 0x27;

// Soil Moisture Thresholds
const int MOISTURE_CRITICAL = 30;
const int MOISTURE_LOW = 40;
const int MOISTURE_OPTIMAL = 60;

// ADC Configuration
const int ADC_RESOLUTION = 4096;
const float ADC_VOLTAGE = 3.3;

// Moisture Calibration
float moisture_calibration = 1.0;

// Timing Configuration
const unsigned long AUTOMATION_INTERVAL = 10000;
const unsigned long STATUS_INTERVAL = 10000;
const unsigned long WIFI_TIMEOUT = 20000;
const unsigned long MQTT_TIMEOUT = 15000;
const unsigned long BUTTON_DEBOUNCE = 200;
const unsigned long PUMP_DURATION = 30000;

// Servo Configuration
const int SERVO_OPEN_ANGLE = 180;   // Mở (Độ ẩm thấp)
const int SERVO_MID_ANGLE = 90;     // Trung bình/Ổn định (Độ ẩm chưa đạt/tối ưu)
const int SERVO_CLOSED_ANGLE = 0;   // Đóng (Độ ẩm cao)

// System Variables
bool system_initialized = false;
bool mqtt_connected = false;
bool auto_mode = true;
bool valve_servo_open = false;
bool relay_active = false;
bool led_active = false;
bool pump_timer_active = false;

// Timing Variables
unsigned long last_automation_check = 0;
unsigned long last_status_publish = 0;
unsigned long last_sensor_read = 0;
unsigned long last_wifi_check = 0;
unsigned long last_mqtt_check = 0;
unsigned long last_button_check = 0;
unsigned long last_pump_start = 0;
unsigned long system_start_time = 0;

// Sensor Variables
float current_moisture = 0.0;
int current_potentiometer = 0;
int current_button_state = HIGH;
int moisture_threshold = MOISTURE_LOW;

// Button State Variables
bool button_last_state = HIGH;
bool button_current_state = HIGH;

// Global Objects
WiFiClient espClient;
PubSubClient client(espClient);
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 16, 2);
DynamicJsonDocument json_doc(512);

// Servo Object
Servo valveServo;

// Utility Functions
void printWiFiStatus() {
    Serial.print("WiFi Status: ");
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Connected");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("Disconnected or Connecting");
    }
}

void printMQTTStatus() {
    Serial.print("MQTT Status: ");
    if (client.connected()) {
        Serial.println("Connected");
    } else {
        Serial.println("Disconnected");
    }
}

void updateLCD() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Moisture:");
    lcd.print(current_moisture, 0);
    lcd.print("%");
    
    lcd.setCursor(0, 1);
    lcd.print(auto_mode ? "AUTO" : "MAN");
    lcd.print(" Thr:");
    lcd.print(moisture_threshold);
    
    // Show valve status
    if (valve_servo_open) {
        lcd.print(" VALVE OPEN");
    } else {
        lcd.print(" VALVE OFF");
    }
}

// Servo Control Functions
void openValveServo() {
    valveServo.write(SERVO_OPEN_ANGLE);
    valve_servo_open = true;
    Serial.println("💧 Valve servo activated (OPEN)");
}

void closeValveServo() {
    valveServo.write(SERVO_CLOSED_ANGLE);
    valve_servo_open = false;
    Serial.println("💧 Valve servo deactivated (CLOSED)");
}

// Relay Control Functions (Không dùng trong logic tự động mới)
void activatePumpRelay() {
    digitalWrite(RELAY_PIN, HIGH);
    relay_active = true;
    Serial.println("💧 Pump relay activated (Manual/Deprecated)");
}

void deactivatePumpRelay() {
    digitalWrite(RELAY_PIN, LOW);
    relay_active = false;
    Serial.println("💧 Pump relay deactivated");
}

// LED Control
void activateIndicatorLED() {
    digitalWrite(LED_PIN, HIGH);
    led_active = true;
}

void deactivateIndicatorLED() {
    digitalWrite(LED_PIN, LOW);
    led_active = false;
}

// Combined control functions (đã được tối ưu hóa cho Servo van)
void startWatering() {
    openValveServo();
    activateIndicatorLED();
    Serial.println("💧 Watering started (Valve Opened)");
}

void stopWatering() {
    closeValveServo();
    deactivateIndicatorLED();
    Serial.println("💧 Watering stopped (Valve Closed)");
}

// Sensor Reading Functions
float readSoilMoisture() {
    int raw_value = analogRead(SOIL_MOISTURE_PIN);
    
    // Convert to percentage (0-100%)
    float voltage = (raw_value * ADC_VOLTAGE) / ADC_RESOLUTION;
    current_moisture = (1.0 - (voltage / ADC_VOLTAGE)) * 100.0 * moisture_calibration;
    current_moisture = constrain(current_moisture, 0.0, 100.0);
    
    return current_moisture;
}

int readPotentiometer() {
    int reading = analogRead(POTENTIOMETER_PIN);
    current_potentiometer = reading;
    
    // Map potentiometer to threshold range (20-80%)
    moisture_threshold = map(reading, 0, 4095, 20, 80);
    
    return reading;
}

// Soil Moisture Automation Logic (Cập nhật để điều khiển Servo van)
void checkAutomation() {
    unsigned long current_time = millis();
    if (current_time - last_automation_check < AUTOMATION_INTERVAL) {
        return;
    }
    last_automation_check = current_time;

    if (!auto_mode || !mqtt_connected) {
        return;
    }

    Serial.println("💧 Running soil moisture automation check...");
    
    // Read sensor values
    readSoilMoisture();
    readPotentiometer();
    
    Serial.println("💧 Moisture: " + String(current_moisture) + "%, Threshold: " + String(moisture_threshold) + "%");

    // LOGIC ĐIỀU KHIỂN SERVO 3 TRẠNG THÁI (0/90/180):

    // 1. ĐỘ ẨM CAO (Đã tưới đủ, cần đóng van)
    if (current_moisture >= MOISTURE_OPTIMAL) {
        if (valveServo.read() != SERVO_CLOSED_ANGLE) {
            valveServo.write(SERVO_CLOSED_ANGLE); // 0 độ
            valve_servo_open = false;
            deactivateIndicatorLED();
            Serial.println("💧 OPTIMAL/HIGH: Moisture sufficient - closing servo (0°)");
        }
    } 
    // 2. ĐỘ ẨM THẤP (Cần tưới ngay, mở van hoàn toàn)
    else if (current_moisture <= moisture_threshold) {
        if (valveServo.read() != SERVO_OPEN_ANGLE) {
            valveServo.write(SERVO_OPEN_ANGLE); // 180 độ
            valve_servo_open = true;
            activateIndicatorLED();
            Serial.println("💧 LOW: Moisture below threshold - opening servo (180°)");
        }
    }
    // 3. ĐỘ ẨM TRUNG BÌNH (Chưa đạt tối ưu, duy trì tưới nhỏ giọt hoặc chuẩn bị)
    else {
        if (valveServo.read() != SERVO_MID_ANGLE) {
            valveServo.write(SERVO_MID_ANGLE); // 90 độ
            valve_servo_open = true; // Coi như đang ở trạng thái kích hoạt tưới
            activateIndicatorLED();
            Serial.println("💧 IN PROGRESS: Moisture not optimal yet - setting servo (90°)");
        }
    }

    updateLCD();
}

void handleButtons() {
    unsigned long current_time = millis();
    if (current_time - last_button_check < BUTTON_DEBOUNCE) {
        return;
    }
    last_button_check = current_time;

    // Read button state
    button_current_state = digitalRead(BUTTON_PIN);

    // Button pressed (FALLING edge)
    if (button_current_state == LOW && button_last_state == HIGH) {
        auto_mode = !auto_mode;
        Serial.println("💧 Mode changed to: " + String(auto_mode ? "AUTO" : "MANUAL"));
        
        // Publish mode change
        if (client.connected()) {
            json_doc.clear();
            json_doc["action"] = "mode_changed";
            json_doc["new_mode"] = auto_mode ? "auto" : "manual";
            json_doc["timestamp"] = millis();
            
            String message;
            serializeJson(json_doc, message);
            client.publish(MQTT_TOPIC_STATUS, message.c_str());
        }
        
        // Manual pump control (Chỉ điều khiển Servo van)
        if (!auto_mode) {
            if (valve_servo_open) {
                stopWatering();
            } else {
                startWatering();
            }
        }
        
        updateLCD();
    }

    // Update button states
    button_last_state = button_current_state;
}

void publishSensorData() {
    if (!client.connected()) {
        return;
    }

    json_doc.clear();
    json_doc["device_id"] = "soil_moisture_board";
    
    // Thêm giá trị cảm biến độ ẩm đất
    json_doc["moisture_value"] = current_moisture; 
    json_doc["moisture_threshold"] = moisture_threshold;
    
    json_doc["potentiometer_value"] = current_potentiometer;
    json_doc["auto_mode"] = auto_mode;
    
    // Thêm trạng thái Servo van nước
    json_doc["valve_servo_angle"] = valveServo.read(); // Đọc góc hiện tại
    json_doc["valve_open"] = valve_servo_open; 
    
    json_doc["led_active"] = led_active;
    json_doc["uptime"] = millis() - system_start_time;
    
    String message;
    serializeJson(json_doc, message);
    
    client.publish(MQTT_TOPIC_SENSOR, message.c_str());
    Serial.println("💧 Soil moisture data published to MQTT");
}

void publishSystemStatus() {
    if (!client.connected()) {
        return;
    }

    json_doc.clear();
    json_doc["device_id"] = "soil_moisture_board";
    json_doc["status"] = "online";
    json_doc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
    json_doc["mqtt_connected"] = mqtt_connected;
    json_doc["auto_mode"] = auto_mode;
    
    // Thêm trạng thái hoạt động của van nước
    json_doc["valve_state"] = valve_servo_open ? "OPEN" : "CLOSED"; 
    
    json_doc["system_uptime"] = millis() - system_start_time;
    json_doc["firmware_version"] = "1.0.1";
    json_doc["timestamp"] = millis();
    
    String message;
    serializeJson(json_doc, message);
    
    client.publish(MQTT_TOPIC_STATUS, message.c_str());
    Serial.println("💧 Soil moisture system status published to MQTT");
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.println("MQTT message received: " + String(topic));
    
    // Null-terminate the payload
    payload[length] = '\0';
    String message = String((char*)payload);
    
    // Parse JSON message
    DeserializationError error = deserializeJson(json_doc, message);
    if (error) {
        Serial.println("JSON parsing failed: " + String(error.c_str()));
        return;
    }

    String command = json_doc["command"];
    
    if (command == "set_auto_mode") {
        bool new_auto_mode = json_doc["auto_mode"];
        auto_mode = new_auto_mode;
        Serial.println("💧 Auto mode set to: " + String(auto_mode ? "AUTO" : "MANUAL"));
        updateLCD();
        
    } else if (command == "control_pump") {
        // Lệnh điều khiển bơm nay điều khiển Servo van
        bool activate = json_doc["activate"];
        if (activate) {
            startWatering();
        } else {
            stopWatering();
        }
        updateLCD();
        
    } else if (command == "set_threshold") {
        int new_threshold = json_doc["threshold"];
        moisture_threshold = constrain(new_threshold, 10, 90);
        Serial.println("💧 Moisture threshold set to: " + String(moisture_threshold) + "%");
        updateLCD();
        
    } else if (command == "get_status") {
        publishSystemStatus();
        publishSensorData();
    }
}

bool connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    Serial.println("Connecting to WiFi...");
    unsigned long start_time = millis();
    
    while (WiFi.status() != WL_CONNECTED && 
           (millis() - start_time) < WIFI_TIMEOUT) {
        delay(500);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected successfully!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        return true;
    } else {
        Serial.println("\nWiFi connection failed!");
        return false;
    }
}

bool connectMQTT() {
    client.setServer(MQTT_BROKER, MQTT_PORT);
    client.setCallback(mqttCallback);
    
    Serial.println("Connecting to MQTT broker...");
    
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        
        if (client.connect(MQTT_CLIENT_ID)) {
            Serial.println("connected");
            mqtt_connected = true;
            
            // Subscribe to command topic
            client.subscribe(MQTT_TOPIC_COMMANDS);
            
            // Publish initial status
            publishSystemStatus();
            
            return true;
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
    
    return false;
}

void checkWiFiConnection() {
    unsigned long current_time = millis();
    if (current_time - last_wifi_check < 30000) { // Check every 30 seconds
        return;
    }
    last_wifi_check = current_time;
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connection lost, attempting to reconnect...");
        connectWiFi();
    }
}

void checkMQTTConnection() {
    unsigned long current_time = millis();
    if (current_time - last_mqtt_check < 10000) { // Check every 10 seconds
        return;
    }
    last_mqtt_check = current_time;
    
    if (!client.connected() && WiFi.status() == WL_CONNECTED) {
        Serial.println("MQTT connection lost, attempting to reconnect...");
        mqtt_connected = false;
        connectMQTT();
    }
    
    client.loop();
}

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("=== ESP32 Soil Moisture Control Board ===");
    Serial.println("Author: MiniMax Agent");
    Serial.println("Starting system initialization...");
    
    // Initialize system timing
    system_start_time = millis();
    system_initialized = false;
    
    // Initialize hardware pins
    Serial.println("Initializing hardware pins...");
    
    // Analog pins
    pinMode(SOIL_MOISTURE_PIN, INPUT);
    pinMode(POTENTIOMETER_PIN, INPUT);
    
    // Digital pins
    pinMode(LED_PIN, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // Initialize servos
    Serial.println("Initializing pump servo...");
    valveServo.attach(VALVE_SERVO_PIN);
    
    // Set servo to stop position
    closeValveServo();
    
    // Initialize LCD
    Serial.println("Initializing LCD display...");
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Smart Garden");
    lcd.setCursor(0, 1);
    lcd.print("Soil Control v1.0");
    delay(2000);
    
    // Initialize WiFi
    Serial.println("Initializing WiFi connection...");
    if (connectWiFi()) {
        Serial.println("WiFi connected successfully!");
        printWiFiStatus();
    } else {
        Serial.println("WiFi connection failed!");
    }
    
    // Initialize MQTT
    Serial.println("Initializing MQTT connection...");
    if (connectMQTT()) {
        Serial.println("MQTT connected successfully!");
        printMQTTStatus();
    } else {
        Serial.println("MQTT connection failed!");
    }
    
    // Initialize sensor readings
    Serial.println("Initializing sensor readings...");
    readSoilMoisture();
    readPotentiometer();
    
    // Initialize outputs
    deactivatePumpRelay();
    deactivateIndicatorLED();
    
    // Final system status
    system_initialized = true;
    updateLCD();
    
    Serial.println();
    Serial.println("=== System initialization complete! ===");
    Serial.println("Soil Moisture Control Board Ready");
    Serial.println("WiFi: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected"));
    Serial.println("MQTT: " + String(client.connected() ? "Connected" : "Disconnected"));
    Serial.println("Auto Mode: " + String(auto_mode ? "Enabled" : "Disabled"));
    Serial.println("Moisture: " + String(current_moisture) + "%, Threshold: " + String(moisture_threshold) + "%");
    Serial.println();
}

void loop() {
    unsigned long current_time = millis();
    
    // Check connections
    checkWiFiConnection();
    checkMQTTConnection();
    
    // Handle button inputs
    handleButtons();
    
    // Check automation
    checkAutomation();
    
    // Publish sensor data periodically
    if (current_time - last_sensor_read >= 5000) { // Every 5 seconds
        readSoilMoisture();
        readPotentiometer();
        publishSensorData();
        last_sensor_read = current_time;
    }
    
    // Publish system status periodically
    if (current_time - last_status_publish >= STATUS_INTERVAL) {
        publishSystemStatus();
        last_status_publish = current_time;
    }
    
    // Update LCD display periodically
    static unsigned long last_lcd_update = 0;
    if (current_time - last_lcd_update >= 2000) { // Update every 2 seconds
        updateLCD();
        last_lcd_update = current_time;
    }
    
    // Small delay to prevent excessive CPU usage
    delay(100);
}
/*
 * ESP32 pH Control Board - PWM STABLE (OPTIMIZED)
 * C++/Arduino Version for Smart Garden System
 * Optimized: Unified servo/LED functions, streamlined logic
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
const char* MQTT_CLIENT_ID = "ESP_PH_CONTROL_01";
const char* MQTT_TOPIC_SENSOR = "garden/sensor/ph_data";
const char* MQTT_TOPIC_STATUS = "garden/status/ph_board";
const char* MQTT_TOPIC_ALERTS = "garden/alerts/ph_board";
const char* MQTT_TOPIC_COMMANDS = "garden/control/ph_commands";

// Hardware Pin Definitions
const int PH_ANALOG_PIN = 34;
const int ACID_SERVO_PIN = 4;
const int ALKALINE_SERVO_PIN = 5;
const int STATUS_LED_PIN = 2;
const int ACID_LED_PIN = 23;
const int ALKALINE_LED_PIN = 26;
const int OPTIMAL_LED_PIN = 27;
const int BUTTON_PIN = 25;
const int PH_THRESHOLD_POT_PIN = 32;
const int PH_CALIBRATION_POT_PIN = 33;
const int LCD_SDA_PIN = 21;
const int LCD_SCL_PIN = 22;
const int LCD_I2C_ADDRESS = 0x27;

// pH Thresholds
const float PH_RANGE_DEVIATION = 0.2;
const float PH_OPTIMAL_MIN_DEFAULT = 6.5;

// ADC Configuration 
const int ADC_RESOLUTION = 4096;

// Servo Configuration
const int SERVO_OPEN_ANGLE = 180;
const int SERVO_CLOSED_ANGLE = 0;
const int SERVO_MID_ANGLE = 90;
const int SERVO_MIN_PULSE_WIDTH = 780;
const int SERVO_MAX_PULSE_WIDTH = 2250;

// Dosing Control
const unsigned long DOSING_DURATION = 3000;
const unsigned long DOSING_COOLDOWN = 15000;

// Timing Configuration
const unsigned long AUTOMATION_INTERVAL = 5000;
const unsigned long STATUS_INTERVAL = 10000;
const unsigned long WIFI_TIMEOUT = 20000;
const unsigned long BUTTON_DEBOUNCE = 200;

// System Variables
bool system_initialized = false;
bool mqtt_connected = false;
bool auto_mode = true;
bool acid_servo_active = false;
bool alkaline_servo_active = false;
bool dosing_active = false;
bool led_pwm_active = false;

// Timing Variables
unsigned long last_automation_check = 0;
unsigned long last_status_publish = 0;
unsigned long last_sensor_read = 0;
unsigned long last_wifi_check = 0;
unsigned long last_mqtt_check = 0;
unsigned long last_button_check = 0;
unsigned long last_dosing_start = 0;
unsigned long system_start_time = 0;

// Sensor Variables
float current_ph = 7.0;
float current_ph_threshold = 7.0;
float ph_calibration = 0.0;
int current_threshold_pot = 0;
int current_calibration_pot = 0;

// LED/PWM Variables
int led_brightness = 0;
int target_led_brightness = 0;

// Button State Variables
bool button_last_state = HIGH;
bool button_current_state = HIGH;

// pH Status Enum
enum PHStatus {
    PH_STRONG_ACIDIC,
    PH_ACIDIC,
    PH_OPTIMAL,
    PH_ALKALINE,
    PH_STRONG_ALKALINE
};
PHStatus current_ph_status = PH_OPTIMAL;

// Enum for dosing types
enum DosingType { ACID, ALKALINE };

// Enum for LED types
enum LEDType { LED_ACID, LED_ALKALINE, LED_OPTIMAL, LED_STATUS };

// Global Objects
WiFiClient espClient;
PubSubClient client(espClient);
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 16, 2);
DynamicJsonDocument json_doc(512);

// Servo Objects
Servo acidServo;
Servo alkalineServo;

// === UTILITY FUNCTIONS ===
String getPHStatusString() {
    switch (current_ph_status) {
        case PH_STRONG_ACIDIC: return "strong_acidic";
        case PH_ACIDIC: return "acidic";
        case PH_OPTIMAL: return "optimal";
        case PH_ALKALINE: return "alkaline";
        case PH_STRONG_ALKALINE: return "strong_alkaline";
        default: return "unknown";
    }
}

// === UNIFIED LED CONTROL ===
void setLED(LEDType type, bool state) {
    int pin;
    String name;
    
    switch(type) {
        case LED_ACID:
            pin = ACID_LED_PIN;
            name = "Acid LED";
            break;
        case LED_ALKALINE:
            pin = ALKALINE_LED_PIN;
            name = "Alkaline LED";
            break;
        case LED_OPTIMAL:
            pin = OPTIMAL_LED_PIN;
            name = "Optimal LED";
            break;
        case LED_STATUS:
            pin = STATUS_LED_PIN;
            name = "Status LED";
            break;
    }
    
    digitalWrite(pin, state ? HIGH : LOW);
}

void deactivateAllLEDs() {
    setLED(LED_ACID, false);
    setLED(LED_ALKALINE, false);
    setLED(LED_OPTIMAL, false);
    setLEDBrightness(0);
}

void setLEDBrightness(int brightness) {
    brightness = constrain(brightness, 0, 255);
    led_brightness = brightness;
    analogWrite(STATUS_LED_PIN, brightness);
    target_led_brightness = brightness;
    led_pwm_active = brightness > 0;
}

// === UNIFIED SERVO CONTROL ===
void activateServo(DosingType type, int angle) {
    angle = constrain(angle, SERVO_CLOSED_ANGLE, SERVO_OPEN_ANGLE);
    
    Servo* servo;
    Servo* other_servo;
    bool* active_flag;
    bool* other_active_flag;
    String name;
    
    switch(type) {
        case ACID:
            servo = &acidServo;
            other_servo = &alkalineServo;
            active_flag = &acid_servo_active;
            other_active_flag = &alkaline_servo_active;
            name = "Acid";
            break;
        case ALKALINE:
            servo = &alkalineServo;
            other_servo = &acidServo;
            active_flag = &alkaline_servo_active;
            other_active_flag = &acid_servo_active;
            name = "Alkaline";
            break;
    }
    
    // Ngăn chặn việc nhỏ cả hai cùng lúc
    if (*other_active_flag) {
        other_servo->write(SERVO_CLOSED_ANGLE);
        *other_active_flag = false;
    }
    
    if (servo->read() != angle) {
        servo->write(angle);
        delay(10);
    }
    
    *active_flag = (angle != SERVO_CLOSED_ANGLE);
    dosing_active = *active_flag;
    
    if (*active_flag) {
        last_dosing_start = millis();
        Serial.println("⚗️ " + name + " servo ACTIVATED (" + String(angle) + "°) starting dose...");
    }
}

void deactivateServo(DosingType type) {
    bool* active_flag;
    Servo* servo;
    String name;
    
    switch(type) {
        case ACID:
            active_flag = &acid_servo_active;
            servo = &acidServo;
            name = "Acid";
            break;
        case ALKALINE:
            active_flag = &alkaline_servo_active;
            servo = &alkalineServo;
            name = "Alkaline";
            break;
    }
    
    if (*active_flag) {
        servo->write(SERVO_CLOSED_ANGLE);
        delay(10);
        *active_flag = false;
        Serial.println("⚗️ " + name + " servo DEACTIVATED (0°)");
    }
}

void stopAllDosing() {
    deactivateServo(ACID);
    deactivateServo(ALKALINE);
    dosing_active = false;
}

// === LCD CONTROL ===
void updateLCD() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("pH:");
    lcd.print(current_ph, 2);
    
    lcd.setCursor(9, 0);
    switch (current_ph_status) {
        case PH_STRONG_ACIDIC: lcd.print("S-ACID"); break;
        case PH_ACIDIC: lcd.print("ACID"); break;
        case PH_OPTIMAL: lcd.print("OPTIMAL"); break;
        case PH_ALKALINE: lcd.print("ALKALINE"); break;
        case PH_STRONG_ALKALINE: lcd.print("S-ALK"); break;
    }
    
    lcd.setCursor(0, 1);
    lcd.print(auto_mode ? "AUTO" : "MAN");
    lcd.print(" Thr:");
    lcd.print(current_ph_threshold, 1);
    
    if (acid_servo_active) {
        lcd.print(" ACID");
    } else if (alkaline_servo_active) {
        lcd.print(" ALK");
    } else {
        lcd.print(" OK");
    }
}

// === SENSOR READING ===
float readPHSensor() {
    int raw_value = analogRead(PH_ANALOG_PIN);
    int constrained_raw = constrain(raw_value, 0, ADC_RESOLUTION - 1);
    long ph_scaled = map((long)constrained_raw, 0, ADC_RESOLUTION - 1, 0, 14000);
    float raw_ph = ph_scaled / 1000.0;
    
    current_ph = raw_ph + ph_calibration;
    current_ph = constrain(current_ph, 0.0, 14.0);

    Serial.print("Raw ADC (GPIO34): "); Serial.print(raw_value);
    Serial.print(", PH Calculated: "); Serial.print(current_ph, 2);
    Serial.print(", Calibration Offset: "); Serial.println(ph_calibration, 2);

    updatePHStatus();
    return current_ph;
}

void updatePHStatus() {
    if (current_ph < 5.0) {
        current_ph_status = PH_STRONG_ACIDIC;
    } else if (current_ph < 6.5) {
        current_ph_status = PH_ACIDIC;
    } else if (current_ph >= 6.5 && current_ph <= 7.5) {
        current_ph_status = PH_OPTIMAL;
    } else if (current_ph <= 9.0) {
        current_ph_status = PH_ALKALINE;
    } else {
        current_ph_status = PH_STRONG_ALKALINE;
    }
}

int readThresholdPotentiometer() {
    int reading = analogRead(PH_THRESHOLD_POT_PIN);
    current_threshold_pot = reading;
    current_ph_threshold = map((long)reading, 0, ADC_RESOLUTION - 1, 50, 90) / 10.0;
    return reading;
}

int readCalibrationPotentiometer() {
    int reading = analogRead(PH_CALIBRATION_POT_PIN);
    current_calibration_pot = reading;
    ph_calibration = map((long)reading, 0, ADC_RESOLUTION - 1, -200, 200) / 100.0;
    return reading;
}

void calibratePH() {
    readCalibrationPotentiometer();
    Serial.println("⚗️ Calibrate: pH Calibration Offset set via Potentiometer: " + String(ph_calibration, 2));
    
    for (int i = 0; i < 3; i++) {
        setLEDBrightness(255);
        delay(100);
        setLEDBrightness(0);
        delay(100);
    }
    auto_control_logic();
}

// === AUTO CONTROL LOGIC ===
void auto_control_logic() {
    unsigned long current_time = millis();

    // Check if currently dosing
    if (dosing_active) {
        if (current_time - last_dosing_start >= DOSING_DURATION) {
            stopAllDosing();
            last_automation_check = current_time;
            deactivateAllLEDs();
            setLED(LED_STATUS, true);
        }
        return;
    }
    
    // Check if auto mode is disabled
    if (!auto_mode) {
        stopAllDosing();
        deactivateAllLEDs();
        setLED(LED_STATUS, true);
        return;
    }

    // Check cooldown period
    if (current_time - last_automation_check < DOSING_COOLDOWN) {
        deactivateAllLEDs();
        setLED(LED_STATUS, true);
        return;
    }
    
    last_automation_check = current_time;
    
    float ph_level = readPHSensor();
    float ph_target_min = current_ph_threshold - PH_RANGE_DEVIATION;
    float ph_target_max = current_ph_threshold + PH_RANGE_DEVIATION;
    
    // Determine action based on pH level
    if (ph_level > ph_target_max) {
        // pH too high - needs acid
        Serial.println("⚗️ AUTO: PH Alkaline (" + String(ph_level, 2) + ") > Max Thr (" + String(ph_target_max, 2) + "). Acid dosing required.");
        deactivateAllLEDs();
        setLED(LED_ACID, true);
        setLEDBrightness(255);
        acidServo.write(SERVO_OPEN_ANGLE);
        alkalineServo.write(SERVO_CLOSED_ANGLE);
        acid_servo_active = true;
        alkaline_servo_active = false;
        dosing_active = true;
        last_dosing_start = current_time;
        
    } else if (ph_level < ph_target_min) {
        // pH too low - needs alkaline
        Serial.println("⚗️ AUTO: PH Acidic (" + String(ph_level, 2) + ") < Min Thr (" + String(ph_target_min, 2) + "). Alkaline dosing required.");
        deactivateAllLEDs();
        setLED(LED_ALKALINE, true);
        setLEDBrightness(255);
        alkalineServo.write(SERVO_OPEN_ANGLE);
        acidServo.write(SERVO_CLOSED_ANGLE);
        alkaline_servo_active = true;
        acid_servo_active = false;
        dosing_active = true;
        last_dosing_start = current_time;
        
    } else {
        // pH optimal
        Serial.println("⚗️ AUTO: PH Optimal (" + String(ph_level, 2) + ") in range (" + String(ph_target_min, 2) + "-" + String(ph_target_max, 2) + "). Dosing stopped.");
        deactivateAllLEDs();
        setLED(LED_OPTIMAL, true);
        setLEDBrightness(0);
        alkalineServo.write(SERVO_CLOSED_ANGLE);
        acidServo.write(SERVO_CLOSED_ANGLE);
        acid_servo_active = false;
        alkaline_servo_active = false;
        dosing_active = false;
    }
}

// === MQTT FUNCTIONS ===
void publishSensorData() {
    if (!client.connected()) { return; }
    json_doc.clear();
    json_doc["device_id"] = "ph_control_board";
    json_doc["ph_value"] = current_ph;
    json_doc["ph_status"] = getPHStatusString();
    json_doc["ph_threshold"] = current_ph_threshold;
    json_doc["ph_calibration"] = ph_calibration;
    json_doc["analog_raw"] = analogRead(PH_ANALOG_PIN);
    json_doc["auto_mode"] = auto_mode;
    json_doc["acid_servo_angle"] = acidServo.read();
    json_doc["alkali_servo_angle"] = alkalineServo.read();
    json_doc["dosing_active"] = dosing_active;
    json_doc["uptime"] = millis() - system_start_time;
    String message;
    serializeJson(json_doc, message);
    client.publish(MQTT_TOPIC_SENSOR, message.c_str());
    Serial.println("⚗️ pH sensor data published to MQTT");
}

void publishSystemStatus() {
    if (!client.connected()) { return; }
    json_doc.clear();
    json_doc["device_id"] = "ph_control_board";
    json_doc["status"] = "online";
    json_doc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
    json_doc["mqtt_connected"] = client.connected();
    json_doc["auto_mode"] = auto_mode;
    json_doc["system_uptime"] = millis() - system_start_time;
    json_doc["firmware_version"] = "1.2.0_optimized";
    json_doc["timestamp"] = millis();
    String message;
    serializeJson(json_doc, message);
    client.publish(MQTT_TOPIC_STATUS, message.c_str());
    Serial.println("⚗️ pH system status published to MQTT");
}

void handleButtons() {
    unsigned long current_time = millis();
    if (current_time - last_button_check < BUTTON_DEBOUNCE) { return; }
    button_current_state = digitalRead(BUTTON_PIN);

    if (button_current_state == LOW && button_last_state == HIGH) {
        auto_mode = !auto_mode;
        Serial.println("⚗️ Mode changed to: " + String(auto_mode ? "AUTO" : "MANUAL"));
        if (!auto_mode) stopAllDosing();
        updateLCD();
    }
    button_last_state = button_current_state;
    last_button_check = current_time;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.println("MQTT message received: " + String(topic));
    payload[length] = '\0';
    String message = String((char*)payload);
    DeserializationError error = deserializeJson(json_doc, message);
    if (error) { Serial.println("JSON parsing failed: " + String(error.c_str())); return; }
    String command = json_doc["command"];
    
    if (command == "set_auto_mode") {
        bool new_auto_mode = json_doc["auto_mode"];
        auto_mode = new_auto_mode;
        if (!auto_mode) stopAllDosing();
        Serial.println("⚗️ Auto mode set to: " + String(auto_mode ? "AUTO" : "MANUAL"));
        
    } else if (command == "set_ph_threshold") {
        float new_threshold = json_doc["threshold"];
        current_ph_threshold = constrain(new_threshold, 4.0, 10.0);
        Serial.println("⚗️ pH threshold set to: " + String(current_ph_threshold));
        
    } else if (command == "calibrate_ph") {
        float new_offset = json_doc["offset"] | 0.0;
        ph_calibration = constrain(new_offset, -2.0, 2.0);
        Serial.println("⚗️ Calibrate: PH Offset set via MQTT: " + String(ph_calibration, 2));
        
    } else if (command == "start_acid_dosing") {
        if (!auto_mode) activateServo(ACID, SERVO_OPEN_ANGLE);
        
    } else if (command == "start_alkaline_dosing") {
        if (!auto_mode) activateServo(ALKALINE, SERVO_OPEN_ANGLE);
        
    } else if (command == "stop_dosing") {
        stopAllDosing();
        
    } else if (command == "get_status") {
        publishSystemStatus();
        publishSensorData();
    }
    updateLCD();
}

// === WIFI & MQTT CONNECTION ===
bool connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.println("Connecting to WiFi...");
    unsigned long start_time = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start_time) < WIFI_TIMEOUT) {
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
    unsigned long start_time = millis();
    while (!client.connected() && (millis() - start_time) < 15000) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect(MQTT_CLIENT_ID)) {
            Serial.println("connected");
            mqtt_connected = true;
            client.subscribe(MQTT_TOPIC_COMMANDS);
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
    if (current_time - last_wifi_check < 30000) { return; }
    last_wifi_check = current_time;
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connection lost, attempting to reconnect...");
        connectWiFi();
    }
}

void checkMQTTConnection() {
    unsigned long current_time = millis();
    if (current_time - last_mqtt_check < 10000) { return; }
    last_mqtt_check = current_time;
    if (!client.connected() && WiFi.status() == WL_CONNECTED) {
        Serial.println("MQTT connection lost, attempting to reconnect...");
        mqtt_connected = false;
        connectMQTT();
    }
    client.loop();
}

// === SETUP ===
void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("=== ESP32 pH Control Board (Optimized) ===");
    Serial.println("Version: 1.2.0");
    Serial.println("Starting system initialization...");
    
    system_start_time = millis();
    system_initialized = false;
    
    // Initialize hardware pins
    Serial.println("Initializing hardware pins...");
    pinMode(PH_ANALOG_PIN, INPUT);
    pinMode(PH_THRESHOLD_POT_PIN, INPUT);
    pinMode(PH_CALIBRATION_POT_PIN, INPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(ACID_LED_PIN, OUTPUT);
    pinMode(ALKALINE_LED_PIN, OUTPUT);
    pinMode(OPTIMAL_LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // Initialize servos
    Serial.println("Initializing dosing servos...");
    acidServo.attach(ACID_SERVO_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
    alkalineServo.attach(ALKALINE_SERVO_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
    
    // Initialization pulse
    Serial.println("Sending initialization pulse to both servos (90 deg)...");
    acidServo.write(SERVO_MID_ANGLE);
    alkalineServo.write(SERVO_MID_ANGLE);
    delay(1000);

    // Close servos
    stopAllDosing();
    
    // Initialize LCD
    Serial.println("Initializing LCD display...");
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Smart Garden");
    lcd.setCursor(0, 1);
    lcd.print("pH Control v1.2");
    delay(2000);
    
    // Initialize WiFi & MQTT
    Serial.println("Initializing WiFi connection...");
    connectWiFi();
    Serial.println("Initializing MQTT connection...");
    connectMQTT();
    
    // Initialize sensor readings
    Serial.println("Initializing sensor readings and calibration...");
    readCalibrationPotentiometer();
    readPHSensor();
    readThresholdPotentiometer();
    
    // Initialize LEDs
    deactivateAllLEDs();
    
    // Final system status
    system_initialized = true;
    updateLCD();
    
    Serial.println();
    Serial.println("=== System initialization complete! ===");
    Serial.println("pH Control Board Ready");
    Serial.println("WiFi: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected"));
    Serial.println("MQTT: " + String(client.connected() ? "Connected" : "Disconnected"));
    Serial.println("Auto Mode: " + String(auto_mode ? "Enabled" : "Disabled"));
    Serial.println("pH: " + String(current_ph, 2) + ", Threshold: " + String(current_ph_threshold, 1) + ", Offset: " + String(ph_calibration, 2));
    Serial.println();
}

// === LOOP ===
void loop() {
    unsigned long current_time = millis();
    
    // Check connections & MQTT loop
    checkWiFiConnection();
    checkMQTTConnection();
    
    // Handle button inputs
    handleButtons();
    
    // Check automation logic
    if (auto_mode) {
        auto_control_logic();
    } else {
        stopAllDosing();
        deactivateAllLEDs();
        setLED(LED_STATUS, true);
    }
    
    // Publish sensor data periodically
    if (current_time - last_sensor_read >= 5000) {
        readPHSensor();
        readThresholdPotentiometer();
        readCalibrationPotentiometer();
        publishSensorData();
        last_sensor_read = current_time;
    }
    
    // Publish system status periodically
    if (current_time - last_status_publish >= STATUS_INTERVAL) {
        publishSystemStatus();
        publishSensorData();
        last_status_publish = current_time;
    }
    
    // Update LCD display periodically
    static unsigned long last_lcd_update = 0;
    if (current_time - last_lcd_update >= 2000) {
        updateLCD();
        last_lcd_update = current_time;
    }
    
    // Small delay to prevent excessive CPU usage
    delay(100);
}
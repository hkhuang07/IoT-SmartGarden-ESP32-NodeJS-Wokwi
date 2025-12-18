/*
 * ESP32 Light Sensor Control Board
 * C++/Arduino Version for Smart Garden System
 * * Hardware Configuration (from Wokwi diagram):
 * - LDR Sensor: GPIO32 (Analog)
 * - PWM LED: GPIO2 
 * - Relay: GPIO4
 * - Servo 1 (Roof): GPIO12
 * - Servo 2 (Valve): GPIO13
 * * LOGIC: All actuators controlled by Light Sensor (LDR).
 * * Author: Huynh Quoc Huy
 * Date: 2025-12-02
 */


#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h> 
// Sử dụng ESP32Servo.h giúp tự động hóa cấu hình LEDC cho Servo

// WiFi Configuration
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

// MQTT Configuration
const char* MQTT_BROKER = "broker.hivemq.com";
const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "ESP_LIGHT_CONTROL_01";
const char* MQTT_TOPIC_SENSOR = "garden/sensor/light_data";
const char* MQTT_TOPIC_STATUS = "garden/status/light_board";
const char* MQTT_TOPIC_ALERTS = "garden/alerts/light_board";
const char* MQTT_TOPIC_COMMANDS = "garden/control/light_commands";

// Hardware Pin Definitions
const int LDR_SENSOR_PIN = 32;
const int LED_PWM_PIN = 2;
const int RELAY_PIN = 4;
const int ROOF_SERVO_PIN1 = 12; 
const int ROOF_SERVO_PIN2 = 13;
const int POTENTIOMETER_PIN = 34;
const int BUTTON_1_PIN = 25;
const int BUTTON_2_PIN = 26;
const int BUTTON_3_PIN = 27;
const int LCD_SDA_PIN = 21;
const int LCD_SCL_PIN = 22;
const int LCD_I2C_ADDRESS = 0x27;

// Light Thresholds (Analog values from LDR, 0-4095 on ESP32)
const int LIGHT_MIN = 800;
const int LIGHT_MAX = 3500;
const int DARKNESS_THRESHOLD = 200;

// Timing Configuration
const unsigned long AUTOMATION_INTERVAL = 1000; 
const unsigned long STATUS_INTERVAL = 5000;
const unsigned long WIFI_TIMEOUT = 20000;
const unsigned long MQTT_TIMEOUT = 15000;
const unsigned long BUTTON_DEBOUNCE = 200;

// System Variables
bool system_initialized = false;
bool mqtt_connected = false;
bool auto_mode = true;
bool roof_servo_active1 = false;
bool roof_servo_active2 = false;
bool relay_active = false;
bool led_pwm_active = false;

// Timing Variables
unsigned long last_automation_check = 0;
unsigned long last_status_publish = 0;
unsigned long last_sensor_read = 0;
unsigned long last_wifi_check = 0;
unsigned long last_mqtt_check = 0;
unsigned long last_button_check = 0;
unsigned long system_start_time = 0;

// PWM Variables
int led_brightness = 0;
int target_led_brightness = 0;

// Sensor Variables
int current_light_level = 0;
int current_potentiometer = 0;

// Button State Variables
bool button_1_last_state = HIGH;
bool button_2_last_state = HIGH;
bool button_3_last_state = HIGH;
bool button_1_current_state = HIGH;
bool button_2_current_state = HIGH;
bool button_3_current_state = HIGH;

// Global Objects
WiFiClient espClient;
PubSubClient client(espClient);
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 16, 2);
DynamicJsonDocument json_doc(512);

Servo roofServo1;
Servo roofServo2;

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
    lcd.print("Light: ");
    lcd.print(current_light_level);
    
    lcd.setCursor(10, 0);
    lcd.print(auto_mode ? "AUTO" : "MAN");
    
    lcd.setCursor(0, 1);
    lcd.print("LED:");
    lcd.print(led_brightness);
    
    lcd.setCursor(8, 1);
    if (roof_servo_active1 || roof_servo_active2 ) {
        lcd.print("SRV ON");
    } else {
        lcd.print("SRV OFF");
    }
}

// HÀM ĐIỀU KHIỂN SERVO BẰNG THƯ VIỆN SERVO
void activateRoofServo(int angle) {
    roofServo1.write(constrain(angle, 0, 180));
    roofServo2.write(constrain(angle, 0, 180));
    roof_servo_active1 = (angle != 0);
    roof_servo_active2 = (angle != 0);
    Serial.println("🏠 Roof servo 1 ACTIVATED: " + String(angle) + " degrees");
    Serial.println("🏠 Roof servo 2 ACTIVATED: " + String(angle) + " degrees");
}

void deactivateRoofServo() {
    if (roof_servo_active1 || roof_servo_active2) {
        roofServo1.write(0); 
        roofServo2.write(0); 
        roof_servo_active1 = false;
        roof_servo_active2 = false;
        Serial.println("🏠 Roof servo DEACTIVATED");
    }
}

/*void activateValveServo(int angle) {
    valveServo.write(constrain(angle, 0, 180));
    valve_servo_active = (angle != 0);
    Serial.println("💧 Valve servo ACTIVATED: " + String(angle) + " degrees");
}*/

/*void deactivateValveServo() {
    if (valve_servo_active) {
        valveServo.write(0); 
        valve_servo_active = false;
        Serial.println("💧 Valve servo DEACTIVATED");
    }
}*/

// HÀM ĐIỀU KHIỂN LED 
void setLEDBrightness(int brightness) { 
    brightness = constrain(brightness, 0, 255);
    led_brightness = brightness;
    
    // Sử dụng analogWrite (PWM)
    analogWrite(LED_PWM_PIN, brightness);
    
    target_led_brightness = brightness;
    led_pwm_active = brightness > 0;
    Serial.println("LED brightness set to: " + String(brightness));
}

void activateRelay() {
    if (!relay_active) {
        digitalWrite(RELAY_PIN, HIGH);
        relay_active = true;
        Serial.println("Relay activated");
    }
}

void deactivateRelay() {
    if (relay_active) {
        digitalWrite(RELAY_PIN, LOW);
        relay_active = false;
        Serial.println("Relay deactivated");
    }
}

int readLightSensor() {
    int reading = analogRead(LDR_SENSOR_PIN);
    current_light_level = reading;
    return reading;
}

int readPotentiometer() {
    int reading = analogRead(POTENTIOMETER_PIN);
    current_potentiometer = reading;
    return reading;
}

void auto_control_logic() {
    int light_level = readLightSensor();
    // --- 1. SÁNG QUÁ MỨC ---
    if (light_level > LIGHT_MAX) {
        Serial.println("☀️ AUTO: LIGHT TOO HIGH. Maximize roof opening.");
        deactivateRelay();
        setLEDBrightness(0);
        activateRoofServo(180); 
        //deactivateValveServo();
    // --- 2. TỐI ---
    } else if (light_level < LIGHT_MIN) {
        Serial.println("🌑 AUTO: DARKNESS DETECTED. Activating artificial light.");
        activateRelay();
        setLEDBrightness(255); 
        activateRoofServo(0); 
        //deactivateValveServo();
    // --- 3. ÁNH SÁNG VỪA PHẢI ---
    } else {
        Serial.println("🌥️ AUTO: Light level stable. Adjusting LED.");
        int pwm_value = map(light_level, LIGHT_MIN, LIGHT_MAX, 255, 0); 
        pwm_value = constrain(pwm_value, 0, 150); 
        
        setLEDBrightness(pwm_value);
        if (pwm_value > 0) {
            activateRelay();
        } else {
            deactivateRelay();
        }
        activateRoofServo(0); 
        //deactivateValveServo();
    }
}

void checkAutomation() {
    unsigned long current_time = millis();
    if (current_time - last_automation_check < 500) { 
        return;
    }
    last_automation_check = current_time;

    if (!auto_mode) {
        return;
    }
    
    readLightSensor();
    readPotentiometer(); 
    
    auto_control_logic();
}

void publishSensorData() {
    if (!client.connected()) {
        return;
    }
    json_doc.clear();
    json_doc["device_id"] = "light_sensor";
    json_doc["light_value"] = current_light_level;
    json_doc["potentiometer_value"] = current_potentiometer;
    json_doc["led_brightness"] = led_brightness;
    json_doc["auto_mode"] = auto_mode;
    json_doc["roof1_servo_angle"] = roofServo1.read(); 
    json_doc["roof2_servo_angle"] = roofServo2.read();
    json_doc["relay_active"] = relay_active;
    json_doc["uptime"] = millis() - system_start_time;
    String message;
    serializeJson(json_doc, message);
    client.publish(MQTT_TOPIC_SENSOR, message.c_str());
}

void publishSystemStatus() {
    if (!client.connected()) {
        return;
    }
    json_doc.clear();
    json_doc["device_id"] = "light_control_board";
    json_doc["status"] = "online";
    json_doc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
    json_doc["mqtt_connected"] = mqtt_connected;
    json_doc["auto_mode"] = auto_mode;
    json_doc["system_uptime"] = millis() - system_start_time;
    json_doc["firmware_version"] = "1.0.0";
    json_doc["timestamp"] = millis();
    String message;
    serializeJson(json_doc, message);
    client.publish(MQTT_TOPIC_STATUS, message.c_str());
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.println("MQTT message received: " + String(topic));
    
    payload[length] = '\0';
    String message = String((char*)payload);
    
    DeserializationError error = deserializeJson(json_doc, message);
    if (error) {
        Serial.println("JSON parsing failed: " + String(error.c_str()));
        return;
    }

    String command = json_doc["command"];
    
    if (command == "set_auto_mode") {
        bool new_auto_mode = json_doc["auto_mode"];
        auto_mode = new_auto_mode;
        Serial.println("Auto mode set to: " + String(auto_mode ? "AUTO" : "MANUAL"));
        updateLCD();
        
    } else if (command == "set_led_brightness") {
        int brightness = json_doc["brightness"];
        setLEDBrightness(brightness);
        updateLCD();
        
    } else if (command == "control_relay") {
        bool activate = json_doc["activate"];
        if (activate) {
            activateRelay();
        } else {
            deactivateRelay();
        }
        updateLCD();
        
    } else if (command == "control_roof_servo") {
        int angle = json_doc["angle"];
        activateRoofServo(angle);
        updateLCD();
        
    } /*else if (command == "control_valve_servo") {
        int angle = json_doc["angle"];
        //activateValveServo(angle);
        updateLCD();
      }*/
        
     else if (command == "get_status") {
        publishSystemStatus();
        publishSensorData();
    }
}

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
    
    while (!client.connected()) {
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
    Serial.println("=== ESP32 Light Sensor Control Board ===");
    Serial.println("Author: Huynh Quoc Huy");
    Serial.println("Starting system initialization...");
    
    // Initialize system timing
    system_start_time = millis();
    system_initialized = false;
    
    // Initialize hardware pins
    Serial.println("Initializing hardware pins...");
    
    // LED setup 
    pinMode(LED_PWM_PIN, OUTPUT);
    
    // Servo setup: Sử dụng thư viện ESP32Servo.h
    Serial.println("Attaching Servos...");
    // Hàm attach() tự động cấu hình PWM LEDC
    roofServo1.attach(ROOF_SERVO_PIN1); 
    roofServo2.attach(ROOF_SERVO_PIN2);

    // Digital pins
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BUTTON_1_PIN, INPUT_PULLUP);
    pinMode(BUTTON_2_PIN, INPUT_PULLUP);
    pinMode(BUTTON_3_PIN, INPUT_PULLUP);
    
    // Initialize servos
    Serial.println("Setting initial servo positions...");
    activateRoofServo(0); 
    //deactivateValveServo();
    
    // Initialize LCD
    Serial.println("Initializing LCD display...");
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Smart Garden");
    lcd.setCursor(0, 1);
    lcd.print("Light Control v1.0");
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
    current_light_level = readLightSensor();
    current_potentiometer = readPotentiometer();
    
    // Initialize LED and relay
    setLEDBrightness(0);
    deactivateRelay();
    
    // Final system status
    system_initialized = true;
    updateLCD();
    
    Serial.println();
    Serial.println("=== System initialization complete! ===");
    Serial.println("Light Sensor Control Board Ready");
    Serial.println("WiFi: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected"));
    Serial.println("MQTT: " + String(client.connected() ? "Connected" : "Disconnected"));
    Serial.println("Auto Mode: " + String(auto_mode ? "Enabled" : "Disabled"));
    Serial.println("Light Level: " + String(current_light_level));
    Serial.println();
}

void loop() {
    unsigned long current_time = millis();
    
    // Check connections
    checkWiFiConnection();
    checkMQTTConnection();
    
    // Check automation
    checkAutomation();
    
    // Publish sensor data periodically
    if (current_time - last_sensor_read >= 5000) { // Every 5 seconds
        readLightSensor();
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
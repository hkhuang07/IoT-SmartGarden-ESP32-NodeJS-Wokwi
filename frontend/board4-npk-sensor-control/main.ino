/*
 * ESP32 NPK Nutrient Control Board - REAL-TIME SERVO UPDATE
 * C++/Arduino Version for Smart Garden System
 * Fix: Servo updates in REAL-TIME based on current NPK values
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
const char* MQTT_BROKER = "52.58.31.240"; 

const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "ESP_NPK_CONTROL_01";
const char* MQTT_TOPIC_SENSOR = "garden/sensor/npk_data";
const char* MQTT_TOPIC_STATUS = "garden/status/npk_board";
const char* MQTT_TOPIC_ALERTS = "garden/alerts/npk_board";
const char* MQTT_TOPIC_COMMANDS = "garden/control/npk_commands";

// Hardware Pin Definitions
const int N_PIN = 34;
const int P_PIN = 35;
const int K_PIN = 32;
const int N_SERVO_PIN = 4;
const int P_SERVO_PIN = 5;
const int K_SERVO_PIN = 12;
const int LED_PIN = 2;
const int BUTTON_PIN = 25;
const int POTENTIOMETER_PIN = 33;
const int LCD_SDA_PIN = 21;
const int LCD_SCL_PIN = 22;
const int LCD_I2C_ADDRESS = 0x27;

// NPK Thresholds - ADJUSTED FOR ACTUAL SENSOR RANGE
// Sensor outputs: N=50-150ppm, P=20-100ppm, K=50-150ppm
const float N_MIN = 60.0;        // Below 60: CRITICAL (180°)
const float N_OPTIMAL = 100.0;   // 60-100: LOW (90°), Above 100: OPTIMAL (0°)
const float N_MAX = 150.0;
const float P_MIN = 30.0;        // Below 30: CRITICAL (180°)
const float P_OPTIMAL = 60.0;    // 30-60: LOW (90°), Above 60: OPTIMAL (0°)
const float P_MAX = 100.0;
const float K_MIN = 60.0;        // Below 60: CRITICAL (180°)
const float K_OPTIMAL = 100.0;   // 60-100: LOW (90°), Above 100: OPTIMAL (0°)
const float K_MAX = 150.0;

// ADC Configuration
const int ADC_RESOLUTION = 4096;
const float ADC_VOLTAGE = 3.3;

// NPK Calibration Values
float n_calibration = 1.0;
float p_calibration = 1.0;
float k_calibration = 1.0;
const float PPM_SCALING_FACTOR = 50.0;

// Timing Configuration
const unsigned long AUTOMATION_INTERVAL = 5000;  // Check every 5s for responsiveness
const unsigned long STATUS_INTERVAL = 10000;
const unsigned long WIFI_TIMEOUT = 20000;
const unsigned long MQTT_TIMEOUT = 15000;
const unsigned long BUTTON_DEBOUNCE = 200;

// Servo Configuration
const int SERVO_OPEN_ANGLE = 180;
const int SERVO_MID_ANGLE = 90;
const int SERVO_CLOSED_ANGLE = 0;
const int SERVO_MIN_PULSE_WIDTH = 780;
const int SERVO_MAX_PULSE_WIDTH = 2250;

// System Variables
bool system_initialized = false;
bool mqtt_connected = false;
bool auto_mode = true;
bool n_servo_active = false;
bool p_servo_active = false;
bool k_servo_active = false;
bool led_active = false;

// Timing Variables
unsigned long last_automation_check = 0;
unsigned long last_status_publish = 0;
unsigned long last_sensor_read = 0;
unsigned long last_wifi_check = 0;
unsigned long last_mqtt_check = 0;
unsigned long last_button_check = 0;
unsigned long system_start_time = 0;

// Sensor Variables
float current_nitrogen = 0.0;
float current_phosphorus = 0.0;
float current_potassium = 0.0;
int current_potentiometer = 0;
int current_button_state = HIGH;

// Button State Variables
bool button_last_state = HIGH;
bool button_current_state = HIGH;

// PWM Variables
int led_brightness = 0;

// Servo state tracking - REMOVED state strings, only track angles
int current_nservo_angle = -1;
int current_pservo_angle = -1;
int current_kservo_angle = -1;

// Global Objects
WiFiClient espClient;
PubSubClient client(espClient);
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 16, 2);
DynamicJsonDocument json_doc(512);

// Servo Objects
Servo nServo;
Servo pServo;
Servo kServo;

// Enum for nutrient types
enum NutrientType { NITROGEN, PHOSPHORUS, POTASSIUM };

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
    lcd.print("N:");
    lcd.print(current_nitrogen, 0);
    lcd.print(" P:");
    lcd.print(current_phosphorus, 0);
    lcd.print(" K:");
    lcd.print(current_potassium, 0);
    
    lcd.setCursor(0, 1);
    lcd.print(auto_mode ? "AUTO" : "MAN");
    lcd.print(" V:");
    lcd.print(nServo.read());
    lcd.print(",");
    lcd.print(pServo.read());
    lcd.print(",");
    lcd.print(kServo.read());
}

// === UNIFIED SERVO CONTROL ===
void setServoAngle(NutrientType type, int angle) {
    angle = constrain(angle, 0, 180);
    
    Servo* servo;
    bool* active_flag;
    int* current_angle;
    String name;
    
    switch(type) {
        case NITROGEN:
            servo = &nServo;
            active_flag = &n_servo_active;
            current_angle = &current_nservo_angle;
            name = "N";
            break;
        case PHOSPHORUS:
            servo = &pServo;
            active_flag = &p_servo_active;
            current_angle = &current_pservo_angle;
            name = "P";
            break;
        case POTASSIUM:
            servo = &kServo;
            active_flag = &k_servo_active;
            current_angle = &current_kservo_angle;
            name = "K";
            break;
    }
    
    // Only update if angle actually changed
    if (*current_angle != angle) {
        servo->write(angle);
        *active_flag = (angle > 0);
        *current_angle = angle;
        Serial.println("🔧 " + name + " servo: " + String(*current_angle) + "° → " + String(angle) + "°");
    }
}

void deactivateAllServos() {
    setServoAngle(NITROGEN, 0);
    setServoAngle(PHOSPHORUS, 0);
    setServoAngle(POTASSIUM, 0);
}

// === LED CONTROL ===
void setLEDBrightness(int brightness) { 
    brightness = constrain(brightness, 0, 255);
    if (led_brightness != brightness) {
        led_brightness = brightness;
        analogWrite(LED_PIN, brightness);
        led_active = (brightness > 0);
    }
}

void deactivateLED() {
    setLEDBrightness(0);
}

// Sensor Reading Functions
float readNitrogen() {
    int raw_value = analogRead(N_PIN);
    float voltage = (raw_value * ADC_VOLTAGE) / ADC_RESOLUTION;
    current_nitrogen = voltage * n_calibration * PPM_SCALING_FACTOR;
    return current_nitrogen;
}

float readPhosphorus() {
    int raw_value = analogRead(P_PIN);
    float voltage = (raw_value * ADC_VOLTAGE) / ADC_RESOLUTION;
    current_phosphorus = voltage * p_calibration * PPM_SCALING_FACTOR;
    return current_phosphorus;
}

float readPotassium() {
    int raw_value = analogRead(K_PIN);
    float voltage = (raw_value * ADC_VOLTAGE) / ADC_RESOLUTION;
    current_potassium = voltage * k_calibration * PPM_SCALING_FACTOR;
    return current_potassium;
}

int readPotentiometer() {
    int reading = analogRead(POTENTIOMETER_PIN);
    current_potentiometer = reading;
    return reading;
}

// === CALCULATE SERVO ANGLE FROM NPK VALUE ===
int calculateServoAngle(float value, float min_threshold, float optimal_threshold) {
    // Below minimum: OPEN (180°)
    if (value < min_threshold) {
        return SERVO_OPEN_ANGLE;
    }
    // Below optimal: MID (90°)
    else if (value < optimal_threshold) {
        return SERVO_MID_ANGLE;
    }
    // Optimal or above: CLOSED (0°)
    else {
        return SERVO_CLOSED_ANGLE;
    }
}

// === GET STATUS STRING ===
String getStatusString(float value, float min_threshold, float optimal_threshold) {
    if (value < min_threshold) return "CRITICAL";
    else if (value < optimal_threshold) return "LOW";
    else return "OPTIMAL";
}

// === REAL-TIME AUTO CONTROL LOGIC ===
void auto_control_logic() {
    // Read all sensors
    float n_value = readNitrogen();
    float p_value = readPhosphorus();
    float k_value = readPotassium();
    
    Serial.println("\n╔════════════════════════════════════╗");
    Serial.println("║   NPK REAL-TIME CONTROL UPDATE    ║");
    Serial.println("╚════════════════════════════════════╝");
    
    // Calculate target angles based on CURRENT values
    int n_target_angle = calculateServoAngle(n_value, N_MIN, N_OPTIMAL);
    int p_target_angle = calculateServoAngle(p_value, P_MIN, P_OPTIMAL);
    int k_target_angle = calculateServoAngle(k_value, K_MIN, K_OPTIMAL);
    
    // Get status strings
    String n_status = getStatusString(n_value, N_MIN, N_OPTIMAL);
    String p_status = getStatusString(p_value, P_MIN, P_OPTIMAL);
    String k_status = getStatusString(k_value, K_MIN, K_OPTIMAL);
    
    // Print current readings and targets
    Serial.println("┌─────────────────────────────────┐");
    Serial.println("│ NITROGEN (N)                    │");
    Serial.println("├─────────────────────────────────┤");
    Serial.print("│ Value: "); Serial.print(n_value, 1); Serial.println(" ppm");
    Serial.print("│ Status: "); Serial.println(n_status);
    Serial.print("│ Target Angle: "); Serial.print(n_target_angle); Serial.println("°");
    Serial.println("└─────────────────────────────────┘");
    
    Serial.println("┌─────────────────────────────────┐");
    Serial.println("│ PHOSPHORUS (P)                  │");
    Serial.println("├─────────────────────────────────┤");
    Serial.print("│ Value: "); Serial.print(p_value, 1); Serial.println(" ppm");
    Serial.print("│ Status: "); Serial.println(p_status);
    Serial.print("│ Target Angle: "); Serial.print(p_target_angle); Serial.println("°");
    Serial.println("└─────────────────────────────────┘");
    
    Serial.println("┌─────────────────────────────────┐");
    Serial.println("│ POTASSIUM (K)                   │");
    Serial.println("├─────────────────────────────────┤");
    Serial.print("│ Value: "); Serial.print(k_value, 1); Serial.println(" ppm");
    Serial.print("│ Status: "); Serial.println(k_status);
    Serial.print("│ Target Angle: "); Serial.print(k_target_angle); Serial.println("°");
    Serial.println("└─────────────────────────────────┘");
    
    // Update servos to target angles (only if changed)
    setServoAngle(NITROGEN, n_target_angle);
    setServoAngle(PHOSPHORUS, p_target_angle);
    setServoAngle(POTASSIUM, k_target_angle);
    
    // Update LED based on worst nutrient status
    int max_brightness = 0;
    if (n_target_angle > max_brightness) max_brightness = n_target_angle;
    if (p_target_angle > max_brightness) max_brightness = p_target_angle;
    if (k_target_angle > max_brightness) max_brightness = k_target_angle;
    
    // Map servo angle (0-180) to LED brightness (0-255)
    int led_value = map(max_brightness, 0, 180, 0, 255);
    setLEDBrightness(led_value);
    
    Serial.println("\n✅ Servo positions updated");
    Serial.println("═══════════════════════════════════\n");
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
    
    Serial.println("🔄 Running automation check...");
    auto_control_logic();
    updateLCD();
}

void publishSensorData() {
    if (!client.connected()) {
        Serial.println("⚠️  Cannot publish sensor data - MQTT disconnected");
        return;
    }

    json_doc.clear();
    
    // Primary sensor data (matching Node.js expected format)
    json_doc["device_id"] = "npk_control_board";
    json_doc["nitrogen"] = current_nitrogen;
    json_doc["phosphorus"] = current_phosphorus;
    json_doc["potassium"] = current_potassium;
    
    // Additional metadata
    json_doc["nitrogen_ppm"] = current_nitrogen;
    json_doc["phosphorus_ppm"] = current_phosphorus;
    json_doc["potassium_ppm"] = current_potassium;
    json_doc["potentiometer_value"] = current_potentiometer;
    json_doc["auto_mode"] = auto_mode;
    json_doc["n_servo_angle"] = nServo.read();
    json_doc["p_servo_angle"] = pServo.read();
    json_doc["k_servo_angle"] = kServo.read();
    json_doc["led_brightness"] = led_brightness;
    json_doc["uptime"] = millis() - system_start_time;
    json_doc["timestamp"] = millis();
    
    String message;
    serializeJson(json_doc, message);
    
    // Publish with QoS 1 for guaranteed delivery
    bool published = client.publish(MQTT_TOPIC_SENSOR, message.c_str(), false);
    
    if (published) {
        Serial.println("📡 NPK data published to MQTT");
        Serial.println("   Topic: " + String(MQTT_TOPIC_SENSOR));
        Serial.println("   Payload: " + message);
    } else {
        Serial.println("❌ Failed to publish NPK data");
    }
}

void publishSystemStatus() {
    if (!client.connected()) {
        Serial.println("⚠️  Cannot publish status - MQTT disconnected");
        return;
    }

    json_doc.clear();
    json_doc["device_id"] = "npk_control_board";
    json_doc["status"] = "online";
    json_doc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
    json_doc["mqtt_connected"] = mqtt_connected;
    json_doc["auto_mode"] = auto_mode;
    
    bool any_servo_open = n_servo_active || p_servo_active || k_servo_active;
    json_doc["dosing_state"] = any_servo_open ? "ACTIVE" : "INACTIVE"; 
    
    json_doc["system_uptime"] = millis() - system_start_time;
    json_doc["firmware_version"] = "1.0.8_mqtt_fix";
    json_doc["timestamp"] = millis();
    
    String message;
    serializeJson(json_doc, message);
    
    bool published = client.publish(MQTT_TOPIC_STATUS, message.c_str(), false);
    
    if (published) {
        Serial.println("📡 System status published");
    } else {
        Serial.println("❌ Failed to publish system status");
    }
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
        bool new_auto_mode = json_doc["auto_mode"];
        auto_mode = new_auto_mode;
        Serial.println("🔧 Auto mode: " + String(auto_mode ? "ON" : "OFF"));
        if (!auto_mode) {
            deactivateAllServos();
            deactivateLED();
        }
        updateLCD();
        
    } else if (command == "control_nitrogen_servo") {
        if (!auto_mode) {
            int angle = json_doc["angle"];
            setServoAngle(NITROGEN, angle);
            updateLCD();
        }
        
    } else if (command == "control_phosphorus_servo") {
        if (!auto_mode) {
            int angle = json_doc["angle"];
            setServoAngle(PHOSPHORUS, angle);
            updateLCD();
        }
        
    } else if (command == "control_potassium_servo") {
        if (!auto_mode) {
            int angle = json_doc["angle"];
            setServoAngle(POTASSIUM, angle);
            updateLCD();
        }
        
    } else if (command == "calibrate_npk") {
        float pot_factor = (float)readPotentiometer() / 2048.0; 
        n_calibration = pot_factor;
        p_calibration = pot_factor;
        k_calibration = pot_factor;
        Serial.println("🔧 Calibration factor: " + String(pot_factor, 3));
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
    while (WiFi.status() != WL_CONNECTED && (millis() - start_time) < WIFI_TIMEOUT) {
        delay(500);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        return true;
    } else {
        Serial.println("\nWiFi failed!");
        return false;
    }
}

bool connectMQTT() {
    client.setServer(MQTT_BROKER, MQTT_PORT);
    client.setCallback(mqttCallback);
    client.setBufferSize(512); // Ensure enough buffer for JSON
    
    Serial.println("Connecting to MQTT...");
    Serial.println("Broker: " + String(MQTT_BROKER) + ":" + String(MQTT_PORT));
    
    int attempts = 0;
    while (!client.connected() && attempts < 3) {
        Serial.print("MQTT attempt ");
        Serial.print(attempts + 1);
        Serial.print("...");
        
        if (client.connect(MQTT_CLIENT_ID)) {
            Serial.println("connected");
            mqtt_connected = true;
            
            // Subscribe to command topic
            bool subscribed = client.subscribe(MQTT_TOPIC_COMMANDS);
            if (subscribed) {
                Serial.println("✓ Subscribed to: " + String(MQTT_TOPIC_COMMANDS));
            } else {
                Serial.println("✗ Failed to subscribe to commands");
            }
            
            // Publish initial status
            publishSystemStatus();
            delay(100); // Small delay between publishes
            publishSensorData();
            
            Serial.println("✓ MQTT fully initialized");
            return true;
        } else {
            Serial.print("failed, rc=");
            Serial.println(client.state());
            attempts++;
            delay(2000);
        }
    }
    return false;
}

void checkWiFiConnection() {
    unsigned long current_time = millis();
    if (current_time - last_wifi_check < 30000) { return; }
    last_wifi_check = current_time;
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi lost, reconnecting...");
        connectWiFi();
    }
}

void checkMQTTConnection() {
    unsigned long current_time = millis();
    if (current_time - last_mqtt_check < 10000) { return; }
    last_mqtt_check = current_time;
    if (!client.connected() && WiFi.status() == WL_CONNECTED) {
        Serial.println("MQTT lost, reconnecting...");
        mqtt_connected = false;
        connectMQTT();
    }
    client.loop();
}

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("╔═══════════════════════════════════════╗");
    Serial.println("║  ESP32 NPK Control - Real-Time v1.0.8║");
    Serial.println("╚═══════════════════════════════════════╝");
    
    system_start_time = millis();
    system_initialized = false;
    
    // Initialize pins
    pinMode(N_PIN, INPUT);
    pinMode(P_PIN, INPUT);
    pinMode(K_PIN, INPUT);
    pinMode(POTENTIOMETER_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // Initialize servos
    Serial.println("→ Initializing servos...");
    nServo.attach(N_SERVO_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
    pServo.attach(P_SERVO_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
    kServo.attach(K_SERVO_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
    
    // Test pulse
    Serial.println("→ Servo test (90°)...");
    nServo.write(90);
    pServo.write(90);
    kServo.write(90);
    delay(1000);
    
    // Close all
    deactivateAllServos();
    Serial.println("→ Servos ready (0°)");
    
    // Initialize LCD
    Serial.println("→ Initializing LCD...");
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("NPK Control");
    lcd.setCursor(0, 1);
    lcd.print("v1.0.7 RealTime");
    delay(2000);
    
    // WiFi
    Serial.println("→ Connecting WiFi...");
    connectWiFi();
    
    // MQTT
    Serial.println("→ Connecting MQTT...");
    connectMQTT();
    
    // Initial sensor read
    Serial.println("→ Reading sensors...");
    readNitrogen();
    readPhosphorus();
    readPotassium();
    readPotentiometer();
    
    deactivateLED();
    system_initialized = true;
    updateLCD();
    
    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║       SYSTEM READY                    ║");
    Serial.println("╚═══════════════════════════════════════╝");
    Serial.println("WiFi: " + String(WiFi.status() == WL_CONNECTED ? "✓" : "✗"));
    Serial.println("MQTT: " + String(client.connected() ? "✓" : "✗"));
    Serial.println("Auto: " + String(auto_mode ? "ON" : "OFF"));
    Serial.printf("NPK: N=%.1f P=%.1f K=%.1f ppm\n", current_nitrogen, current_phosphorus, current_potassium);
    Serial.println();
}

void loop() {
    unsigned long current_time = millis();
    
    // Maintain connections
    checkWiFiConnection();
    checkMQTTConnection();

    // Run automation
    checkAutomation();
    
    // Publish sensor data
    if (current_time - last_sensor_read >= 5000) {
        readNitrogen();
        readPhosphorus();
        readPotassium();
        readPotentiometer();
        
        // Ensure MQTT is connected before publishing
        if (client.connected()) {
            publishSensorData();
        } else {
            Serial.println("⚠️  Skipping sensor publish - MQTT disconnected");
        }
        
        last_sensor_read = current_time;
    }
    
    // Publish status
    if (current_time - last_status_publish >= STATUS_INTERVAL) {
        publishSystemStatus();
        last_status_publish = current_time;
    }
    
    // Update LCD
    static unsigned long last_lcd_update = 0;
    if (current_time - last_lcd_update >= 3000) {
        updateLCD();
        last_lcd_update = current_time;
    }
    
    delay(10);
}
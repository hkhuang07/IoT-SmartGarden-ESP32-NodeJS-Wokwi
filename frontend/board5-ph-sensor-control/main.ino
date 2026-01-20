/*
 * ESP32 pH Control Board 
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
const char* MQTT_CLIENT_ID = "ESP_PH_CONTROL_01";
const char* MQTT_TOPIC_SENSOR = "garden/sensor/ph_data";
const char* MQTT_TOPIC_STATUS = "garden/status/ph_board";
const char* MQTT_TOPIC_COMMANDS = "garden/control/ph_commands";

// Hardware Pin Definitions
const int PH_ANALOG_PIN = 34;
const int ACID_SERVO_PIN = 4;
const int ALKALINE_SERVO_PIN = 5;
const int STATUS_LED_PIN = 2;
const int ACID_LED_PIN = 23;
const int ALKALINE_LED_PIN = 26;
const int OPTIMAL_LED_PIN = 27;
const int PH_CALIBRATION_POT_PIN = 32;

// pH Thresholds
const float PH_STRONG_ACID_MAX = 4.0;
const float PH_ACID_MAX = 6.5;
const float PH_OPTIMAL_MIN = 6.5;
const float PH_OPTIMAL_MAX = 7.5;
const float PH_ALKALINE_MIN = 7.5;
const float PH_STRONG_ALKALINE_MIN = 10.0;

// Servo Configuration
const int SERVO_OPEN_ANGLE = 180;
const int SERVO_MID_ANGLE = 90;
const int SERVO_CLOSED_ANGLE = 0;

// Timing Configuration
const unsigned long PROCESS_INTERVAL = 3000;  // Xử lý sensor mỗi 3s
const unsigned long STATUS_INTERVAL = 10000;

// System Variables
float current_ph = 7.0;
float ph_calibration = 0.0;
unsigned long last_process = 0;
unsigned long last_status_publish = 0;
unsigned long system_start_time = 0;

// Global Objects
WiFiClient espClient;
PubSubClient client(espClient);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo acidServo;
Servo alkalineServo;

// === FUNCTIONS ===

String getPHStatusString(float ph) {
    if (ph < PH_STRONG_ACID_MAX) return "STR_ACID";
    else if (ph < PH_ACID_MAX) return "ACIDIC";
    else if (ph >= PH_OPTIMAL_MIN && ph <= PH_OPTIMAL_MAX) return "OPTIMAL";
    else if (ph <= PH_STRONG_ALKALINE_MIN) return "ALKALINE";
    else return "STR_ALKA";
}

void setLEDs(int acid, int alkaline, int optimal) {
    digitalWrite(ACID_LED_PIN, acid);
    digitalWrite(ALKALINE_LED_PIN, alkaline);
    digitalWrite(OPTIMAL_LED_PIN, optimal);
}

void setServoAngle(int type, int angle) {
    if (type == 0) acidServo.write(angle); // 0 = Acid
    else alkalineServo.write(angle);       // 1 = Alkaline
}

float readSensors() {
    // Read pH
    int raw_ph = analogRead(PH_ANALOG_PIN);
    float ph_mapped = (map(raw_ph, 0, 4095, 0, 14000) / 1000.0);
    
    // Read Calibration Pot
    int raw_pot = analogRead(PH_CALIBRATION_POT_PIN);
    ph_calibration = (map(raw_pot, 0, 4095, -200, 200) / 100.0);
    
    current_ph = constrain(ph_mapped + ph_calibration, 0.0, 14.0);
    return current_ph;
}

void updateLCD(float ph) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("pH:"); lcd.print(ph, 2);
    lcd.setCursor(9, 0);
    lcd.print(getPHStatusString(ph));
    
    lcd.setCursor(0, 1);
    lcd.print("A:"); lcd.print(acidServo.read());
    lcd.print(" K:"); lcd.print(alkalineServo.read());
}

void reactive_control_logic() {
    float ph = readSensors();
    int acid_target = 0;
    int alkaline_target = 0;

    // Logic phản ứng dựa trên ngưỡng
    if (ph < PH_STRONG_ACID_MAX) {
        alkaline_target = SERVO_OPEN_ANGLE;
        setLEDs(LOW, HIGH, LOW);
    } else if (ph < PH_ACID_MAX) {
        alkaline_target = SERVO_MID_ANGLE;
        setLEDs(LOW, HIGH, LOW);
    } else if (ph > PH_STRONG_ALKALINE_MIN) {
        acid_target = SERVO_OPEN_ANGLE;
        setLEDs(HIGH, LOW, LOW);
    } else if (ph > PH_ALKALINE_MIN) {
        acid_target = SERVO_MID_ANGLE;
        setLEDs(HIGH, LOW, LOW);
    } else {
        setLEDs(LOW, LOW, HIGH); // Optimal
    }

    setServoAngle(0, acid_target);
    setServoAngle(1, alkaline_target);
    
    updateLCD(ph);
    
    Serial.printf("Sensor Update -> pH: %.2f | Acid: %d | Alkali: %d\n", ph, acid_target, alkaline_target);
}

void publishData() {
    if (!client.connected()) return;
    StaticJsonDocument<256> doc;
    doc["device_id"] = "ph_control_board";
    doc["ph"] = current_ph;
    doc["status"] = getPHStatusString(current_ph);
    doc["acid_deg"] = acidServo.read();
    doc["alkaline_deg"] = alkalineServo.read();
    
    char buffer[256];
    serializeJson(doc, buffer);
    client.publish(MQTT_TOPIC_SENSOR, buffer);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    StaticJsonDocument<256> doc;
    deserializeJson(doc, payload, length);
    String cmd = doc["command"];
    
    // Vẫn giữ lại khả năng hiệu chuẩn từ xa qua MQTT
    if (cmd == "calibrate") {
        float offset = doc["offset"];
        ph_calibration = offset;
        Serial.println("Manual Calibration Update");
    }
}

void setup() {
    Serial.begin(115200);
    system_start_time = millis();

    pinMode(ACID_LED_PIN, OUTPUT);
    pinMode(ALKALINE_LED_PIN, OUTPUT);
    pinMode(OPTIMAL_LED_PIN, OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);

    acidServo.attach(ACID_SERVO_PIN);
    alkalineServo.attach(ALKALINE_SERVO_PIN);
    
    // Khởi tạo servo về 0
    acidServo.write(0);
    alkalineServo.write(0);

    lcd.init();
    lcd.backlight();
    lcd.print("pH Board System");
    lcd.setCursor(0,1);
    lcd.print("Reactive Mode");

    // Connect WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nWiFi Connected");

    // Connect MQTT
    client.setServer(MQTT_BROKER, MQTT_PORT);
    client.setCallback(mqttCallback);
    while (!client.connected()) {
        if (client.connect(MQTT_CLIENT_ID)) {
            client.subscribe(MQTT_TOPIC_COMMANDS);
            client.publish(MQTT_TOPIC_STATUS, "{\"status\":\"online\"}");
        } else {
            delay(5000);
        }
    }
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) ESP.restart();
    if (!client.connected()) setup(); // Reconnect
    client.loop();

    unsigned long now = millis();

    // Core Logic: Phản ứng theo sensor
    if (now - last_process >= PROCESS_INTERVAL) {
        reactive_control_logic();
        publishData();
        last_process = now;
    }

    // Nháy LED Status báo hiệu hệ thống đang sống
    digitalWrite(STATUS_LED_PIN, (now / 500) % 2); 
}
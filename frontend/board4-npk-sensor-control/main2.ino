/*
 * ESP32 NPK Nutrient Control Board - PWM STABLE
 * C++/Arduino Version for Smart Garden System
 * FIX: Áp dụng dải Pulse Width (us) ổn định 780-2250 cho cả ba Servo (N, P, K).
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
const char* MQTT_CLIENT_ID = "ESP_NPK_CONTROL_01";
const char* MQTT_TOPIC_SENSOR = "garden/sensor/npk_data";
const char* MQTT_TOPIC_STATUS = "garden/status/npk_board";
const char* MQTT_TOPIC_ALERTS = "garden/alerts/npk_board";
const char* MQTT_TOPIC_COMMANDS = "garden/control/npk_commands";

// Hardware Pin Definitions
const int N_PIN = 34; // Nitrogen Sensor
const int P_PIN = 35; // Phosphorus Sensor
const int K_PIN = 32; // Potassium Sensor
const int N_SERVO_PIN = 4;
const int P_SERVO_PIN = 5;
const int K_SERVO_PIN = 12; // GPIO0
const int LED_PIN = 2;
const int BUTTON_PIN = 25;
const int POTENTIOMETER_PIN = 33;
const int LCD_SDA_PIN = 21;
const int LCD_SCL_PIN = 22;
const int LCD_I2C_ADDRESS = 0x27;

// NPK Thresholds 
const float N_MIN = 20.0;
const float N_OPTIMAL= 35.0;
const float N_MAX = 50.0;
const float P_MIN = 10.0;
const float P_OPTIMAL= 20.0;
const float P_MAX = 30.0;
const float K_MIN = 30.0;
const float K_OPTIMAL= 45.0;
const float K_MAX = 60.0;

// ADC Configuration
const int ADC_RESOLUTION = 4096;
const float ADC_VOLTAGE = 3.3;

// NPK Calibration Values
float n_calibration = 1.0;
float p_calibration = 1.0;
float k_calibration = 1.0;

const float PPM_SCALING_FACTOR = 50.0; 

// Timing Configuration
const unsigned long AUTOMATION_INTERVAL = 15000;
const unsigned long STATUS_INTERVAL = 10000;
const unsigned long WIFI_TIMEOUT = 20000;
const unsigned long MQTT_TIMEOUT = 15000;
const unsigned long BUTTON_DEBOUNCE = 200;

// Servo Configuration
const int SERVO_OPEN_ANGLE = 180;
const int SERVO_MID_ANGLE = 90;
const int SERVO_CLOSED_ANGLE = 0;

// HẰNG SỐ PWM TỐI ƯU (Dựa trên Soil Moisture Board: 780us - 2250us)
const int SERVO_MIN_PULSE_WIDTH = 780;  // micro seconds cho 0 độ
const int SERVO_MAX_PULSE_WIDTH = 2250; // micro seconds cho 180 độ

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

// Servo state tracking
int current_nservo_angle = -1;
int current_pservo_angle = -1;
int current_kservo_angle = -1;
String current_nstate = "INITIALIZING";
String current_pstate = "INITIALIZING";
String current_kstate = "INITIALIZING";

// Global Objects
WiFiClient espClient;
PubSubClient client(espClient);
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 16, 2);
DynamicJsonDocument json_doc(512);

// Servo Objects
Servo nServo;
Servo pServo;
Servo kServo;

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
    lcd.print(" DOSING:");
    
    // Hiển thị trạng thái Servo
    if (nServo.read() > SERVO_CLOSED_ANGLE || pServo.read() > SERVO_CLOSED_ANGLE || kServo.read() > SERVO_CLOSED_ANGLE) {
        lcd.print("ON");
    } else {
        lcd.print("OFF");
    }
}

// === SERVO CONTROL ===
void activeNServo(int angle) {
    angle = constrain(angle, 0, 180);   //giới hạn giá trị góc xoay
    nServo.write(angle);                //xoay góc với giá trị góc angle
    n_servo_active = (angle != 0);      //giá trị valve servo đang hoạt động
    current_nservo_angle = angle;       //góc hiện tại bằng angle
    Serial.println("🧪 Nitrogen servo: " + String(angle) + "°");
    Serial.println("🧪 Nitrogen servo active: " + String(n_servo_active) + "°");
    Serial.println("🧪 Nitrogen servo angle: " + String(current_nservo_angle) + "°");
}

void deactivateNServo() {
    if (n_servo_active) {
        nServo.write(0);                //xoay góc với giá trị góc 0
        n_servo_active = false;         //giá trị valve servo đang tắt
        current_nservo_angle = 0;      //góc hiện tại bằng 0
        Serial.println("🧪 Nitrogen servo DEACTIVATED: " + String(angle) + "°");
        Serial.println("🧪 Nitrogen servo active: " + String(n_servo_active) + "°");
        Serial.println("🧪 Nitrogen servo: " + String(current_nservo_angle) + "°");
    }
}

// === SERVO CONTROL ===
void activePServo(int angle) {
    angle = constrain(angle, 0, 180);   //giới hạn giá trị góc xoay
    pServo.write(angle);                //xoay góc với giá trị góc angle
    p_servo_active = (angle != 0);      //giá trị valve servo đang hoạt động
    current_pservo_angle = angle;       //góc hiện tại bằng angle
    Serial.println("🧪 Phosphorus servo: " + String(angle) + "°");
    Serial.println("🧪 Phosphorus servo active: " + String(p_servo_active) + "°");
    Serial.println("🧪 Phosphorus servo angle: " + String(current_pservo_angle) + "°");
}

void deactivatePServo() {
    if (p_servo_active) {
        pServo.write(0);                //xoay góc với giá trị góc 0
        p_servo_active = false;         //giá trị valve servo đang tắt
        current_pservo_angle = 0;      //góc hiện tại bằng 0
        Serial.println("🧪 Phosphorus servo DEACTIVATED: " + String(angle) + "°");
        Serial.println("🧪 Phosphorus servo active: " + String(p_servo_active) + "°");
        Serial.println("🧪 Phosphorus servo: " + String(current_pservo_angle) + "°");
    }
}

// === SERVO CONTROL ===
void activeKServo(int angle) {
    angle = constrain(angle, 0, 180);   //giới hạn giá trị góc xoay
    kServo.write(angle);                //xoay góc với giá trị góc angle
    k_servo_active = (angle != 0);      //giá trị valve servo đang hoạt động
    current_kservo_angle = angle;       //góc hiện tại bằng angle
    Serial.println("🧪 Potassium servo: " + String(angle) + "°");
    Serial.println("🧪 Potassium servo active: " + String(k_servo_active) + "°");
    Serial.println("🧪 Potassium servo angle: " + String(current_kservo_angle) + "°");
}

void deactivateKServo() {
    if (k_servo_active) {
        kServo.write(0);                //xoay góc với giá trị góc 0
        k_servo_active = false;         //giá trị valve servo đang tắt
        current_kservo_angle = 0;      //góc hiện tại bằng 0
        Serial.println("🧪 Potassium servo DEACTIVATED: " + String(angle) + "°");
        Serial.println("🧪 Potassium servo active: " + String(k_servo_active) + "°");
        Serial.println("🧪 Potassium servo: " + String(current_kservo_angle) + "°");
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

// Sensor Reading Functions
float readNitrogen() {
    int raw_value = analogRead(N_PIN);
    float voltage = (raw_value * ADC_VOLTAGE) / ADC_RESOLUTION;
    current_nitrogen = voltage * n_calibration * PPM_SCALING_FACTOR;
    Serial.print("Raw N: " + String(raw_value) + ", N: " + String(current_nitrogen, 2) + "ppm | ");
    return current_nitrogen;
}

float readPhosphorus() {
    int raw_value = analogRead(P_PIN);
    float voltage = (raw_value * ADC_VOLTAGE) / ADC_RESOLUTION;
    current_phosphorus = voltage * p_calibration * PPM_SCALING_FACTOR;
    Serial.print("Raw P: " + String(raw_value) + ", P: " + String(current_phosphorus, 2) + "ppm | ");
    return current_phosphorus;
}

float readPotassium() {
    int raw_value = analogRead(K_PIN);
    float voltage = (raw_value * ADC_VOLTAGE) / ADC_RESOLUTION;
    current_potassium = voltage * k_calibration * PPM_SCALING_FACTOR;
    Serial.println("Raw K: " + String(raw_value) + ", K: " + String(current_potassium, 2) + "ppm");
    return current_potassium;
}

int readPotentiometer() {
    int reading = analogRead(POTENTIOMETER_PIN);
    current_potentiometer = reading;
    return reading;
}

// === AUTO CONTROL LOGIC ===
void auto_control_logic() {
    current_nitrogen = readNitrogen();
    current_phosphorus = readPhosphorus();
    current_potassium = readPotassium();

    String target_nstate = ""; //Trạng thái
    String target_pstate = ""; //Trạng thái
    String target_kstate = ""; //Trạng thái
    int target_nangle = 0;     //Góc xoay 0 độ
    int target_pangle = 0;     //Góc xoay 0 độ
    int target_kangle = 0;     //Góc xoay 0 độ
    int target_brightness = 0;//Độ sáng 0

    
    if (current_nitrogen > N_MIN) {
        current_nstate = "NITROGEN LOW";               //Cập nhật trạng thái valve thành thấp       
        nServo.write(90);                              //Xoay góc 90 độ
        target_brightness = (float)225/2;              //Độ sáng 127
    } else if (current_nitrogen > N_OPTIMAL) {
        current_nstate = "NITROGEN OPTIMAL";           //Cập nhật trạng thái valve thành đủ
        nServo.write(0);                               //Xoay góc 0 độ
        target_brightness = 0;                         //Độ sáng 0
    } else {
        current_nstate = "NITROGEN CRITICAL";  
        nServo.write(180);                              //Xoay góc 0 độ
        target_brightness = 255;                        //Độ sáng 0
    }
    
    if (current_phosphorus > P_MIN) {
        current_pstate = "PHOSPHORUS LOW";               //Cập nhật trạng thái valve thành thấp       
        pServo.write(90);                              //Xoay góc 90 độ
        target_brightness = (float)225/2;              //Độ sáng 127
    } else if (current_phosphorus > P_OPTIMAL) {
        current_pstate = "PHOSPHORUS OPTIMAL";           //Cập nhật trạng thái valve thành đủ
        pServo.write(0);                               //Xoay góc 0 độ
        target_brightness = 0;                         //Độ sáng 0
    } else {
        current_pstate = "PHOSPHORUS CRITICAL";  
        pServo.write(180);                              //Xoay góc 0 độ
        target_brightness = 255;                        //Độ sáng 0
    }

    if (current_potassium > K_MIN) {
        current_kstate = "POTASSIUM LOW";               //Cập nhật trạng thái valve thành thấp       
        kServo.write(90);                              //Xoay góc 90 độ
        target_brightness = (float)225/2;              //Độ sáng 127
    } else if (current_potassium > K_OPTIMAL) {
        current_kstate = "POTASSIUM OPTIMAL";           //Cập nhật trạng thái valve thành đủ
        kServo.write(0);                               //Xoay góc 0 độ
        target_brightness = 0;                         //Độ sáng 0
    } else {
        current_kstate = "POTASSIUM CRITICAL";  
        kServo.write(180);                              //Xoay góc 0 độ
        target_brightness = 255;                        //Độ sáng 0
    }
    
    
    // Kiểm tra thay đổi
    bool state_nchanged = false
    bool state_pchanged = false
    bool state_kchanged = false

    if (target_nstate != current_nstate) {
        state_nchanged = true;
        Serial.println("🧪 State changed: " + String(current_nstate) + " → " + String(target_nstate));
    }
    if (target_pstate != current_pstate) {
        state_pchanged = true;
        Serial.println("🧪 State changed: " + String(current_pstate) + " → " + String(target_pstate));
    }
    if (target_kstate != current_kstate) {
        state_kchanged = true;
        Serial.println("🧪 State changed: " + String(current_kstate) + " → " + String(target_kstate));
    }

    bool angle_nchanged = false;
    bool angle_pchanged = false;
    bool angle_kchanged = false;

    if (target_nangle != current_nservo_angle) {
        angle_nchanged = true;
        Serial.println("🧪 Angle changed: " + String(current_nservo_angle) + "° → " + String(target_nangle) + "°");
    }
    if (target_pangle != current_pservo_angle) {
        angle_pchanged = true;
        Serial.println("🧪 Angle changed: " + String(current_pservo_angle) + "° → " + String(target_pangle) + "°");
    }
    if (target_kangle != current_kservo_angle) {
        angle_kchanged = true;
        Serial.println("🧪 Angle changed: " + String(current_kservo_angle) + "° → " + String(target_kangle) + "°");
    }

    if (state_nchanged || angle_nchanged) {
        Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        Serial.print("🧪 State: "); Serial.print(current_nstate); Serial.print(" → "); Serial.println(target_nstate);
        Serial.print("🧪 Moisture: ");  Serial.print(current_nitrogen, 1); Serial.println("ppm");
        
        String status_Nmsg = "";
        if (target_state == "OPTIMAL") {
            status_Nmsg = "NITROGEN OPTIMAL (≥60%) - Valve CLOSED";
        } else if (target_state == "LOW") {
            status_Nmsg = "NITROGEN LOW (30-60%) - Valve PARTIAL";
        } else if (target_state == "CRITICAL") {
            status_Nmsg = "NITROGEN CRITICAL (<30%) - Valve OPEN";
        }
        Serial.println("   " + status_Nmsg);
        Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        // Cập nhật servo và LED
        Serial.println("🧪 Activating Nitrogen valve servo to " + String(target_nangle) + "°")    ;
        activateValveServo(target_angle);
        Serial.println("💡 Setting LED brightness to " + String(target_brightness)) ;
        setLEDBrightness(target_brightness);
        
        // Lưu trạng thái
        Serial.println("✅ State updated to " + target_nstate);
        current_state = target_nstate;

    } else {
        Serial.println("⏸️  State: " + current_state + " (stable)");
    }

    if (state_pchanged || angle_pchanged) {
        Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        Serial.print("🧪 State: "); Serial.print(current_pstate); Serial.print(" → "); Serial.println(target_pstate);
        Serial.print("🧪 Moisture: ");  Serial.print(current_phosphorus, 1); Serial.println("ppm");
        
        String status_Pmsg = "";
        if (target_state == "OPTIMAL") {
            status_Pmsg = "PHOSPHORUS OPTIMAL (≥60%) - Valve CLOSED";
        } else if (target_state == "LOW") {
            status_Pmsg = "PHOSPHORUS LOW (30-60%) - Valve PARTIAL";
        } else if (target_state == "CRITICAL") {
            status_Pmsg = "PHOSPHORUS CRITICAL (<30%) - Valve OPEN";
        }
        Serial.println("   " + status_Pmsg);
        Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        // Cập nhật servo và LED
        Serial.println("🧪 Activating Phosphorus valve servo to " + String(target_pangle) + "°")    ;
        activateValveServo(target_pangle);
        Serial.println("💡 Setting LED brightness to " + String(target_brightness)) ;
        setLEDBrightness(target_brightness);
        
        // Lưu trạng thái
        Serial.println("✅ State updated to " + target_pstate);
        current_state = target_pstate;
    } else {
        Serial.println("⏸️  State: " + current_state + " (stable)");
    }

    if (state_kchanged || angle_kchanged) {
        Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        Serial.print("🧪 State: "); Serial.print(current_kstate); Serial.print(" → "); Serial.println(target_kstate);
        Serial.print("🧪 Moisture: ");  Serial.print(current_potassium, 1); Serial.println("ppm");
        
        String status_Kmsg = "";
        if (target_state == "OPTIMAL") {
            status_Kmsg = "POTASSIUM OPTIMAL (≥60%) - Valve CLOSED";
        } else if (target_state == "LOW") {
            status_Kmsg = "POTASSIUM LOW (30-60%) - Valve PARTIAL";
        } else if (target_state == "CRITICAL") {
            status_Kmsg = "POTASSIUM CRITICAL (<30%) - Valve OPEN";
        }
        Serial.println("   " + status_Kmsg);
        Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        
        // Cập nhật servo và LED
        Serial.println("🧪 Activating Potassium valve servo to " + String(target_kangle) + "°")    ;
        activateValveServo(target_kangle);
        Serial.println("💡 Setting LED brightness to " + String(target_brightness)) ;
        setLEDBrightness(target_brightness);
        
        // Lưu trạng thái
        Serial.println("✅ State updated to " + target_kstate);
        current_state = target_kstate;
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
    
    updateLCD();

}


void publishSensorData() {
    if (!client.connected()) {
        return;
    }

    json_doc.clear();
    json_doc["device_id"] = "npk_control_board";
    json_doc["nitrogen_ppm"] = current_nitrogen;
    json_doc["phosphorus_ppm"] = current_phosphorus;
    json_doc["potassium_ppm"] = current_potassium;
    json_doc["potentiometer_value"] = current_potentiometer;
    json_doc["auto_mode"] = auto_mode;
    json_doc["n_servo_angle"] = nServo.read();
    json_doc["p_servo_angle"] = pServo.read();
    json_doc["k_servo_angle"] = kServo.read();
    json_doc["led_active"] = led_active;
    json_doc["uptime"] = millis() - system_start_time;
    
    String message;
    serializeJson(json_doc, message);
    
    client.publish(MQTT_TOPIC_SENSOR, message.c_str());
    Serial.println("🧪 NPK sensor data published to MQTT");
}

void publishSystemStatus() {
    if (!client.connected()) {
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
    json_doc["firmware_version"] = "1.0.3_pwm_stable"; // Cập nhật version
    json_doc["timestamp"] = millis();
    
    String message;
    serializeJson(json_doc, message);
    
    client.publish(MQTT_TOPIC_STATUS, message.c_str());
    Serial.println("🧪 NPK system status published to MQTT");
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
        Serial.println("🧪 Auto mode set to: " + String(auto_mode ? "AUTO" : "MANUAL"));
        updateLCD();
        
    } else if (command == "control_nitrogen_servo") {
        int angle = json_doc["angle"];
        setNServo(angle);
        updateLCD();
        
    } else if (command == "control_phosphorus_servo") {
        int angle = json_doc["angle"];
        setPServo(angle);
        updateLCD();
        
    } else if (command == "control_potassium_servo") {
        int angle = json_doc["angle"];
        setKServo(angle);
        updateLCD();
        
    } else if (command == "calibrate_npk") {
        float pot_factor = (float)readPotentiometer() / 2048.0; 
        n_calibration = pot_factor;
        p_calibration = pot_factor;
        k_calibration = pot_factor;
        Serial.println("🧪 NPK calibrated with factor: " + String(pot_factor));
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

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("=== ESP32 NPK Nutrient Control Board ===");
    Serial.println("Author: MiniMax Agent");
    Serial.println("Starting system initialization...");
    
    system_start_time = millis();
    system_initialized = false;
    
    // Initialize hardware pins
    Serial.println("Initializing hardware pins...");
    
    pinMode(N_PIN, INPUT);
    pinMode(P_PIN, INPUT);
    pinMode(K_PIN, INPUT);
    pinMode(POTENTIOMETER_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // Initialize servos
    Serial.println("Initializing servos...");
    // Gắn Servo với Pulse Width Min/Max TỐI ƯU (780us - 2250us)
    nServo.attach(N_SERVO_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
    pServo.attach(P_SERVO_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
    kServo.attach(K_SERVO_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
    
    // Khởi tạo pulse mạnh (Giống Soil Moisture)
    Serial.println("Sending initialization pulse to all servos (90 deg)...");
    nServo.write(SERVO_MID_ANGLE);
    pServo.write(SERVO_MID_ANGLE);
    kServo.write(SERVO_MID_ANGLE);
    delay(1000); 

    // Set servos to closed position
    deactivateAllServos();
    
    // Initialize LCD
    Serial.println("Initializing LCD display...");
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Smart Garden NPK");
    lcd.setCursor(0, 1);
    lcd.print("Control v1.0");
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
    readNitrogen();
    readPhosphorus();
    readPotassium();
    readPotentiometer();
    
    // Initialize LED
    deactivateLED();
    
    // Final system status
    system_initialized = true;
    updateLCD();
    
    Serial.println();
    Serial.println("=== System initialization complete! ===");
    Serial.println("NPK Control Board Ready");
    Serial.println("WiFi: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected"));
    Serial.println("MQTT: " + String(client.connected() ? "Connected" : "Disconnected"));
    Serial.println("Auto Mode: " + String(auto_mode ? "Enabled" : "Disabled"));
    Serial.println("N: " + String(current_nitrogen, 2) + "ppm, P: " + String(current_phosphorus, 2) + "ppm, K: " + String(current_potassium, 2) + "ppm");
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
        readNitrogen();
        readPhosphorus();
        readPotassium();
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
    if (current_time - last_lcd_update >= 3000) { // Update every 3 seconds
        updateLCD();
        last_lcd_update = current_time;
    }
    
    // Small delay to prevent excessive CPU usage
    delay(10);
}
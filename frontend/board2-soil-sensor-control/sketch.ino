/*
 * ESP32 Soil Moisture Control Board - FIXED
 * C++/Arduino Version for Smart Garden System
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
const int SOIL_MOISTURE_PIN = 34; // Cảm biến độ ẩm (AO)
const int VALVE_SERVO_PIN = 4;
const int LED_PIN = 2; // Dùng cho PWM
const int BUTTON_PIN = 25;
const int POTENTIOMETER_PIN = 32; // Biến trở
const int LCD_SDA_PIN = 21;
const int LCD_SCL_PIN = 22;
const int LCD_I2C_ADDRESS = 0x27;

// Soil Moisture Thresholds
const int MOISTURE_CRITICAL = 30; // Nếu độ ẩm < 30% -> Mở 180°
const int MOISTURE_LOW = 40;      // Ngưỡng thấp (Dùng cho logic 90°)
const int MOISTURE_OPTIMAL = 60;  // Nếu độ ẩm >= 60% -> Đóng 0°

// ADC Configuration (: Cần 4095 để ánh xạ)
const int MOISTURE_MAX_RAW = 4095; // Max raw ADC value (12-bit ADC)

// Timing Configuration
const unsigned long AUTOMATION_INTERVAL = 1000;
const unsigned long STATUS_INTERVAL = 5000;
const int SERVO_OPEN_ANGLE = 180;
const int SERVO_CLOSED_ANGLE = 0;
const int SERVO_MID_ANGLE = 90;

// System Variables
bool system_initialized = false;
bool mqtt_connected = false;
bool auto_mode = true;
bool valve_servo_open = false;

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

// Global Objects
WiFiClient espClient;
PubSubClient client(espClient);
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 16, 2);
DynamicJsonDocument json_doc(512);
Servo valveServo;

// Utility Functions
void setLEDBrightness(int brightness) { 
    brightness = constrain(brightness, 0, 255);
    led_brightness = brightness;
    analogWrite(LED_PIN, brightness);
    Serial.println("LED brightness set to: " + String(brightness));
}

void openValveAngle(int angle) {
    angle = constrain(angle, 0, 180);
    valveServo.write(angle);
    delay(10); // Lỗi 2 FIX: Thêm độ trễ nhỏ để đảm bảo tín hiệu Servo được gửi
    valve_servo_open = (angle != 0);
    Serial.println("💧 Valve servo OPEN: " + String(angle) + "°");
    // KHÔNG dùng digitalWrite(LED_PIN, HIGH); ở đây
}

void closeValveServo() {
    // Lỗi 2 FIX: Xóa lệnh dư thừa và không dùng digitalWrite cho LED ở đây
    if (valveServo.read() != 0) { // Chỉ đóng nếu đang mở
        valveServo.write(0);  
        delay(10); // Lỗi 2 FIX: Thêm độ trễ nhỏ
        valve_servo_open = false;
        // KHÔNG dùng digitalWrite(LED_PIN, HIGH); ở đây
        Serial.println("💧 Valve servo CLOSED: 0°");
    }
}

// Lỗi 1 FIX: Đảo ngược logic map
float readSoilMoisture()
{
    // Đọc giá trị ADC thô từ cảm biến
    int raw_value = analogRead(SOIL_MOISTURE_PIN);    
    // Đảm bảo giá trị Raw ADC nằm trong giới hạn 0-4095
    int constrained_raw = constrain(raw_value, 0, MOISTURE_MAX_RAW);
    
    // LỖI 1 FIX: Ánh xạ Raw ADC (0-4095) sang 100-0%
    // Raw ADC 0 (Wet/Low Resistance) -> 100% Moisture
    // Raw ADC 4095 (Dry/High Resistance) -> 0% Moisture
    int moisture_int = map(constrained_raw, 0, MOISTURE_MAX_RAW, 100, 0);
    
    // Lưu và trả về giá trị độ ẩm đã tính toán (phần trăm)
    current_moisture = constrain((float)moisture_int, 0.0, 100.0);
    
    // Log debug giá trị thô và đã tính toán
    Serial.print("Raw ADC Direct read (GPIO34): "); Serial.print(raw_value); 
    Serial.print(", Moisture Int (0-100%): "); Serial.println(moisture_int);
    Serial.print(", Moisture Calc: "); Serial.println(current_moisture);

    return current_moisture;
}

int readPotentiometer() {
    int reading = analogRead(POTENTIOMETER_PIN);  
    current_potentiometer = reading;
    
    // Map potentiometer (0-4095) to threshold range (20-80%)
    moisture_threshold = map(reading, 0, 4095, 20, 80);
    
    return reading;
}

void auto_control_logic() {
    int current_moisture = (int)readSoilMoisture(); // Lấy giá trị int để so sánh
    
    if (current_moisture >= MOISTURE_OPTIMAL) {
        Serial.println("💧 OPTIMAL/HIGH: "+String(current_moisture)+" Closing servo (0°)");
        setLEDBrightness(0);
        closeValveServo();          
    } else if (current_moisture >= MOISTURE_CRITICAL) { // Ví dụ: 30% <= Độ ẩm < 60%
        Serial.println("💧 LOW:"+String(current_moisture)+" Setting servo to (90°)");
        setLEDBrightness(127); // Sáng vừa
        openValveAngle(90);  
    } else { // Độ ẩm < 30%
        Serial.println("💧 CRITICAL:"+String(current_moisture)+" Opening servo (180°)");
        setLEDBrightness(255); // Sáng mạnh nhất
        openValveAngle(180);        
    }
}

void checkAutomation() {
    unsigned long current_time = millis();
    if (current_time - last_automation_check < AUTOMATION_INTERVAL) {
        return;
    }
    last_automation_check = current_time;

    if (!auto_mode) {
        return;
    }
    // readSoilMoisture() và readPotentiometer() được gọi bên trong auto_control_logic()
    // và cũng được gọi trong loop() để gửi MQTT.
    
    auto_control_logic();
}

void updateLCD() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Moisture:");
    lcd.print(current_moisture, 0);
    lcd.print("%");
    
    lcd.setCursor(0, 1);
    lcd.print(auto_mode ? "AUTO" : "MAN");
    lcd.print(" Ang:");
    lcd.print(valveServo.read()); // Hiển thị góc Servo thực tế
    
    // Show valve status
    if (valveServo.read() > SERVO_CLOSED_ANGLE) {
        lcd.print(" OPEN");
    } else {
        lcd.print(" CLOSED");
    }
}

// Các hàm kết nối WiFi/MQTT/Callback (Giữ nguyên)
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
    json_doc["valve_servo_angle"] = valveServo.read(); 
    json_doc["valve_open"] = valveServo.read() > SERVO_CLOSED_ANGLE;    
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
    json_doc["valve_state"] = valveServo.read() > SERVO_CLOSED_ANGLE ? "OPEN" : "CLOSED";    
    json_doc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
    json_doc["mqtt_connected"] = mqtt_connected;
    json_doc["auto_mode"] = auto_mode;
    json_doc["system_uptime"] = millis() - system_start_time;
    json_doc["firmware_version"] = "1.0.1_fixed"; // Cập nhật version
    json_doc["timestamp"] = millis();
    
    String message;
    serializeJson(json_doc, message);
    
    client.publish(MQTT_TOPIC_STATUS, message.c_str());
    Serial.println("💧 Soil moisture system status published to MQTT");
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
        Serial.println("💧 Auto mode set to: " + String(auto_mode ? "AUTO" : "MANUAL"));
        updateLCD();
        
    } else if (command == "control_pump") {
        bool activate = json_doc["activate"];
        if (activate) {
             Serial.println("💧 Manual watering started (180°)");
             openValveAngle(SERVO_OPEN_ANGLE);
        } else {
            Serial.println("💧 Manual watering stopped (0°)");
            closeValveServo();
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
    
    while (WiFi.status() != WL_CONNECTED && (millis() - start_time) < 20000) {
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
    // Rút gọn, chủ yếu check MQTT
    unsigned long current_time = millis();
    static unsigned long last_check = 0;
    if (current_time - last_check < 10000) {  // Check every 10 seconds
        return;
    }
    last_check = current_time;
    
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
    Serial.println("Author: Huynh Quoc Huy");
    Serial.println("Starting system initialization...");
    
    system_start_time = millis();
    system_initialized = false;

    // Initialize hardware pins
    Serial.println("Initializing hardware pins...");
    
    pinMode(SOIL_MOISTURE_PIN, INPUT);
    pinMode(POTENTIOMETER_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // Initialize servos
    Serial.println("Initializing pump servo...");
    // ESP32Servo attach
    valveServo.attach(VALVE_SERVO_PIN);
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
    } else {
        Serial.println("WiFi connection failed!");
    }
    
    // Initialize MQTT
    Serial.println("Initializing MQTT connection...");
    if (connectMQTT()) {
        Serial.println("MQTT connected successfully!");
    } else {
        Serial.println("MQTT connection failed!");
    }
    
    // Initialize sensor readings
    Serial.println("Initializing sensor readings...");
    current_moisture = readSoilMoisture();
    current_potentiometer = readPotentiometer();

    // Initialize outputs
    setLEDBrightness(0);

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
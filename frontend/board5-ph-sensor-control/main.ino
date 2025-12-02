/*
 * Smart Garden - Board 7: pH Sensor Control (ESP32)
 * Chức năng: Đo độ pH, tự động điều chỉnh bằng acid/alkaline dosing
 * Áp dụng: C++, I2C/SPI, Digital Analog, Servo, pH Control
 * Môi trường: Wokwi simulation
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>

// WiFi Configuration
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// MQTT Configuration
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* CLIENT_ID = "ESP_BOARD7_PH_01";
const char* PUBLISH_TOPIC = "garden/sensor/ph";
const char* SUBSCRIBE_TOPIC = "garden/control/ph_";

// Hardware Configuration - Board 7
#define PH_PIN_ANALOG 34
#define PH_PIN_DIGITAL 35
#define ACID_DOSING_SERVO_PIN 4
#define ALKALINE_DOSING_SERVO_PIN 5
#define ACID_LED_PIN 25
#define ALKALINE_LED_PIN 26
#define OPTIMAL_LED_PIN 27
#define STATUS_LED_PIN 2
#define CALIBRATE_BUTTON_PIN 23
#define PH_THRESHOLD_POT_PIN 32
#define AUTO_MODE_SWITCH_PIN 33
#define ALERT_BUZZER_PIN 13

// I2C/SPI Configuration
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// Servo Configuration
#include <ESP32Servo.h>
Servo acidDosingServo;
Servo alkalineDosingServo;

// LCD Configuration
LiquidCrystal_I2C lcd(0x27, 16, 2);

// WatchDog Timer
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 30

// Global Variables
WiFiClient espClient;
PubSubClient client(espClient);
String clientId = String(CLIENT_ID) + "_" + String(random(0xffff), HEX);

// pH Control Variables
float currentPH = 7.0;
float phThreshold = 7.0;
float phCalibration = 0.0;
bool phDigitalValue = false;
bool autoMode = true;
unsigned long lastSensorRead = 0;
unsigned long lastControlAction = 0;
unsigned long lastMQTTUpdate = 0;
unsigned long lastLCDUpdate = 0;

// Control thresholds
#define PH_ACIDIC_MIN 6.0
#define PH_OPTIMAL_MIN 6.5
#define PH_OPTIMAL_MAX 7.5
#define PH_ALKALINE_MAX 8.0

// Dosing control
#define DOSING_DURATION 5000  // 5 seconds dosing
#define DOSING_COOLDOWN 30000 // 30 seconds cooldown
#define SERVO_ACID_POSITION 0      // Acid dosing position
#define SERVO_NEUTRAL_POSITION 90  // Neutral position
#define SERVO_ALKALINE_POSITION 180 // Alkaline dosing position

// Status variables
enum PHStatus {
  PH_STRONG_ACIDIC,
  PH_ACIDIC,
  PH_OPTIMAL,
  PH_ALKALINE,
  PH_STRONG_ALKALINE
};

PHStatus currentPHStatus = PH_OPTIMAL;
bool acidDosingActive = false;
bool alkalineDosingActive = false;
unsigned long dosingStartTime = 0;

// Setup WiFi
void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    
    // Blink status LED while connecting
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
  }

  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Turn on status LED when connected
  digitalWrite(STATUS_LED_PIN, HIGH);
}

// Setup MQTT
void setupMQTT() {
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      
      // Subscribe to control topics
      client.subscribe(SUBSCRIBE_TOPIC);
      client.subscribe("garden/control/all");
      
      // Publish connection message
      client.publish("garden/status", (clientId + " connected").c_str(), true);
      
      digitalWrite(STATUS_LED_PIN, HIGH);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// MQTT Callback
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.println("Message received: [" + String(topic) + "] " + message);

  if (String(topic).equals(SUBSCRIBE_TOPIC)) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, message);
    
    if (doc.containsKey("action")) {
      String action = doc["action"];
      
      if (action.equals("auto_mode")) {
        autoMode = doc["enabled"];
        Serial.println("Auto mode: " + String(autoMode ? "ON" : "OFF"));
      } else if (action.equals("calibrate")) {
        calibratePH();
      } else if (action.equals("manual_dosing")) {
        String type = doc["type"];
        int duration = doc["duration"];
        performManualDosing(type, duration);
      } else if (action.equals("set_threshold")) {
        phThreshold = doc["threshold"];
        Serial.println("pH threshold set to: " + String(phThreshold));
      }
    }
  }
}

// Read pH sensor
void readPHSensor() {
  // Read analog value (0-4095) and convert to pH
  int rawValue = analogRead(PH_PIN_ANALOG);
  phDigitalValue = digitalRead(PH_PIN_DIGITAL);
  
  // Convert ADC value to pH (calibrated)
  // ADC range: ~1200 (acidic) to ~2800 (alkaline), neutral ~2048
  float rawPH;
  if (rawValue <= 2048) {
    // Acidic range
    rawPH = map(rawValue, 1200, 2048, 4.0, 7.0);
  } else {
    // Alkaline range
    rawPH = map(rawValue, 2048, 2800, 7.0, 10.0);
  }
  
  // Apply calibration
  currentPH = rawPH + phCalibration;
  currentPH = constrain(currentPH, 0.0, 14.0);
  
  // Determine pH status
  updatePHStatus();
  
  // Read threshold potentiometer
  int potValue = analogRead(PH_THRESHOLD_POT_PIN);
  phThreshold = map(potValue, 0, 4095, 5.0, 9.0);
  
  // Read auto mode switch
  autoMode = digitalRead(AUTO_MODE_SWITCH_PIN);
}

// Update pH status
void updatePHStatus() {
  if (currentPH < 5.0) {
    currentPHStatus = PH_STRONG_ACIDIC;
  } else if (currentPH < PH_ACIDIC_MIN) {
    currentPHStatus = PH_ACIDIC;
  } else if (currentPH >= PH_OPTIMAL_MIN && currentPH <= PH_OPTIMAL_MAX) {
    currentPHStatus = PH_OPTIMAL;
  } else if (currentPH <= PH_ALKALINE_MAX) {
    currentPHStatus = PH_ALKALINE;
  } else {
    currentPHStatus = PH_STRONG_ALKALINE;
  }
}

// Perform pH control
void performPHControl() {
  if (!autoMode) return; // Manual mode - no automatic control
  
  unsigned long currentTime = millis();
  
  // Check if we're in cooldown period
  if (currentTime - lastControlAction < DOSING_COOLDOWN) {
    return;
  }
  
  bool needsAcid = false;
  bool needsAlkaline = false;
  
  // Determine if pH adjustment is needed
  if (currentPH < (phThreshold - 0.5)) {
    needsAlkaline = true; // pH too low, need to increase (make less acidic)
  } else if (currentPH > (phThreshold + 0.5)) {
    needsAcid = true; // pH too high, need to decrease (make less alkaline)
  }
  
  // Perform dosing if needed
  if (needsAlkaline && !alkalineDosingActive) {
    startAlkalineDosing();
  } else if (needsAcid && !acidDosingActive) {
    startAcidDosing();
  } else if (!needsAcid && !needsAlkaline) {
    stopAllDosing();
  }
}

// Start acid dosing
void startAcidDosing() {
  Serial.println("Starting ACID dosing...");
  acidDosingActive = true;
  dosingStartTime = millis();
  lastControlAction = millis();
  
  // Move servo to acid position
  acidDosingServo.write(SERVO_ACID_POSITION);
  
  // Turn on acid LED
  digitalWrite(ACID_LED_PIN, HIGH);
  
  // Turn off alkaline LED
  digitalWrite(ALKALINE_LED_PIN, LOW);
  
  // Sound alert
  digitalWrite(ALERT_BUZZER_PIN, HIGH);
  delay(500);
  digitalWrite(ALERT_BUZZER_PIN, LOW);
  
  client.publish("garden/log", "pH control: Starting acid dosing", false);
}

// Start alkaline dosing
void startAlkalineDosing() {
  Serial.println("Starting ALKALINE dosing...");
  alkalineDosingActive = true;
  dosingStartTime = millis();
  lastControlAction = millis();
  
  // Move servo to alkaline position
  alkalineDosingServo.write(SERVO_ALKALINE_POSITION);
  
  // Turn on alkaline LED
  digitalWrite(ALKALINE_LED_PIN, HIGH);
  
  // Turn off acid LED
  digitalWrite(ACID_LED_PIN, LOW);
  
  // Sound alert
  digitalWrite(ALERT_BUZZER_PIN, HIGH);
  delay(500);
  digitalWrite(ALERT_BUZZER_PIN, LOW);
  
  client.publish("garden/log", "pH control: Starting alkaline dosing", false);
}

// Stop all dosing
void stopAllDosing() {
  if (acidDosingActive || alkalineDosingActive) {
    Serial.println("Stopping all dosing...");
    
    // Return servos to neutral
    acidDosingServo.write(SERVO_NEUTRAL_POSITION);
    alkalineDosingServo.write(SERVO_NEUTRAL_POSITION);
    
    // Turn off LEDs
    digitalWrite(ACID_LED_PIN, LOW);
    digitalWrite(ALKALINE_LED_PIN, LOW);
    
    // Turn on optimal LED if pH is in optimal range
    if (currentPHStatus == PH_OPTIMAL) {
      digitalWrite(OPTIMAL_LED_PIN, HIGH);
    } else {
      digitalWrite(OPTIMAL_LED_PIN, LOW);
    }
    
    acidDosingActive = false;
    alkalineDosingActive = false;
    
    client.publish("garden/log", "pH control: Stopped dosing", false);
  }
}

// Check dosing timeout
void checkDosingTimeout() {
  if (acidDosingActive || alkalineDosingActive) {
    unsigned long currentTime = millis();
    if (currentTime - dosingStartTime > DOSING_DURATION) {
      stopAllDosing();
    }
  }
}

// Perform manual dosing
void performManualDosing(String type, int duration) {
  Serial.println("Manual dosing: " + type + " for " + String(duration) + "ms");
  
  if (type.equals("acid")) {
    acidDosingServo.write(SERVO_ACID_POSITION);
    digitalWrite(ACID_LED_PIN, HIGH);
  } else if (type.equals("alkaline")) {
    alkalineDosingServo.write(SERVO_ALKALINE_POSITION);
    digitalWrite(ALKALINE_LED_PIN, HIGH);
  }
  
  delay(duration);
  
  // Stop dosing
  acidDosingServo.write(SERVO_NEUTRAL_POSITION);
  alkalineDosingServo.write(SERVO_NEUTRAL_POSITION);
  digitalWrite(ACID_LED_PIN, LOW);
  digitalWrite(ALKALINE_LED_PIN, LOW);
  
  client.publish("garden/log", "Manual dosing completed: " + type, false);
}

// Calibrate pH sensor
void calibratePH() {
  Serial.println("Calibrating pH sensor...");
  
  // Take multiple readings for calibration
  float sum = 0;
  int readings = 10;
  
  for (int i = 0; i < readings; i++) {
    int rawValue = analogRead(PH_PIN_ANALOG);
    float tempPH;
    
    if (rawValue <= 2048) {
      tempPH = map(rawValue, 1200, 2048, 4.0, 7.0);
    } else {
      tempPH = map(rawValue, 2048, 2800, 7.0, 10.0);
    }
    
    sum += tempPH;
    delay(100);
  }
  
  float averagePH = sum / readings;
  
  // Assume we're calibrating with pH 7.0 buffer solution
  phCalibration = 7.0 - averagePH;
  
  Serial.println("Calibration completed. Offset: " + String(phCalibration));
  client.publish("garden/log", "pH sensor calibrated. Offset: " + String(phCalibration), false);
  
  // Blink status LED to indicate calibration
  for (int i = 0; i < 5; i++) {
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(200);
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(200);
  }
}

// Update LCD display
void updateLCD() {
  lcd.clear();
  
  // Line 1: pH value and status
  lcd.setCursor(0, 0);
  lcd.print("pH: ");
  lcd.print(currentPH, 1);
  
  // Show status
  lcd.setCursor(8, 0);
  switch (currentPHStatus) {
    case PH_STRONG_ACIDIC: lcd.print("S-ACID"); break;
    case PH_ACIDIC: lcd.print("ACID"); break;
    case PH_OPTIMAL: lcd.print("OPTIMAL"); break;
    case PH_ALKALINE: lcd.print("ALKALINE"); break;
    case PH_STRONG_ALKALINE: lcd.print("S-ALK"); break;
  }
  
  // Line 2: Threshold and mode
  lcd.setCursor(0, 1);
  lcd.print("Thr: ");
  lcd.print(phThreshold, 1);
  
  lcd.setCursor(8, 1);
  lcd.print(autoMode ? "AUTO" : "MAN");
  
  // Show dosing status
  if (acidDosingActive) {
    lcd.setCursor(13, 1);
    lcd.print("AC");
  } else if (alkalineDosingActive) {
    lcd.setCursor(13, 1);
    lcd.print("AL");
  }
}

// Send MQTT data
void sendMQTTData() {
  DynamicJsonDocument doc(1024);
  
  doc["device_id"] = CLIENT_ID;
  doc["ph_value"] = currentPH;
  doc["ph_status"] = getPHStatusString();
  doc["digital_value"] = phDigitalValue;
  doc["analog_raw"] = analogRead(PH_PIN_ANALOG);
  doc["threshold"] = phThreshold;
  doc["auto_mode"] = autoMode;
  doc["calibration"] = phCalibration;
  doc["acid_dosing_active"] = acidDosingActive;
  doc["alkaline_dosing_active"] = alkalineDosingActive;
  doc["timestamp"] = millis();
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  client.publish(PUBLISH_TOPIC, jsonString.c_str());
}

// Get pH status string
String getPHStatusString() {
  switch (currentPHStatus) {
    case PH_STRONG_ACIDIC: return "strong_acidic";
    case PH_ACIDIC: return "acidic";
    case PH_OPTIMAL: return "optimal";
    case PH_ALKALINE: return "alkaline";
    case PH_STRONG_ALKALINE: return "strong_alkaline";
    default: return "unknown";
  }
}

// Setup function
void setup() {
  Serial.begin(115200);
  Serial.println("=== Smart Garden - Board 7: pH Sensor Control ===");
  
  // Initialize hardware pins
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ACID_LED_PIN, OUTPUT);
  pinMode(ALKALINE_LED_PIN, OUTPUT);
  pinMode(OPTIMAL_LED_PIN, OUTPUT);
  pinMode(CALIBRATE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(AUTO_MODE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(ALERT_BUZZER_PIN, OUTPUT);
  
  // Initialize servos
  acidDosingServo.attach(ACID_DOSING_SERVO_PIN);
  alkalineDosingServo.attach(ALKALINE_DOSING_SERVO_PIN);
  acidDosingServo.write(SERVO_NEUTRAL_POSITION);
  alkalineDosingServo.write(SERVO_NEUTRAL_POSITION);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart Garden");
  lcd.setCursor(0, 1);
  lcd.print("pH Control v1.0");
  delay(2000);
  
  // Setup WiFi and MQTT
  setupWiFi();
  setupMQTT();
  
  // Initialize WatchDog
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  Serial.println("Setup completed. Starting main loop...");
  
  // Initial readings
  readPHSensor();
  updateLCD();
}

// Main loop
void loop() {
  // Feed WatchDog
  esp_task_wdt_reset();
  
  // Reconnect MQTT if needed
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();
  
  unsigned long currentTime = millis();
  
  // Read sensor every 1 second
  if (currentTime - lastSensorRead > 1000) {
    readPHSensor();
    lastSensorRead = currentTime;
  }
  
  // Perform pH control
  performPHControl();
  
  // Check dosing timeout
  checkDosingTimeout();
  
  // Update LCD every 500ms
  if (currentTime - lastLCDUpdate > 500) {
    updateLCD();
    lastLCDUpdate = currentTime;
  }
  
  // Send MQTT data every 5 seconds
  if (currentTime - lastMQTTUpdate > 5000) {
    sendMQTTData();
    lastMQTTUpdate = currentTime;
  }
  
  // Handle calibration button
  if (digitalRead(CALIBRATE_BUTTON_PIN) == LOW) {
    delay(50); // Debounce
    if (digitalRead(CALIBRATE_BUTTON_PIN) == LOW) {
      calibratePH();
    }
  }
  
  delay(100);
}

// Interrupt Service Routine (optional - for emergency stop)
void IRAM_ATTR emergencyStop() {
  stopAllDosing();
  acidDosingActive = false;
  alkalineDosingActive = false;
}
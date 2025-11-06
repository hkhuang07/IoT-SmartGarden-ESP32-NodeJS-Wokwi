#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Dinh nghia cac chan ket noi
#define SOIL_MOISTURE_PIN A0
#define RAIN_SENSOR_PIN A1
#define LIGHT_SENSOR_PIN A2
#define RELAY_PUMP_PIN 2
#define RELAY_HEATER_PIN 3

// Dinh nghia cac gioi han
#define SEUIL_SOIL_DRY 500
#define SEUIL_RAIN_DRY 102
#define SEUIL_TEMP_LOW 18.0
#define SEUIL_LIGHT_LOW 100

// Thiet lap giao tiep cho NPK
SoftwareSerial NPKSerial(16, 17); // RX, TX
const byte NPK_REQUEST_N = {0x01, 0x03, 0x00, 0x1E, 0x00, 0x01, 0xE4, 0x0C};
const byte NPK_REQUEST_P = {0x01, 0x03, 0x00, 0x1F, 0x00, 0x01, 0xB5, 0xCC};
const byte NPK_REQUEST_K = {0x01, 0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xC0};

// Khoi tao doi tuong BME280
Adafruit_BME280 bme;

// Cac ham doc gia tri tu cam bien
int readSoilMoisture() {
  return analogRead(SOIL_MOISTURE_PIN);
}

int readRainSensor() {
  return analogRead(RAIN_SENSOR_PIN);
}

int readLightSensor() {
  return analogRead(LIGHT_SENSOR_PIN);
}

void readBME280Data() {
  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();
  float pressure = bme.readPressure() / 100.0F; // Chuyen ve hPa
  float altitude = bme.readAltitude(1013.25); // 1013.25 hPa la ap suat tieu chuan

  Serial.print("Temperature - Nhiet do: ");
  Serial.print(temperature);Serial.println(" *C");
  Serial.print("Nhiet do: ");
  Serial.print(temperature);Serial.println(" *C");

  Serial.print("Air humidity: ");
  Serial.print(humidity);   Serial.println(" %");
  Serial.print("Do am khong khi: ");
  Serial.print(humidity);   Serial.println(" %");

  Serial.print("Pressure: ");
  Serial.print(pressure);   Serial.println(" hPa");
  Serial.print("Ap suat: ");
  Serial.print(pressure);   Serial.println(" hPa");

  Serial.print("Estimated altitude: ");
  Serial.print(altitude);   Serial.println(" m");
  Serial.print("Do cao uoc tinh: ");
  Serial.print(altitude);   Serial.println(" m");
 
}

void readNPKData() {
  byte npkBuffer;
  int N = 0, P = 0, K = 0;

  // Doc gia tri N
  NPKSerial.write(NPK_REQUEST_N, sizeof(NPK_REQUEST_N));
  delay(100);
  if (NPKSerial.available() >= 7) {
    for (int i = 0; i < 7; i++) {
      npkBuffer[i] = NPKSerial.read();
    }
    N = (int)npkBuffer;
  }

  // Doc gia tri P
  NPKSerial.write(NPK_REQUEST_P, sizeof(NPK_REQUEST_P));
  delay(100);
  if (NPKSerial.available() >= 7) {
    for (int i = 0; i < 7; i++) {
      npkBuffer[i] = NPKSerial.read();
    }
    P = (int)npkBuffer;
  }
  
  // Doc gia tri K
  NPKSerial.write(NPK_REQUEST_K, sizeof(NPK_REQUEST_K));
  delay(100);
  if (NPKSerial.available() >= 7) {
    for (int i = 0; i < 7; i++) {
      npkBuffer[i] = NPKSerial.read();
    }
    K = (int)npkBuffer;
  }

  Serial.print("NPK: N="); Serial.print(N); Serial.print(" P="); Serial.print(P); Serial.print(" K="); Serial.println(K);
}

void setup() {
  Serial.begin(115200);
  NPKSerial.begin(9600); // Giao tiep voi cam bien NPK
  
  pinMode(RELAY_PUMP_PIN, OUTPUT);
  pinMode(RELAY_HEATER_PIN, OUTPUT);
  digitalWrite(RELAY_PUMP_PIN, HIGH); // Tat relay mac dinh
  digitalWrite(RELAY_HEATER_PIN, HIGH);

  if (!bme.begin()) {
    Serial.println("BME280 sensor not found!");
    Serial.println("Khong tim thay cam bien BME280!");
  } else {
    Serial.println("Initialize BME280 successfully!");
    Serial.println("Khoi tao BME280 thanh cong!");
  }
}

void loop() {
  // Doc du lieu tu cac cam bien
  int soilMoisture = readSoilMoisture();
  int rainValue = readRainSensor();
  int lightValue = readLightSensor();
  
  Serial.print("Soil moisture: ");  Serial.println(soilMoisture);
  Serial.print("Do am dat: ");      Serial.println(soilMoisture);
  Serial.print("Rainfall: ");       Serial.println(rainValue);
  Serial.print("Gia tri mua: ");    Serial.println(rainValue);
  Serial.print("Brightness: ");     Serial.println(lightValue);
  Serial.print("Gia tri anh sang: ");Serial.println(lightValue);

  // Kiem tra logic dieu khien
  if (rainValue > SEUIL_RAIN_DRY) { // Khong mua
    if (soilMoisture < SEUIL_SOIL_DRY) { // Dat kho
      Serial.println("Dry soil. Watering plants automatically...");
      Serial.println("Dat kho. Tuoi cay tu dong...");
      digitalWrite(RELAY_PUMP_PIN, LOW); // Bat bom
    } else {
      Serial.println("Expected soil moisture. Pump is off.");
      Serial.println("Do am dat du. Bom da tat.");
      digitalWrite(RELAY_PUMP_PIN, HIGH); // Tat bom
    }
  } else {
    Serial.println("It's raining. Turn off the irrigation system.");
    Serial.println("Troi dang mua. Ngat he thong tuoi.");
    digitalWrite(RELAY_PUMP_PIN, HIGH); // Luon tat bom khi mua
  }

  // Kiem tra nhiet do va anh sang, dieu khien den suoi
  readBME280Data();
  if (lightValue < SEUIL_LIGHT_LOW && bme.readTemperature() < SEUIL_TEMP_LOW) {
    Serial.println("It's dark and the temperature is low. Turn on the heater.");
    Serial.println("Troi toi va nhiet do thap. Bat den suoi.");
    digitalWrite(RELAY_HEATER_PIN, LOW);
  } else {
    Serial.println("It's bright or the temperature is normal. Turn off the heater.");
    Serial.println("Troi sang hoac nhiet do binh thuong. Tat den suoi.");
    digitalWrite(RELAY_HEATER_PIN, HIGH);
  }

  // Doc du lieu NPK (dinh ky)
  readNPKData();

  Serial.println("--------------------");
  delay(5000); // delay de de quan sat
}
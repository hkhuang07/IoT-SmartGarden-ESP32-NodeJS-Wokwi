#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Nếu dùng Cloudflare Tunnel: "mqtt-broker.hkhuang07.me"
const char* mqtt_server = "mqtt-broker.hkhuang07.me"; 
const int mqtt_port = 1883; 

#define CLIENT_ID "ESP_SENSOR_SOIL_MOISTURE_01" 
#define PUBLISH_INTERVAL 30000              

#define SOIL_MOISTURE_PIN 34 
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;

const char* ssid = "Wokwi-WiFi";
const char* password = "";

void callback(char* topic, byte* payload, unsigned int length) {}

void setup_wifi(){
  delay(10);
  WiFi.begin(ssid,password);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print("="); 
  }
  Serial.println(">\nWiFi connected!");
}

void reconnect(){
  while(!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if(client.connect(CLIENT_ID)){
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("Try again in 3 seconds");
      delay(3000);
    }
  }
}

void loop(){
  if(!client.connected())
  {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > PUBLISH_INTERVAL){
    lastMsg = now;

    // ĐỌC GIÁ TRỊ ADC TỪ CHÂN CẢM BIẾN ĐỘ ẨM ĐẤT
    int soil_moisture_raw = analogRead(SOIL_MOISTURE_PIN); 
    
    StaticJsonDocument<256> doc;
    // Sử dụng 'm' (moisture) cho giá trị độ ẩm
    doc["m"] = soil_moisture_raw; 
    
    char payload_str[256];
    serializeJson(doc, payload_str);
    
    client.publish("smartgarden/sensor/soil_moisture", payload_str); 

    Serial.print("Published data: ");
    Serial.println(payload_str); 
  }
}

void setup()
{
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server,mqtt_port);
  pinMode(SOIL_MOISTURE_PIN, INPUT); 
  client.setCallback(callback);
}
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wokwi_MQTT.h>

//MQTT Model & Wifi Configuration
const char* mqtt_server = "127.0.0.1"; 
const int mqtt_port = 1883;

#define CLIENT_ID "ESP_SENSOR_LDR_FRONT" 
#define PUBLISH_INTERVAL 30000 

//Light Sensor Configuration
#define LDR_PIN 32
WifiClient espClient;
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
  Serial.print(">\nWiFi connected!");(

  Wokwi_MQTT_Config config;
  config.broker_url = (char*)("tcp://" + String(mqtt_server) + ":" + String(mqtt_port)).c_str();
  Wokwi_MQTT_Setup(&config);
}

void reconnect(){
  while(!client.connected())
  {
    Serial.print("Attemping MQTT connection...");
    if(client.connected(CLIENT_ID)){
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
    reconnected();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > PUBLISH_INTERVAL){
    lastMsg = now;

    int light_raw = analogRead(LDR_PIN);
    
    StaticJsonDocument<256> doc;
    doc["l"] = light_raw; 

    char payload_str[256];
    serializeJson(doc, payload_str);
    
    client.publish("smartgarden/sensor/temp_humidity",payload_str);

    Serial.print("Published data: ");
    Serial.println(payload_str); 
}

void setup()
{
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server,mqtt_port);
  pinMode(LDR_PIN, INPUT); 
  client.setCallback(callback);
}
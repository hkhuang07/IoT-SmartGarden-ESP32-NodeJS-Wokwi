#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* mqtt_server = "mqtt-broker.hkhuang07.me"; 
const int mqtt_port = 1883; 

#define CLIENT_ID "ESP_SENSOR_NPK_03" 
#define PUBLISH_INTERVAL 30000              

#define N_PIN 34    
#define P_PIN 35    
#define K_PIN 32   

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;

const char* ssid = "Wokwi-WiFi";
const char* password = "";

void callback(char* topic, byte* payload, unsigned int length) {}

void setup_wifi(){
  delay(10);
  WiFi.begin(ssid,password);
  while(WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print("="); 
    Serial.println(">\nWiFi connected!");  
  }
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

    int n_raw = analogRead(N_PIN);  
    int p_raw = analogRead(P_PIN);  
    int k_raw = analogRead(K_PIN);  
    
    StaticJsonDocument<256> doc;
    doc["n"] = n_raw;
    doc["p"] = p_raw;
    doc["k"] = k_raw;
    
    char payload_str[256];
    serializeJson(doc, payload_str);
    
    client.publish("smartgarden/sensor/npk_level", payload_str); 

    Serial.print("Published data: ");
    Serial.println(payload_str); 
  }
}

void setup()
{
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server,mqtt_port);
  pinMode(N_PIN, INPUT); 
  pinMode(P_PIN, INPUT);
  pinMode(K_PIN, INPUT);
  client.setCallback(callback);
}
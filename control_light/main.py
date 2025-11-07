import time
import json
from machine import Pin
from umqtt.simple import MQTTClient

MQTT_SERVER = "mqtt-broker.hkhuang07.me" 
MQTT_PORT = 8883 
CLIENT_ID = "MPY_ACTUATOR_LIGHT_02" 
TOPIC_SUB_LIGHT = b"smartgarden/control/light_main" 

LIGHT_RELAY_PIN = 15 

main_light_relay = Pin(LIGHT_RELAY_PIN, Pin.OUT)
main_light_relay.value(1) 

client = None 

def sub_cb(topic, msg):
    global client
    print(f"[CMD] Topic: {topic.decode()} | Payload: {msg.decode()}")
    
    if topic == TOPIC_SUB_LIGHT:
        command = msg.decode().upper() 
        
        if command == "ON":
            main_light_relay.value(0) 
            print("--> LIGHT STATUS: ON")
            client.publish(b"smartgarden/status/light_main", b"ON") 
            
        elif command == "OFF":
            main_light_relay.value(1)
            print("--> LIGHT STATUS: OFF")
            client.publish(b"smartgarden/status/light_main", b"OFF")
        
        else:
            print("--> Unknown command received.")

def connect_mqtt():
    global client
    try:
        client = MQTTClient(CLIENT_ID, MQTT_SERVER, port=MQTT_PORT)
        client.set_callback(sub_cb) 
        client.connect()
        print(f"MQTT Connected to {MQTT_SERVER}:{MQTT_PORT}")
        client.subscribe(TOPIC_SUB_LIGHT) 
        return client
    except Exception as e:
        print(f"ERROR: Could not connect to MQTT: {e}")
        time.sleep(5)
        return None

while True:
    if client is None:
        client = connect_mqtt()
    if client:
        try:
            client.check_msg() 
        except OSError as e:
            print(f"Connection lost: {e}. Reconnecting...")
            client = None
    time.sleep(1) 
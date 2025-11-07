import time
import json
from machine import Pin, PWM
from umqtt.simple import MQTTClient

MQTT_SERVER = "mqtt-broker.hkhuang07.me" 
MQTT_PORT = 8883 
CLIENT_ID = "MPY_ACTUATOR_VALVE_01"
TOPIC_SUB_VALVE = b"smartgarden/control/water_valve" 

VALVE_SERVO_PIN = 15
# Khai báo Servo PWM
servo_pwm = PWM(Pin(VALVE_SERVO_PIN), freq=50)

# Giá trị xung cho Servo (0-1023)
VALVE_OPEN_DUTY = 120  
VALVE_CLOSE_DUTY = 40 

client = None 

def sub_cb(topic, msg):
    global client
    print(f"[CMD] Topic: {topic.decode()} | Payload: {msg.decode()}")
    
    if topic == TOPIC_SUB_VALVE:
        command = msg.decode().upper() 
        
        if command == "ON":
            servo_pwm.duty(VALVE_OPEN_DUTY)
            print("--> VALVE STATUS: OPENED (ON)")
            client.publish(b"smartgarden/status/water_valve_1", b"OPENED") 
            
        elif command == "OFF":
            servo_pwm.duty(VALVE_CLOSE_DUTY)
            print("--> VALVE STATUS: CLOSED (OFF)")
            client.publish(b"smartgarden/status/water_valve_1", b"CLOSED")
        
        else:
            print("--> Unknown command received.")


# MQTT Model Connection
def connect_mqtt():
    global client
    try:
        client = MQTTClient(CLIENT_ID, MQTT_SERVER, port=MQTT_PORT)
        client.set_callback(sub_cb) 
        client.connect()
        print(f"MQTT Connected to {MQTT_SERVER}:{MQTT_PORT}")
        client.subscribe(TOPIC_SUB_VALVE) 
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
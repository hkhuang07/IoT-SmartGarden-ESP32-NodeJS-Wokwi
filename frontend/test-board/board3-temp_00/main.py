#!/usr/bin/env python3
"""
Smart Garden - Temperature & Humidity Control Board
Board nhận dữ liệu từ DHT22 sensor qua MQTT
Điều khiển: LCD hiển thị, servo van nước, servo mái che, manual switches
Lưu dữ liệu vào MongoDB
"""

import network
import time
import json
import gc
import machine
from machine import Pin, PWM, I2C, unique_id
from umqtt.simple import MQTTClient

# ===== WIFI CONFIGURATION =====
WIFI_SSID = "Wokwi-GUEST"
WIFI_PASSWORD = ""

# ===== MQTT CONFIGURATION =====
MQTT_SERVER = "broker.hivemq.com"
MQTT_PORT = 1883
CLIENT_ID = "ESP_CONTROL_TEMP_HUM_01"

SUBSCRIBE_TOPIC = b"garden/sensor/temp_humidity"
STATUS_TOPIC = b"garden/status/temp_control"
MONGO_TOPIC = b"garden/database/temp_control"

# ===== HARDWARE PINS =====
# LCD Configuration (I2C)
LCD_SDA_PIN = 21
LCD_SCL_PIN = 22
LCD_I2C_ADDR = 0x27

# Servo Pins
SERVO_VALVE_PIN = 15    # Servo van nước
SERVO_ROOF_PIN = 14     # Servo mái che

# Manual Control Switches
SWITCH_VALVE_PIN = 25   # Công tắc điều khiển van nước
SWITCH_ROOF_PIN = 26    # Công tắc điều khiển mái che
SWITCH_MODE_PIN = 27    # Công tắc chuyển đổi Auto/Manual

# LED Indicator
LED_PIN = 2

# ===== SERVO CONFIGURATION =====
VALVE_OPEN_DUTY = 150     # Van mở
VALVE_CLOSE_DUTY = 50     # Van đóng
ROOF_OPEN_DUTY = 180      # Mái che mở (0 độ)
ROOF_CLOSE_DUTY = 0       # Mái che đóng (180 độ)
SERVO_DELAY = 800         # Độ trễ servo
SERVO_FREQ = 50           # Tần số PWM servo

# ===== LCD CONFIGURATION =====
LCD_WIDTH = 20
LCD_HEIGHT = 4

# ===== SYSTEM STATE =====
wifi = None
mqtt_client = None
lcd = None
servo_valve = None
servo_roof = None
switch_valve = None
switch_roof = None
switch_mode = None
led = None

# Control State
auto_mode = True
current_temp = 0.0
current_humid = 0.0
valve_state = "CLOSED"
roof_state = "CLOSED"
mqtt_connected = False
wifi_connected = False
last_data_time = 0
last_activity = time.ticks_ms()

# Thresholds
TEMP_THRESHOLD_COLD = 18.0
TEMP_THRESHOLD_HOT = 30.0
HUMID_THRESHOLD_LOW = 40.0
HUMID_THRESHOLD_HIGH = 70.0

def setup_wifi():
    """Setup WiFi connection"""
    global wifi, wifi_connected
    
    try:
        print("WiFi: Initializing...")
        wifi = network.WLAN(network.STA_IF)
        wifi.active(True)
        
        print(f"WiFi: Connecting to {WIFI_SSID}")
        wifi.connect(WIFI_SSID, WIFI_PASSWORD)
        
        # Wait for connection
        max_attempts = 30
        for attempt in range(max_attempts):
            if wifi.isconnected():
                wifi_connected = True
                ip = wifi.ifconfig()[0]
                print("✅ WiFi: Connected successfully!")
                print("IP:", ip)
                return True
            
            if attempt % 3 == 0:
                print(f"WiFi: Attempt {attempt+1}/{max_attempts}")
            
            time.sleep(0.5)
        
        wifi_connected = False
        print("❌ WiFi: Connection failed")
        return False
        
    except Exception as e:
        wifi_connected = False
        print("❌ WiFi Setup error:", str(e))
        return False

def setup_lcd():
    """Setup I2C LCD Display"""
    global lcd
    
    try:
        print("LCD: Initializing...")
        i2c = I2C(0, sda=Pin(LCD_SDA_PIN), scl=Pin(LCD_SCL_PIN), freq=400000)
        i2c.writeto(0x27, b'\x00')  # Wake up LCD
        
        # Simple LCD initialization
        lcd = i2c
        print("✅ LCD: Initialized")
        
        # Clear display
        lcd.clear()
        
        # Welcome message
        lcd.putstr("Smart Garden v2.0")
        lcd.move_to(0, 1)
        lcd.putstr("Temp&Hum Control")
        
        return True
        
    except Exception as e:
        print("❌ LCD Setup error:", str(e))
        return False

def lcd_update_display():
    """Update LCD display with current data"""
    try:
        if not lcd:
            return
            
        # Clear display
        lcd.putstr("                    ")  # Clear line 1
        lcd.move_to(0, 1)
        lcd.putstr("                    ")  # Clear line 2
        lcd.move_to(0, 2)
        lcd.putstr("                    ")  # Clear line 3
        lcd.move_to(0, 3)
        lcd.putstr("                    ")  # Clear line 4
        
        # Line 1: Temperature
        lcd.move_to(0, 0)
        lcd.putstr(f"Temp: {current_temp:.1f}C")
        
        # Line 2: Humidity
        lcd.move_to(0, 1)
        lcd.putstr(f"Humid: {current_humid:.1f}%")
        
        # Line 3: Valve and Roof status
        lcd.move_to(0, 2)
        lcd.putstr(f"Valve: {valve_state[:4]} Roof:{roof_state[:4]}")
        
        # Line 4: Mode and connection status
        mode_str = "AUTO" if auto_mode else "MAN"
        wifi_str = "W" if wifi_connected else "-"
        mqtt_str = "M" if mqtt_connected else "-"
        lcd.move_to(0, 3)
        lcd.putstr(f"Mode:{mode_str} {wifi_str}{mqtt_str}")
        
    except Exception as e:
        print("❌ LCD update error:", str(e))

def setup_servos():
    """Setup servo motors"""
    global servo_valve, servo_roof
    
    try:
        print("Servos: Initializing...")
        
        # Setup water valve servo
        servo_valve_pin = Pin(SERVO_VALVE_PIN)
        servo_valve = PWM(servo_valve_pin)
        servo_valve.freq(SERVO_FREQ)
        servo_valve.duty(VALVE_CLOSE_DUTY)
        
        # Setup roof servo
        servo_roof_pin = Pin(SERVO_ROOF_PIN)
        servo_roof = PWM(servo_roof_pin)
        servo_roof.freq(SERVO_FREQ)
        servo_roof.duty(ROOF_CLOSE_DUTY)
        
        print("✅ Servos: Initialized")
        return True
        
    except Exception as e:
        print("❌ Servo setup error:", str(e))
        return False

def control_water_valve(open_valve):
    """Control water valve servo"""
    global valve_state
    
    try:
        target_duty = VALVE_OPEN_DUTY if open_valve else VALVE_CLOSE_DUTY
        servo_valve.duty(target_duty)
        valve_state = "OPEN" if open_valve else "CLOSED"
        
        print(f"💧 Water Valve: {valve_state}")
        time.sleep_ms(SERVO_DELAY)
        
        # Update LED
        if open_valve:
            led.on()
        else:
            led.off()
            
        return True
        
    except Exception as e:
        print("❌ Water valve control error:", str(e))
        return False

def control_roof(open_roof):
    """Control roof servo"""
    global roof_state
    
    try:
        target_duty = ROOF_OPEN_DUTY if open_roof else ROOF_CLOSE_DUTY
        servo_roof.duty(target_duty)
        roof_state = "OPEN" if open_roof else "CLOSED"
        
        print(f"🏠 Roof: {roof_state}")
        time.sleep_ms(SERVO_DELAY)
        
        return True
        
    except Exception as e:
        print("❌ Roof control error:", str(e))
        return False

def automatic_control():
    """Automatic control based on temperature and humidity"""
    if not auto_mode:
        return
    
    try:
        # Temperature control logic
        if current_temp < TEMP_THRESHOLD_COLD:
            # Cold: Close roof, open valve (watering)
            control_roof(False)
            control_water_valve(True)
            
        elif current_temp > TEMP_THRESHOLD_HOT:
            # Hot: Open roof for ventilation
            control_roof(True)
            control_water_valve(False)
            
        else:
            # Normal temperature
            control_roof(False)  # Close roof by default
            control_water_valve(False)  # Close valve by default
            
            # Humidity control
            if current_humid < HUMID_THRESHOLD_LOW:
                # Low humidity: Open valve for watering
                control_water_valve(True)
            elif current_humid > HUMID_THRESHOLD_HIGH:
                # High humidity: Close valve
                control_water_valve(False)
                
    except Exception as e:
        print("❌ Automatic control error:", str(e))

def mqtt_callback(topic, msg):
    """MQTT message callback"""
    global last_activity, current_temp, current_humid, last_data_time
    last_activity = time.ticks_ms()
    
    try:
        if not topic or not msg:
            return
            
        topic_str = topic.decode('utf-8', 'ignore')
        message = msg.decode('utf-8', 'ignore')
        
        print(f"[MQTT] Receive: {topic_str} = '{message}'")
        
        if topic_str == "garden/sensor/temp_humidity":
            # Parse JSON data from sensor
            try:
                data = json.loads(message)
                current_temp = float(data.get("temperature", 0.0))
                current_humid = float(data.get("humidity", 0.0))
                last_data_time = time.ticks_ms()
                
                print(f"📊 Temp: {current_temp:.1f}°C, Humid: {current_humid:.1f}%")
                
                # Update LCD
                lcd_update_display()
                
                # Automatic control if enabled
                if auto_mode:
                    automatic_control()
                
            except json.JSONDecodeError:
                print("❌ Invalid JSON data:", message)
                
    except Exception as e:
        print("❌ MQTT callback error:", str(e))

def setup_mqtt():
    """Setup MQTT connection"""
    global mqtt_client, mqtt_connected
    
    try:
        if not wifi_connected:
            print("❌ MQTT: WiFi not available")
            return False
        
        print("MQTT: Connecting to broker...")
        
        mqtt_client = MQTTClient(
            CLIENT_ID,
            MQTT_SERVER,
            MQTT_PORT,
            keepalive=60
        )
        
        mqtt_client.set_callback(mqtt_callback)
        
        # Connect with retry
        max_retries = 3
        for attempt in range(max_retries):
            try:
                mqtt_client.connect(clean_session=True)
                mqtt_connected = True
                print("✅ MQTT: Connected successfully")
                
                # Subscribe to sensor data topic
                mqtt_client.subscribe(SUBSCRIBE_TOPIC)
                print(f"MQTT: Subscribed to {SUBSCRIBE_TOPIC.decode()}")
                
                # Send initial status
                mqtt_client.publish(STATUS_TOPIC, b"ONLINE")
                
                # Send device info
                info = {
                    "device_id": CLIENT_ID,
                    "type": "temp_humidity_controller",
                    "version": "2.0",
                    "auto_mode": auto_mode,
                    "ip": wifi.ifconfig()[0],
                    "uptime": time.ticks_ms(),
                    "thresholds": {
                        "temp_cold": TEMP_THRESHOLD_COLD,
                        "temp_hot": TEMP_THRESHOLD_HOT,
                        "humid_low": HUMID_THRESHOLD_LOW,
                        "humid_high": HUMID_THRESHOLD_HIGH
                    }
                }
                info_str = json.dumps(info)
                mqtt_client.publish(b"garden/system/device_info", info_str.encode())
                
                return True
                
            except Exception as conn_error:
                print(f"❌ MQTT connection attempt {attempt+1} failed:", str(conn_error))
                if attempt < max_retries - 1:
                    print("MQTT: Retrying in 3 seconds...")
                    time.sleep(3)
                else:
                    mqtt_connected = False
                    return False
                    
    except Exception as e:
        mqtt_connected = False
        print("❌ MQTT setup error:", str(e))
        return False

def setup_hardware():
    """Setup all hardware components"""
    global led, switch_valve, switch_roof, switch_mode
    
    try:
        print("=== Smart Garden Temperature & Humidity Control ===")
        
        # Initialize LED
        led = Pin(LED_PIN, Pin.OUT)
        led.off()
        print("✅ LED: Initialized")
        
        # Setup manual switches with pull-up resistors
        switch_valve = Pin(SWITCH_VALVE_PIN, Pin.IN, Pin.PULL_UP)
        switch_roof = Pin(SWITCH_ROOF_PIN, Pin.IN, Pin.PULL_UP)
        switch_mode = Pin(SWITCH_MODE_PIN, Pin.IN, Pin.PULL_UP)
        print("✅ Switches: Initialized")
        
        # Setup LCD
        if setup_lcd():
            lcd.clear()
            lcd.putstr("Initializing...")
        
        # Setup servos
        setup_servos()
        
        print("✅ Hardware: All components ready")
        return True
        
    except Exception as e:
        print("❌ Hardware setup error:", str(e))
        return False

def check_manual_switches():
    """Check manual control switches"""
    try:
        global auto_mode
        
        # Check mode switch (pressed = auto, released = manual)
        if switch_mode.value() == 0:  # Switch pressed
            if not auto_mode:
                auto_mode = True
                print("🔄 Mode: Switched to AUTO")
                lcd_update_display()
        
        # Check valve and roof switches only in manual mode
        if not auto_mode:
            # Check valve switch
            if switch_valve.value() == 0:  # Switch pressed
                control_water_valve(True)
                
            # Check roof switch  
            if switch_roof.value() == 0:  # Switch pressed
                control_roof(True)
                
    except Exception as e:
        print("❌ Manual switch check error:", str(e))

def send_status_report():
    """Send status report to MQTT"""
    try:
        if not mqtt_connected:
            return
            
        status = {
            "device_id": CLIENT_ID,
            "status": "ONLINE",
            "auto_mode": auto_mode,
            "current_temperature": current_temp,
            "current_humidity": current_humid,
            "valve_state": valve_state,
            "roof_state": roof_state,
            "last_data_received": last_data_time,
            "uptime": time.ticks_ms(),
            "mqtt_connected": mqtt_connected,
            "wifi_connected": wifi_connected
        }
        
        status_str = json.dumps(status)
        mqtt_client.publish(STATUS_TOPIC, status_str.encode())
        
        # Also send to MongoDB
        mongo_status = {
            "device_id": CLIENT_ID,
            "sensor_type": "control_status",
            "temperature": current_temp,
            "humidity": current_humid,
            "valve_position": valve_state,
            "roof_position": roof_state,
            "control_mode": "AUTO" if auto_mode else "MANUAL",
            "timestamp": time.ticks_ms()
        }
        
        mongo_str = json.dumps(mongo_status)
        mqtt_client.publish(MONGO_TOPIC, mongo_str.encode())
        
        print("📊 Status: Report sent to MongoDB")
        
    except Exception as e:
        print("❌ Status report error:", str(e))

def main():
    """Main program loop"""
    global last_activity, mqtt_connected, wifi_connected
    
    print("🚀 Starting Temperature & Humidity Control System...")
    
    gc.collect()
    
    # Setup all hardware
    if not setup_hardware():
        print("❌ Hardware setup failed. Retrying...")
        time.sleep(5)
        main()
        return
    
    # Setup WiFi
    if not setup_wifi():
        print("❌ WiFi: Failed to connect. Continuing offline...")
    
    # Setup MQTT
    if not setup_mqtt():
        print("❌ MQTT: Failed to connect. Will retry in main loop...")
    
    print("✅ Temperature & Humidity Control System Ready!")
    
    # Main loop variables
    last_status_report = time.ticks_ms()
    last_lcd_update = time.ticks_ms()
    last_heartbeat = time.ticks_ms()
    
    while True:
        try:
            current_time = time.ticks_ms()
            
            # MQTT processing
            if mqtt_connected and mqtt_client:
                try:
                    mqtt_client.check_msg()
                except Exception as mqtt_error:
                    print(f"❌ MQTT processing error: {str(mqtt_error)}")
                    mqtt_connected = False
                    
            # MQTT reconnection if needed
            if not mqtt_connected:
                if time.ticks_diff(current_time, last_heartbeat) > 60000:  # Retry every minute
                    reconnect_mqtt()
                    last_heartbeat = current_time
            
            # Check manual switches
            check_manual_switches()
            
            # Update LCD every 5 seconds
            if time.ticks_diff(current_time, last_lcd_update) > 5000:
                lcd_update_display()
                last_lcd_update = current_time
            
            # Status report every 5 minutes
            if time.ticks_diff(current_time, last_status_report) > 300000:
                send_status_report()
                last_status_report = current_time
            
            # Connection heartbeat every 30 seconds
            if mqtt_connected:
                mqtt_client.ping()
            
            time.sleep_ms(100)
            
        except KeyboardInterrupt:
            print("\n🛑 Program interrupted by user")
            break
        except Exception as e:
            print("❌ Main loop error:", str(e))
            print("🔄 Attempting recovery...")
            time.sleep(2)

def reconnect_mqtt():
    """MQTT reconnection with exponential backoff"""
    global mqtt_connected
    
    try:
        if mqtt_connected:
            return True
            
        print("MQTT: Attempting reconnection...")
        
        if setup_mqtt():
            print("✅ MQTT: Reconnected successfully")
            return True
        else:
            return False
            
    except Exception as e:
        print("❌ MQTT reconnection error:", str(e))
        return False

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("💥 Fatal error:", str(e))
        print("🔄 System will reboot in 10 seconds...")
        time.sleep(10)
        machine.reset()
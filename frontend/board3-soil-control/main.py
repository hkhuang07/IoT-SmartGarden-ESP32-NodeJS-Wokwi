#!/usr/bin/env python3
"""
Smart Garden Soil Moisture Control Board - AUTO CONTROL INTEGRATED
ESP32 with LCD, Servos, MQTT - MicroPython
Receives soil moisture data from MQTT and controls water valve servo
"""

import network
import time
import json
import gc
import machine
from machine import Pin, PWM, I2C, unique_id, Timer
from umqtt.simple import MQTTClient

# ===== CONFIGURATION =====
WIFI_SSID = "Wokwi-GUEST"
WIFI_PASSWORD = ""

MQTT_SERVER = "broker.hivemq.com"
MQTT_PORT = 1883
MQTT_CLIENT_ID = "ESP_CONTROL_SOIL_MOISTURE_LOGIC_01"

# MQTT Topics
MQTT_SENSOR_DATA_TOPIC = "garden/sensor/soil_moisture_data"
MQTT_CONTROL_COMMANDS_TOPIC = "garden/control/soil_commands"
MQTT_STATUS_TOPIC = "garden/status/soil_control_board"

# Servo Configuration (10-bit duty cycle: 0-1023)
WATER_SERVO_PIN = 15 

# Dải Duty Cycle (40-115)
WATER_CLOSE_DUTY = 40   # ~1.0ms pulse (Valve closed)
WATER_OPEN_DUTY = 115   # ~2.0ms pulse (Valve open)

SERVO_FREQ = 50 

# LCD Configuration (I2C)
LCD_I2C_ADDRESS = 0x27
LCD_I2C_SDA = 21
LCD_I2C_SCL = 22

# Pins
STATUS_LED_PIN = 4 
BUILTIN_LED_PIN = 2 
WATER_BUTTON_PIN = 25 
MODE_BUTTON_PIN = 27

# Soil Moisture Thresholds
THRESHOLD = {
    "moisture": {
        "critical": 30.0,  # < 30% = Critical (Open valve fully)
        "low": 45.0,       # < 45% = Low (Open valve partially)
        "optimal": 60.0    # >= 60% = Optimal (Close valve)
    }
}

# ===== GLOBAL VARIABLES =====
mqtt_client = None
mqtt_connected = False
wifi_connected = False
lcd = None
servo_water = None

water_percentage = 0
auto_mode = True 
current_moisture = 0.0
last_moisture_update = time.ticks_ms()

mqtt_messages_received = 0
system_start_time = time.ticks_ms()
led_state = False
led_blink_last = time.ticks_ms()
last_status_print = time.ticks_ms()

water_button = Pin(WATER_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
mode_button = Pin(MODE_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
status_led = None
builtin_led = None

class I2CLCD1602:
    """Simple I2C LCD driver"""
    
    def __init__(self, i2c, addr=0x27, cols=16, rows=2):
        self.i2c = i2c
        self.addr = addr
        self.cols = cols
        self.rows = rows
        self._init()
    
    def _init(self):
        """Initialize LCD"""
        init_cmds = [0x33, 0x32, 0x28, 0x0C, 0x06, 0x01]
        for cmd in init_cmds:
            self._send_cmd(cmd)
            time.sleep_ms(5)
    
    def _send_cmd(self, cmd):
        """Send command to LCD"""
        high_nib = cmd & 0xF0
        low_nib = (cmd << 4) & 0xF0
        self._i2c_write(high_nib | 0x04)
        self._i2c_write(high_nib)
        self._i2c_write(low_nib | 0x04)
        self._i2c_write(low_nib)
    
    def _send_data(self, data):
        """Send data to LCD"""
        high_nib = data & 0xF0
        low_nib = (data << 4) & 0xF0
        self._i2c_write(high_nib | 0x05)
        self._i2c_write(high_nib | 0x01)
        self._i2c_write(low_nib | 0x05)
        self._i2c_write(low_nib | 0x01)
    
    def _i2c_write(self, data):
        """Write to I2C"""
        data_with_backlight = data | 0x08
        self.i2c.writeto(self.addr, bytes([data_with_backlight]))
        time.sleep_us(50)
    
    def clear(self):
        """Clear display"""
        self._send_cmd(0x01)
        time.sleep_ms(2)
    
    def putstr(self, string):
        """Print string to display"""
        for char in string:
            if 32 <= ord(char) <= 126:
                self._send_data(ord(char))
    
    def move_to(self, col, row):
        """Move cursor to position"""
        if row == 0:
            self._send_cmd(0x80 | col)
        elif row == 1:
            self._send_cmd(0xC0 | col)

def setup_lcd():
    """Setup LCD display"""
    global lcd
    try:
        print("📱 Setting up LCD...")
        i2c = I2C(0, scl=Pin(LCD_I2C_SCL), sda=Pin(LCD_I2C_SDA))
        devices = i2c.scan()
        if LCD_I2C_ADDRESS in devices:
            lcd = I2CLCD1602(i2c, addr=LCD_I2C_ADDRESS)
            lcd.clear()
            lcd.putstr("Soil Control...")
            print("✅ LCD initialized")
            return True
        else:
            print("❌ LCD not found")
            return False
    except Exception as e:
        print(f"❌ LCD setup error: {e}")
        return False

def update_lcd_display():
    """Update LCD with current values"""
    global lcd, mqtt_connected, water_percentage, current_moisture
    
    if not lcd:
        return
        
    try:
        lcd.clear()
        
        # Line 1: Moisture and Mode
        mode_str = "A" if auto_mode else "M"
        line1 = f"[{mode_str}] M:{current_moisture:.0f}%"
        lcd.move_to(0, 0)
        lcd.putstr(line1[:16])
        
        # Line 2: Valve status
        valve_state = "OPEN" if water_percentage == 100 else "PART" if water_percentage > 0 else "CLOSE"
        line2 = f"W:{water_percentage}% {valve_state}"
        line2 += " ON" if mqtt_connected else " OFF"
        
        lcd.move_to(0, 1)
        lcd.putstr(line2[:16])
        
    except Exception as e:
        print(f"❌ LCD update error: {e}")

# ===== SERVO FUNCTIONS =====

def setup_servos():
    """Setup servo motors"""
    global servo_water
    try:
        print("🔧 Setting up servos...")
        
        # --- SERVO WATER (Pin 15) ---
        water_pin = Pin(WATER_SERVO_PIN)
        servo_water = PWM(water_pin, freq=SERVO_FREQ, duty=WATER_CLOSE_DUTY) 
        
        print(f"✅ Servos initialized (Closed) using 10-bit [{WATER_CLOSE_DUTY}-{WATER_OPEN_DUTY}].")
        return True
    except Exception as e:
        print(f"❌ Servo setup error: {e}")
        return False

def set_water_percentage(percentage):
    """Set water valve percentage (0-100)"""
    global water_percentage, servo_water
    
    if not servo_water: 
        return
        
    percentage = max(0, min(100, percentage))
    
    # Skip if no change
    if percentage == water_percentage:
        return 
    
    water_percentage = percentage
    
    # Calculate duty: Map 0-100% to safe range (CLOSE_DUTY - OPEN_DUTY)
    duty = WATER_CLOSE_DUTY + (percentage * (WATER_OPEN_DUTY - WATER_CLOSE_DUTY) / 100)
    
    servo_water.duty(int(duty)) 
    
    # Log action
    if percentage == 100:
        action = "FULLY OPEN"
    elif percentage == 50:
        action = "PARTIAL OPEN"
    elif percentage == 0:
        action = "CLOSED"
    else:
        action = f"{percentage}%"
    
    print(f"💧 Water Valve: {action} (Duty: {int(duty)})")
    update_lcd_display()


def mqtt_callback(topic, msg):
    """MQTT callback function for incoming messages"""
    global mqtt_messages_received, current_moisture, last_moisture_update
    global water_percentage, auto_mode
    
    try:
        topic_str = topic.decode('utf-8')
        msg_str = msg.decode('utf-8')
        mqtt_messages_received += 1
        
        print(f"📨 MQTT: [{topic_str}]: {msg_str}")
        
        if topic_str == MQTT_SENSOR_DATA_TOPIC:
            try:
                data = json.loads(msg_str)
                moisture = data.get('moisture_value')
                
                if moisture is not None:
                    current_moisture = float(moisture)
                    last_moisture_update = time.ticks_ms()
                    print(f"📊 Soil Moisture: {current_moisture:.1f}%")
                    
                    if auto_mode:
                        auto_control_logic() 
                    
                    update_lcd_display()
                else:
                    print("⚠️ Sensor data missing moisture_value key.")

            except Exception as e:
                print(f"❌ Sensor data parse error: {e}")
                
        elif topic_str == MQTT_CONTROL_COMMANDS_TOPIC:
            try:
                command = json.loads(msg_str)
                cmd_type = command.get('command', '')
                
                if cmd_type == 'set_auto_mode':
                    auto_mode = command.get('auto_mode', True)
                    print(f"🔧 Auto mode: {'ON' if auto_mode else 'OFF'}")
                    update_lcd_display()
                    
                elif cmd_type == 'control_pump' or cmd_type == 'water_valve':
                    auto_mode = False 
                    activate = command.get('activate', False)
                    percentage = command.get('value', 100 if activate else 0)
                    set_water_percentage(percentage)
                    print(f"🔧 Manual control: Valve {percentage}%")
                    
            except Exception as e:
                print(f"❌ Command parse error: {e}")
                
    except Exception as e:
        print(f"❌ MQTT message handler error: {e}")


def auto_control_logic():
    """Automatic control logic based on soil moisture"""
    global current_moisture, water_percentage
    
    if not auto_mode:
        return
    
    # Critical: Moisture < 30% → Open valve fully (100%)
    if current_moisture < THRESHOLD["moisture"]["critical"]:
        print(f"🚨 CRITICAL: Moisture LOW ({current_moisture:.1f}% < {THRESHOLD['moisture']['critical']}%). FULLY OPEN Valve")
        if water_percentage != 100:
            set_water_percentage(100)
    
    # Low: Moisture 30-45% → Open valve partially (50%)
    elif current_moisture < THRESHOLD["moisture"]["low"]:
        print(f"⚠️ LOW: Moisture ({current_moisture:.1f}% < {THRESHOLD['moisture']['low']}%). PARTIAL OPEN Valve")
        if water_percentage != 50:
            set_water_percentage(50)
    
    # Optimal: Moisture >= 60% → Close valve (0%)
    elif current_moisture >= THRESHOLD["moisture"]["optimal"]:
        print(f"✅ OPTIMAL: Moisture HIGH ({current_moisture:.1f}% >= {THRESHOLD['moisture']['optimal']}%). CLOSE Valve")
        if water_percentage != 0:
            set_water_percentage(0)
    
    # Between 45-60%: Maintain current state (no action)
    else:
        print(f"🔄 STABLE: Moisture ({current_moisture:.1f}%) in range 45-60%. No action needed.")


def setup_wifi():
    global wifi_connected
    try:
        print("🌐 Setting up WiFi...")
        wifi = network.WLAN(network.STA_IF)
        wifi.active(True)
        wifi.connect(WIFI_SSID, WIFI_PASSWORD)
        max_wait = 10
        while max_wait > 0:
            if wifi.isconnected():
                break
            max_wait -= 1
            time.sleep(1)
        
        if wifi.isconnected():
            wifi_connected = True
            print(f"✅ WiFi connected: {wifi.ifconfig()[0]}")
            return True
        else:
            print("❌ WiFi connection failed")
            return False
    except Exception as e:
        print(f"❌ WiFi setup error: {e}")
        return False

def setup_mqtt():
    global mqtt_client, mqtt_connected
    try:
        print("📡 Setting up MQTT client...")
        mqtt_client = MQTTClient(MQTT_CLIENT_ID, MQTT_SERVER, MQTT_PORT, keepalive=60)
        mqtt_client.set_callback(mqtt_callback)
        print("🔌 Attempting MQTT connection...")
        mqtt_client.connect(clean_session=True)
        
        print("✅ MQTT connected successfully!")
        mqtt_connected = True
        
        # Subscribe to sensor data topic
        mqtt_client.subscribe(MQTT_SENSOR_DATA_TOPIC.encode())
        mqtt_client.subscribe(MQTT_CONTROL_COMMANDS_TOPIC.encode())
        
        print(f"✅ Subscribed to: {MQTT_SENSOR_DATA_TOPIC}")
        print(f"✅ Subscribed to: {MQTT_CONTROL_COMMANDS_TOPIC}")
        return True
    except Exception as e:
        print(f"❌ MQTT setup failed: {e}")
        mqtt_connected = False
        return False

def check_mqtt_connection():
    global mqtt_client, mqtt_connected
    
    if not mqtt_client: 
        return False
    
    try:
        if mqtt_connected:
            try:
                mqtt_client.check_msg()
            except Exception as e:
                mqtt_connected = False
                try:
                    mqtt_client.reconnect()
                    mqtt_connected = True
                    mqtt_client.subscribe(MQTT_SENSOR_DATA_TOPIC.encode())
                    mqtt_client.subscribe(MQTT_CONTROL_COMMANDS_TOPIC.encode())
                    print("✅ MQTT reconnected!")
                except Exception as re:
                    return False
        else:
            if setup_mqtt():
                return True
            else:
                return False
        return mqtt_connected
    except Exception as e:
        print(f"❌ MQTT connection check error: {e}")
        return False

def setup_leds():
    global status_led, builtin_led
    try:
        print("💡 Setting up LEDs...")
        status_led = Pin(STATUS_LED_PIN, Pin.OUT)
        builtin_led = Pin(BUILTIN_LED_PIN, Pin.OUT)
        status_led.off()
        builtin_led.off()
        print("✅ LEDs initialized")
        return True
    except Exception as e:
        print(f"❌ LED setup error: {e}")
        return False

def update_leds():
    global led_state, led_blink_last
    try:
        current_time = time.ticks_ms()
        if current_time - led_blink_last > 1000:
            if mqtt_connected:
                status_led.on()
                builtin_led.on()
            else:
                led_state = not led_state
                status_led.value(led_state)
                builtin_led.value(led_state)
            led_blink_last = current_time
    except Exception as e:
        pass

def print_system_status():
    global current_moisture, water_percentage, auto_mode, mqtt_connected, system_start_time
    try:
        uptime = (time.ticks_ms() - system_start_time) // 1000
        
        print("\n🔄 SYSTEM STATUS:")
        print(f"  MQTT: {'✅ CONNECTED' if mqtt_connected else '❌ DISCONNECTED'}")
        print(f"  Soil Moisture: {current_moisture:.1f}%")
        print(f"  Water Valve: {water_percentage}%")
        print(f"  Uptime: {uptime}s")
        print(f"  Auto Mode: {'ON' if auto_mode else 'OFF'}")
        print("=" * 60)
        
    except Exception as e:
        print(f"❌ Status print error: {e}")
        
def main():
    global system_start_time, last_status_print
    
    try:
        print("🌱 Smart Garden Soil Moisture Control Board")
        
        system_start_time = time.ticks_ms()
        last_status_print = time.ticks_ms()
        
        # Initialize hardware
        print("🔧 Initializing hardware...")
        if not setup_lcd(): 
            print("⚠️ LCD failed, continuing without LCD")
        if not setup_servos(): 
            return
        if not setup_leds(): 
            return
        
        # Setup network connections
        print("\n🌐 Setting up network connections...")
        if not setup_wifi(): 
            return
        if not setup_mqtt(): 
            print("⚠️ MQTT setup failed, will retry in background")
        
        print("\n🚀 System ready!")
        
        while True:
            try:
                current_time = time.ticks_ms()
                check_mqtt_connection()
                
                if mqtt_client and mqtt_connected:
                    mqtt_client.check_msg() 
                    
                # Auto control logic runs when moisture data is fresh
                if auto_mode and (current_time - last_moisture_update) < 60000:
                    pass  # Logic is triggered by MQTT callback
                
                update_leds()
                
                if current_time % 2000 < 100: 
                    update_lcd_display()
                    
                if current_time - last_status_print > 30000:
                    print_system_status() 
                    last_status_print = current_time
                    
                if current_time % 60000 < 1000: 
                    gc.collect()
                
                time.sleep_ms(100)
                
            except KeyboardInterrupt:
                print("\n🛑 System interrupted by user")
                break
            except Exception as e:
                print(f"❌ Main loop error: {e}")
                time.sleep_ms(1000)
                
    except Exception as e:
        print(f"❌ System error: {e}")
    finally:
        cleanup()

def cleanup():
    try:
        print("🧹 Cleaning up...")
        if mqtt_client:
            try: 
                mqtt_client.disconnect()
            except: 
                pass
        if servo_water: 
            servo_water.deinit()
        print("✅ Cleanup complete")
    except Exception as e:
        print(f"❌ Cleanup error: {e}")

if __name__ == "__main__":
    main()
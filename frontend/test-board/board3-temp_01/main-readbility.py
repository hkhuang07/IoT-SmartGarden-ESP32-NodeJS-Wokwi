#!/usr/bin/env python3
"""
Smart Garden Control Board - WOKWI DIAGRAM VERSION
ESP32 with LCD, Servos, Buttons - FIXED FOR DIAGRAM

Based on diagram.json:
- ESP32 DevKit C V4
- 2 Servos (Water Valve: Pin 15, Roof: Pin 14) 
- 2 Buttons (Water: Pin 25, Roof: Pin 26)
- 1 LCD1602 (I2C on Pin 21, 22)
- 1 Status LED (Pin 4)
- Mode Button (Pin 27)

Author: MiniMax Agent
Version: WOKWI DIAGRAM FIXED - 2025-12-02
"""

import network
import time
import json
import gc
import machine
from machine import Pin, PWM, I2C, unique_id, Timer
from umqtt.simple import MQTTClient

# ===== WIFI CONFIGURATION =====
WIFI_SSID = "Wokwi-GUEST"
WIFI_PASSWORD = ""

# ===== MQTT CONFIGURATION - SIMPLE LIKE SUCCESSFUL BOARDS =====
MQTT_SERVER = "broker.hivemq.com"
MQTT_PORT = 1883
MQTT_CLIENT_ID = "ESP_CONTROL_TEMP_HUM_01"

# MQTT Topics
MQTT_SENSOR_DATA_TOPIC = "garden/sensor/temp_hum_data"
MQTT_CONTROL_COMMANDS_TOPIC = "garden/control/commands"
MQTT_STATUS_TOPIC = "garden/status/control_board"

# ===== LCD CONFIGURATION (I2C) =====
LCD_I2C_ADDRESS = 0x27
LCD_I2C_SDA = 21
LCD_I2C_SCL = 22

# ===== HARDWARE PINS (EXACTLY AS PER DIAGRAM) =====
STATUS_LED_PIN = 4         # LED from diagram (Pin 4 via resistor R1)
BUILTIN_LED_PIN = 2        # Built-in ESP32 LED

# Servos (from diagram)
WATER_SERVO_PIN = 15       # Water valve servo (Pin 15)
ROOF_SERVO_PIN = 14        # Roof servo (Pin 14)

# Buttons (from diagram)  
WATER_BUTTON_PIN = 25      # Water valve button (Pin 25)
ROOF_BUTTON_PIN = 26       # Roof button (Pin 26)
MODE_BUTTON_PIN = 27       # Mode button (Pin 27)

# ===== SERVO CONFIGURATION =====
WATER_OPEN_DUTY = 128      # 100% water valve open
WATER_CLOSE_DUTY = 26      # 0% water valve closed
ROOF_OPEN_DUTY = 128       # 100% roof open
ROOF_CLOSE_DUTY = 26       # 0% roof closed
SERVO_FREQ = 50            # 50Hz PWM frequency

# ===== GLOBAL VARIABLES =====
mqtt_client = None
mqtt_connected = False
wifi_connected = False

lcd = None
servo_water = None
servo_roof = None

# Control state
water_percentage = 0
roof_percentage = 0
auto_mode = False
current_temperature = 0.0
current_humidity = 0.0

# System variables
mqtt_messages_received = 0
system_start_time = time.ticks_ms()

# LED variables (FIXED - ensure these are initialized)
led_state = False
led_blink_last = time.ticks_ms()
last_status_print = time.ticks_ms()

# ===== MQTT CALLBACK =====
def mqtt_callback(topic, msg):
    """MQTT callback function for incoming messages"""
    global mqtt_messages_received, current_temperature, current_humidity
    global water_percentage, roof_percentage
    
    try:
        topic_str = topic.decode('utf-8')
        msg_str = msg.decode('utf-8')
        mqtt_messages_received += 1
        
        print(f"📨 MQTT: [{topic_str}]: {msg_str}")
        
        if topic_str == MQTT_SENSOR_DATA_TOPIC:
            # Parse sensor data
            try:
                data = json.loads(msg_str)
                current_temperature = data.get('temperature', 0.0)
                current_humidity = data.get('humidity', 0.0)
                print(f"📊 Sensor data: T={current_temperature:.1f}°C, H={current_humidity:.1f}%")
                update_lcd_display()
            except Exception as e:
                print(f"❌ Sensor data parse error: {e}")
                
        elif topic_str == MQTT_CONTROL_COMMANDS_TOPIC:
            # Parse control commands
            try:
                command = json.loads(msg_str)
                cmd_type = command.get('command', '')
                
                if cmd_type == 'water_valve':
                    percentage = command.get('value', 0)
                    set_water_percentage(percentage)
                    print(f"💧 Water valve set to {percentage}%")
                    
                elif cmd_type == 'roof':
                    percentage = command.get('value', 0)
                    set_roof_percentage(percentage)
                    print(f"🏠 Roof set to {percentage}%")
                    
                elif cmd_type == 'status_request':
                    publish_status("STATUS_RESPONSE")
                    
            except Exception as e:
                print(f"❌ Command parse error: {e}")
                
    except Exception as e:
        print(f"❌ MQTT message handler error: {e}")

# ===== MQTT FUNCTIONS =====

def setup_wifi():
    """Setup WiFi connection - Simple approach"""
    global wifi_connected
    try:
        print("🌐 Setting up WiFi...")
        wifi = network.WLAN(network.STA_IF)
        wifi.active(True)
        wifi.connect(WIFI_SSID, WIFI_PASSWORD)
        
        # Wait for connection
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
    """Setup MQTT client - SIMPLE like successful boards"""
    global mqtt_client, mqtt_connected
    
    try:
        print("📡 Setting up MQTT client...")
        print(f"🔧 MQTT Server: {MQTT_SERVER}:{MQTT_PORT}")
        print(f"🔧 Client ID: {MQTT_CLIENT_ID}")
        
        # Create MQTT client
        mqtt_client = MQTTClient(MQTT_CLIENT_ID, MQTT_SERVER, MQTT_PORT, keepalive=60)
        
        # Set callback BEFORE connecting (FIXED)
        mqtt_client.set_callback(mqtt_callback)
        
        # Connect
        print("🔌 Attempting MQTT connection...")
        mqtt_client.connect(clean_session=True)
        
        print("✅ MQTT connected successfully!")
        mqtt_connected = True
        
        # Subscribe to topics
        mqtt_client.subscribe(MQTT_SENSOR_DATA_TOPIC.encode())
        mqtt_client.subscribe(MQTT_CONTROL_COMMANDS_TOPIC.encode())
        
        print(f"✅ Subscribed to: {MQTT_SENSOR_DATA_TOPIC}")
        print(f"✅ Subscribed to: {MQTT_CONTROL_COMMANDS_TOPIC}")
        
        # Publish initial status
        publish_status("MQTT_CONNECTED")
        
        return True
        
    except Exception as e:
        print(f"❌ MQTT setup failed: {e}")
        mqtt_connected = False
        return False

def check_mqtt_connection():
    """Check and maintain MQTT connection - FIXED VERSION"""
    global mqtt_client, mqtt_connected
    
    if not mqtt_client:
        return False
    
    try:
        if mqtt_connected:
            # Use check_msg() to verify connection (FIXED)
            try:
                mqtt_client.check_msg()
            except Exception as e:
                print(f"⚠️ MQTT connection check failed: {e}")
                mqtt_connected = False
                
                # Attempt reconnection
                try:
                    mqtt_client.reconnect()
                    mqtt_connected = True
                    print("✅ MQTT reconnected!")
                    
                    # Resubscribe to topics
                    mqtt_client.subscribe(MQTT_SENSOR_DATA_TOPIC.encode())
                    mqtt_client.subscribe(MQTT_CONTROL_COMMANDS_TOPIC.encode())
                    
                except Exception as re:
                    print(f"❌ MQTT reconnection failed: {re}")
                    return False
        else:
            # Attempt initial connection
            if setup_mqtt():
                return True
            else:
                return False
                
        return mqtt_connected
        
    except Exception as e:
        print(f"❌ MQTT connection check error: {e}")
        return False

def publish_status(status):
    """Publish status to MQTT"""
    global mqtt_client, mqtt_connected
    
    if not mqtt_client or not mqtt_connected:
        return
        
    try:
        status_data = {
            "device_id": MQTT_CLIENT_ID,
            "status": status,
            "temperature": current_temperature,
            "humidity": current_humidity,
            "water_valve": water_percentage,
            "roof": roof_percentage,
            "auto_mode": auto_mode,
            "uptime": time.ticks_ms() - system_start_time
        }
        
        message = json.dumps(status_data)
        mqtt_client.publish(MQTT_STATUS_TOPIC.encode(), message.encode())
        
    except Exception as e:
        print(f"❌ Status publish error: {e}")

# ===== LCD FUNCTIONS =====

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
        print(f"📋 I2C devices found: {[hex(d) for d in devices]}")
        
        if LCD_I2C_ADDRESS in devices:
            lcd = I2CLCD1602(i2c, addr=LCD_I2C_ADDRESS)
            lcd.clear()
            lcd.putstr("Smart Garden")
            lcd.move_to(0, 1)
            lcd.putstr("Control Ready")
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
    global lcd, mqtt_connected, water_percentage, roof_percentage
    
    if not lcd:
        return
        
    try:
        lcd.clear()
        time.sleep_ms(10)
        
        # Line 1: Temperature and Humidity
        temp_str = f"{current_temperature:.1f}" if current_temperature else "0.0"
        hum_str = f"{current_humidity:.0f}" if current_humidity else "0"
        line1 = f"T{temp_str} H{hum_str}"
        lcd.move_to(0, 0)
        lcd.putstr(line1[:16])
        
        # Line 2: Status and values
        status = "ON" if mqtt_connected else "OFF"
        line2 = f"{status} W:{water_percentage}% R:{roof_percentage}%"
        lcd.move_to(0, 1)
        lcd.putstr(line2[:16])
        
        print(f"📺 LCD: '{line1}' | '{line2}'")
        
    except Exception as e:
        print(f"❌ LCD update error: {e}")

# ===== SERVO FUNCTIONS =====

def setup_servos():
    """Setup servo motors"""
    global servo_water, servo_roof
    
    try:
        print("🔧 Setting up servos...")
        
        # Water valve servo (Pin 15)
        water_pin = Pin(WATER_SERVO_PIN)
        servo_water = PWM(water_pin)
        servo_water.freq(SERVO_FREQ)
        servo_water.duty(WATER_CLOSE_DUTY)
        
        # Roof servo (Pin 14) 
        roof_pin = Pin(ROOF_SERVO_PIN)
        servo_roof = PWM(roof_pin)
        servo_roof.freq(SERVO_FREQ)
        servo_roof.duty(ROOF_CLOSE_DUTY)
        
        print("✅ Servos initialized")
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
    water_percentage = percentage
    
    duty = WATER_CLOSE_DUTY + (percentage * (WATER_OPEN_DUTY - WATER_CLOSE_DUTY) / 100)
    servo_water.duty(int(duty))
    
    status = "MỞ" if percentage > 0 else "ĐÓNG"
    print(f"💧 Water: {percentage}% ({status}) duty:{int(duty)}")
    
    update_lcd_display()

def set_roof_percentage(percentage):
    """Set roof percentage (0-100)"""
    global roof_percentage, servo_roof
    
    if not servo_roof:
        return
        
    percentage = max(0, min(100, percentage))
    roof_percentage = percentage
    
    duty = ROOF_CLOSE_DUTY + (percentage * (ROOF_OPEN_DUTY - ROOF_CLOSE_DUTY) / 100)
    servo_roof.duty(int(duty))
    
    status = "MỞ" if percentage > 0 else "ĐÓNG"
    print(f"🏠 Roof: {percentage}% ({status}) duty:{int(duty)}")
    
    update_lcd_display()

# ===== BUTTON FUNCTIONS =====

def setup_buttons():
    """Setup button inputs"""
    global water_button, roof_button, mode_button
    
    try:
        print("🔘 Setting up buttons...")
        
        # From diagram: Pin 25, 26, 27
        water_button = Pin(WATER_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
        roof_button = Pin(ROOF_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
        mode_button = Pin(MODE_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
        
        print("✅ Buttons initialized")
        print("  - Button 1 (Pin 25): Toggle water valve")
        print("  - Button 2 (Pin 26): Toggle roof")
        print("  - Button 3 (Pin 27): No function")
        return True
        
    except Exception as e:
        print(f"❌ Button setup error: {e}")
        return False

def check_buttons():
    """Check button states"""
    global water_percentage, roof_percentage
    
    try:
        # Water button (Pin 25, active low)
        if water_button and not water_button.value():
            print("🔘 Button 1: Water valve toggle")
            
            if water_percentage == 0:
                set_water_percentage(100)
            else:
                set_water_percentage(0)
            
            time.sleep_ms(300)  # Debounce
        
        # Roof button (Pin 26, active low)
        if roof_button and not roof_button.value():
            print("🔘 Button 2: Roof toggle")
            
            if roof_percentage == 0:
                set_roof_percentage(100)
            else:
                set_roof_percentage(0)
            
            time.sleep_ms(300)  # Debounce
        
        # Mode button (Pin 27, active low) - NO FUNCTION
        if mode_button and not mode_button.value():
            print("🔘 Button 3: No function (ignored)")
            time.sleep_ms(300)  # Debounce
            
    except Exception as e:
        print(f"❌ Button check error: {e}")

# ===== LED FUNCTIONS =====

def setup_leds():
    """Setup LEDs (Based on diagram - Pin 4)"""
    global status_led, builtin_led
    
    try:
        print("💡 Setting up LEDs...")
        
        # From diagram: LED on Pin 4 (via resistor R1)
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
    """Update LED status - FIXED VERSION"""
    global led_state, led_blink_last
    
    try:
        current_time = time.ticks_ms()
        
        if current_time - led_blink_last > 1000:
            if mqtt_connected:
                # Solid on when MQTT connected
                status_led.on()
                builtin_led.on()
            else:
                # Blink when not connected
                led_state = not led_state
                status_led.value(led_state)
                builtin_led.value(led_state)
            
            led_blink_last = current_time
            
    except Exception as e:
        print(f"❌ LED update error: {e}")

# ===== SYSTEM FUNCTIONS =====

def print_system_status():
    """Print system status"""
    try:
        uptime = (time.ticks_ms() - system_start_time) // 1000
        
        print("\n🔄 SYSTEM STATUS:")
        print(f"  MQTT: {'✅ CONNECTED' if mqtt_connected else '❌ DISCONNECTED'}")
        print(f"  WiFi: {'✅ CONNECTED' if wifi_connected else '❌ DISCONNECTED'}")
        print(f"  Temperature: {current_temperature:.1f}°C")
        print(f"  Humidity: {current_humidity:.1f}%")
        print(f"  Water Valve: {water_percentage}%")
        print(f"  Roof: {roof_percentage}%")
        print(f"  Messages: {mqtt_messages_received}")
        print(f"  Uptime: {uptime}s")
        print(f"  Auto Mode: {'ON' if auto_mode else 'OFF'}")
        print("=" * 60)
        
    except Exception as e:
        print(f"❌ Status print error: {e}")

def main():
    """Main system function"""
    global system_start_time, last_status_print
    
    try:
        print("🌱 Smart Garden Control Board - WOKWI DIAGRAM FIXED")
        print("📅 Author: MiniMax Agent")
        print("📅 Date: 2025-12-02")
        print("📊 Based on diagram.json configuration")
        print("🔧 All errors fixed - MQTT, LED, Variables")
        print()
        
        system_start_time = time.ticks_ms()
        last_status_print = time.ticks_ms()
        
        # Initialize hardware
        print("🔧 Initializing hardware...")
        
        if not setup_lcd():
            print("⚠️ LCD failed, continuing without LCD")
        
        if not setup_servos():
            print("❌ Servo setup failed")
            return
        
        if not setup_buttons():
            print("❌ Button setup failed")
            return
            
        if not setup_leds():
            print("❌ LED setup failed")
            return
        
        # Setup network connections
        print("\n🌐 Setting up network connections...")
        
        if not setup_wifi():
            print("❌ WiFi setup failed")
            return
        
        if not setup_mqtt():
            print("⚠️ MQTT setup failed, will retry in background")
        
        print("\n🚀 System ready!")
        print("📋 Features:")
        print("  ✅ FIXED MQTT connection (like successful boards)")
        print("  ✅ Button controls (Toggle water/roof)")
        print("  ✅ LCD display (I2C Pin 21,22)")
        print("  ✅ LED status (Pin 4)")
        print("  ✅ All variables initialized")
        print()
        
        # Main loop
        while True:
            try:
                current_time = time.ticks_ms()
                
                # Check MQTT connection
                check_mqtt_connection()
                
                # Check for MQTT messages
                if mqtt_client and mqtt_connected:
                    try:
                        mqtt_client.check_msg()
                    except Exception as e:
                        print(f"❌ MQTT check error: {e}")
                
                # Check buttons
                check_buttons()
                
                # Update LEDs
                update_leds()
                
                # Update LCD
                update_lcd_display()
                
                # Print status every 30 seconds
                if current_time - last_status_print > 30000:
                    print_system_status()
                    last_status_print = current_time
                
                # Clean up memory periodically
                if current_time % 60000 < 1000:  # Every minute
                    gc.collect()
                
                time.sleep_ms(100)
                
            except KeyboardInterrupt:
                print("\n🛑 System interrupted by user")
                break
            except Exception as e:
                print(f"❌ Main loop error: {e}")
                if status_led:
                    status_led.on()
                time.sleep_ms(1000)
                if status_led:
                    status_led.off()
                
    except Exception as e:
        print(f"❌ System error: {e}")
        if status_led:
            status_led.on()
    finally:
        cleanup()

def cleanup():
    """Clean up system resources"""
    try:
        print("🧹 Cleaning up...")
        
        if mqtt_client:
            try:
                mqtt_client.disconnect()
                print("✅ MQTT disconnected")
            except:
                pass
        
        if servo_water:
            servo_water.deinit()
        if servo_roof:
            servo_roof.deinit()
        
        print("✅ Cleanup complete")
        
    except Exception as e:
        print(f"❌ Cleanup error: {e}")

# ===== BOOTSTRAP =====
if __name__ == "__main__":
    main()
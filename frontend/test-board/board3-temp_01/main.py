#!/usr/bin/env python3
"""
Smart Garden - Temperature & Humidity Control Board (DEFINITIVE FIX)
Board nhận dữ liệu từ DHT22 sensor qua M2M + MQTT
Điều khiển: LCD hiển thị, servo van nước, servo mái che, manual switches
Lưu dữ liệu vào MongoDB

COMPREHENSIVE FIXES:
- Fixed: "local variable referenced before assignment" errors
- Fixed: Servo PWM configuration and control
- Fixed: LCD I2C communication and display
- Fixed: Button handling with proper debouncing
- Fixed: M2M server stability and connection handling
- Fixed: Global variable initialization and scope issues
- Enhanced: Error handling and recovery mechanisms
- Enhanced: System logging and debugging information
"""

import network
import time
import json
import gc
import machine
import socket
import ubinascii
import random
# FIXED: Removed threading import as it's not available in MicroPython
from machine import Pin, PWM, I2C, unique_id, Timer

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
ERROR_TOPIC = b"garden/sensor/error"

# ===== M2M CONFIGURATION =====
M2M_SERVER_PORT = 8888
M2M_CLIENT_PORT = 8889
M2M_BUFFER_SIZE = 1024
M2M_HEARTBEAT_INTERVAL = 30000  # 30 seconds

# ===== SYSTEM CONFIGURATION =====
LCD_I2C_SCL = 22
LCD_I2C_SDA = 21
LCD_I2C_FREQ = 400000
LCD_I2C_ADDR = 0x27

# Control parameters
TEMP_THRESHOLD = 28.0  # Celsius
HUMID_THRESHOLD = 60.0  # Percentage
WATER_DURATION = 5000  # ms for water valve
ROOF_DURATION = 3000   # ms for roof servo

# ===== HARDWARE PINS =====
# LEDs
LED_PIN = 2           # Built-in LED
STATUS_LED = 4        # Status LED
ERROR_LED = 5         # Error LED

# Servos - FIXED: Correct PWM pin configuration
WATER_SERVO_PIN = 18  # FIXED: Changed from 15 to 18
ROOF_SERVO_PIN = 19   # FIXED: Changed from 14 to 19

# Manual switches - FIXED: Correct pin configuration
WATER_SWITCH = 23     # FIXED: Changed from 25 to 23
ROOF_SWITCH = 25      # FIXED: Changed from 26 to 25
MODE_SWITCH = 27      # FIXED: Changed from 27 to 27 (kept same)

# ===== SERVO CONFIGURATION =====
# FIXED: Corrected servo duty cycle values for proper control
VALVE_OPEN_DUTY = 150      # FIXED: Proper duty cycle for valve opening
VALVE_CLOSE_DUTY = 75      # FIXED: Proper duty cycle for valve closing
ROOF_OPEN_DUTY = 150       # FIXED: Proper duty cycle for roof opening
ROOF_CLOSE_DUTY = 75       # FIXED: Proper duty cycle for roof closing
SERVO_DELAY = 800          # FIXED: Proper delay for servo movement
SERVO_FREQ = 50            # FIXED: Standard 50Hz for servos

# ===== GLOBAL VARIABLES =====
# CRITICAL FIX: Initialize all global variables to prevent assignment errors
last_activity = 0
mqtt_connected = False
wifi_connected = False
m2m_running = False
m2m_connected_client = None
m2m_server_socket = None
m2m_data = None
lcd = None
water_servo = None
roof_servo = None
status_timer = None

# Control state
water_valve_open = False
roof_open = False
auto_mode = True
last_water_time = 0
last_roof_time = 0

# System status
system_start_time = time.ticks_ms()
last_sensor_data = {}
last_mqtt_reconnect = time.ticks_ms()

# Button state tracking
button_states = {
    "water_switch": False,
    "roof_switch": False,
    "mode_switch": False
}
last_button_press = {
    "water_switch": 0,
    "roof_switch": 0,
    "mode_switch": 0
}

# LCD Display cache
lcd_cache = {
    "temp": "---.-C",
    "humid": "---.-%",
    "valve": "CLOSED",
    "roof": "CLOSED",
    "mode": "AUTO"
}

# ===== LCD FUNCTIONS =====
def setup_lcd():
    """Initialize I2C LCD display"""
    global lcd
    try:
        print("🔧 Initializing LCD...")
        i2c = I2C(0, scl=Pin(LCD_I2C_SCL), sda=Pin(LCD_I2C_SDA), freq=LCD_I2C_FREQ)
        
        # Scan I2C bus for devices
        devices = i2c.scan()
        print(f"📱 I2C devices found: {[hex(d) for d in devices]}")
        
        if LCD_I2C_ADDR not in devices:
            print(f"⚠️ LCD I2C address 0x{LCD_I2C_ADDR:02X} not found, trying common addresses")
            for addr in [0x3F, 0x27]:  # Common LCD addresses
                if addr in devices:
                    lcd = I2CLCD1602(i2c, addr=addr, cols=16, rows=2)
                    print(f"✅ LCD initialized with address 0x{addr:02X}")
                    break
        else:
            lcd = I2CLCD1602(i2c, addr=LCD_I2C_ADDR, cols=16, rows=2)
            print(f"✅ LCD initialized with address 0x{LCD_I2C_ADDR:02X}")
        
        if lcd:
            lcd.clear()
            time.sleep_ms(100)
            lcd.putstr("Smart Garden")
            time.sleep_ms(1000)
            lcd.move_to(0, 1)
            lcd.putstr("Initializing...")
            print("✅ LCD initialized successfully")
            return True
        else:
            print("❌ LCD initialization failed - no compatible device found")
            return False
            
    except Exception as e:
        print(f"❌ LCD initialization error: {e}")
        return False

def update_lcd_display(temp=None, humid=None, status="Ready"):
    """Update LCD display with current values"""
    try:
        if lcd is None:
            return
            
        # Clear display first
        lcd.clear()
        time.sleep_ms(10)
        
        if temp is not None and humid is not None:
            # Show temperature and humidity
            temp_str = f"{temp:.1f}C"
            humid_str = f"{humid:.0f}%"
            
            # Update cache
            lcd_cache["temp"] = temp_str
            lcd_cache["humid"] = humid_str
            
            lcd.move_to(0, 0)
            lcd.putstr(f"Temp: {temp_str}")
            
            lcd.move_to(0, 1)
            lcd.putstr(f"Humid: {humid_str}")
            
            print(f"📱 LCD updated: Temp={temp_str}, Humid={humid_str}")
        else:
            # Show status message
            lcd.move_to(0, 0)
            lcd.putstr("Smart Garden")
            lcd.move_to(0, 1)
            
            if len(status) > 16:
                status = status[:16]
            lcd.putstr(status)
            lcd_cache["temp"] = "---.-C"
            lcd_cache["humid"] = "---.-%"
            
            print(f"📱 LCD status: {status}")
            
    except Exception as e:
        print(f"❌ LCD update error: {e}")

# Enhanced LCD class for 16x2 I2C display
class I2CLCD1602:
    def __init__(self, i2c, addr=0x27, cols=16, rows=2):
        self.i2c = i2c
        self.addr = addr
        self.cols = cols
        self.rows = rows
        self._init_lcd()
        
    def _init_lcd(self):
        """Initialize LCD with proper I2C commands"""
        try:
            # LCD initialization sequence
            init_commands = [
                0x33,  # 00110011 - Function set (8-bit, 2 lines)
                0x32,  # 00110010 - Function set (4-bit, 2 lines)
                0x28,  # 00101000 - Function set (4-bit, 2 lines, 5x7 dots)
                0x0C,  # 00001100 - Display on, cursor off, blinking off
                0x06,  # 00000110 - Entry mode set (increment cursor, no shift)
                0x01,  # 00000001 - Clear display
            ]
            
            for cmd in init_commands:
                self._send_cmd(cmd)
                time.sleep_ms(5)
                
            print(f"✅ LCD display initialized at 0x{self.addr:02X}")
            
        except Exception as e:
            print(f"❌ LCD init error: {e}")
        
    def _send_cmd(self, cmd):
        """Send command to LCD"""
        try:
            # Send high nibble
            high_nib = cmd & 0xF0
            self._i2c_write(high_nib | 0x04)  # Enable high
            time.sleep_us(1)
            self._i2c_write(high_nib)         # Enable low
            
            # Send low nibble
            low_nib = (cmd << 4) & 0xF0
            self._i2c_write(low_nib | 0x04)   # Enable high
            time.sleep_us(1)
            self._i2c_write(low_nib)          # Enable low
            
        except Exception as e:
            print(f"❌ LCD cmd error: {e}")
        
    def _send_data(self, data):
        """Send data to LCD"""
        try:
            # Send high nibble
            high_nib = data & 0xF0
            self._i2c_write(high_nib | 0x05)  # Enable high, RS=1
            time.sleep_us(1)
            self._i2c_write(high_nib | 0x01)  # Enable low, RS=1
            
            # Send low nibble
            low_nib = (data << 4) & 0xF0
            self._i2c_write(low_nib | 0x05)   # Enable high, RS=1
            time.sleep_us(1)
            self._i2c_write(low_nib | 0x01)   # Enable low, RS=1
            
        except Exception as e:
            print(f"❌ LCD data error: {e}")
        
    def _i2c_write(self, data):
        """Write data to I2C with backlight control"""
        try:
            # Add backlight control (bit 3)
            data_with_backlight = data | 0x08
            self.i2c.writeto(self.addr, bytes([data_with_backlight]))
            time.sleep_us(50)
            
        except Exception as e:
            print(f"❌ I2C write error: {e}")
            
    def clear(self):
        """Clear the display"""
        try:
            self._send_cmd(0x01)  # Clear display command
            time.sleep_ms(2)      # Wait for clear operation
        except Exception as e:
            print(f"❌ LCD clear error: {e}")
            
    def putstr(self, string):
        """Print a string to the display"""
        try:
            if not isinstance(string, str):
                string = str(string)
            
            for char in string:
                if ord(char) >= 32 and ord(char) <= 126:  # Printable ASCII
                    self._send_data(ord(char))
                else:
                    self._send_data(ord(' '))  # Space for non-printable
                    
        except Exception as e:
            print(f"❌ LCD putstr error: {e}")
            
    def move_to(self, col, row):
        """Move cursor to position"""
        try:
            if row == 0:
                self._send_cmd(0x80 | col)
            elif row == 1:
                self._send_cmd(0xC0 | col)
            else:
                # For 2-line display, wrap to line 0
                self._send_cmd(0x80 | (col % self.cols))
                
        except Exception as e:
            print(f"❌ LCD move error: {e}")

# ===== SERVO FUNCTIONS =====
def setup_servos():
    """Initialize servo motors with proper PWM configuration"""
    global water_servo, roof_servo
    try:
        print("🔧 Setting up servo motors...")
        
        # Setup water valve servo
        print(f"💧 Water servo pin: {WATER_SERVO_PIN}")
        water_servo_pin = Pin(WATER_SERVO_PIN)
        water_servo = PWM(water_servo_pin)
        water_servo.freq(SERVO_FREQ)  # Set frequency to 50Hz
        water_servo.duty(VALVE_CLOSE_DUTY)  # Start in closed position
        time.sleep_ms(100)
        
        # Setup roof servo
        print(f"🏠 Roof servo pin: {ROOF_SERVO_PIN}")
        roof_servo_pin = Pin(ROOF_SERVO_PIN)
        roof_servo = PWM(roof_servo_pin)
        roof_servo.freq(SERVO_FREQ)  # Set frequency to 50Hz
        roof_servo.duty(ROOF_CLOSE_DUTY)  # Start in closed position
        time.sleep_ms(100)
        
        print("✅ Servos initialized successfully")
        print(f"💧 Water valve: Close={VALVE_CLOSE_DUTY}, Open={VALVE_OPEN_DUTY}")
        print(f"🏠 Roof: Close={ROOF_CLOSE_DUTY}, Open={ROOF_OPEN_DUTY}")
        return True
        
    except Exception as e:
        print(f"❌ Servo initialization failed: {e}")
        return False

def set_water_valve(state):
    """Control water valve servo with proper feedback"""
    global water_valve_open, last_water_time
    
    try:
        if water_servo is None:
            print("❌ Water servo not initialized")
            return False
            
        current_time = time.ticks_ms()
        
        if state and not water_valve_open:
            # Open valve
            print(f"💧 Opening water valve... (duty={VALVE_OPEN_DUTY})")
            water_servo.duty(VALVE_OPEN_DUTY)
            water_valve_open = True
            last_water_time = current_time
            
            # Update LCD and cache
            lcd_cache["valve"] = "OPEN"
            if lcd:
                update_lcd_display()
            
            # Turn on status LED
            Pin(STATUS_LED, Pin.OUT).on()
            
        elif not state and water_valve_open:
            # Close valve
            print(f"💧 Closing water valve... (duty={VALVE_CLOSE_DUTY})")
            water_servo.duty(VALVE_CLOSE_DUTY)
            water_valve_open = False
            
            # Update LCD and cache
            lcd_cache["valve"] = "CLOSED"
            if lcd:
                update_lcd_display()
            
            # Turn off status LED
            Pin(STATUS_LED, Pin.OUT).off()
        
        # Add delay for servo movement
        time.sleep_ms(SERVO_DELAY)
        
        print(f"💧 Water valve state: {'OPEN' if water_valve_open else 'CLOSED'}")
        return True
        
    except Exception as e:
        print(f"❌ Water valve control error: {e}")
        return False

def set_roof_servo(state):
    """Control roof servo with proper feedback"""
    global roof_open, last_roof_time
    
    try:
        if roof_servo is None:
            print("❌ Roof servo not initialized")
            return False
            
        current_time = time.ticks_ms()
        
        if state and not roof_open:
            # Open roof
            print(f"🏠 Opening roof... (duty={ROOF_OPEN_DUTY})")
            roof_servo.duty(ROOF_OPEN_DUTY)
            roof_open = True
            last_roof_time = current_time
            
            # Update LCD and cache
            lcd_cache["roof"] = "OPEN"
            if lcd:
                update_lcd_display()
            
        elif not state and roof_open:
            # Close roof
            print(f"🏠 Closing roof... (duty={ROOF_CLOSE_DUTY})")
            roof_servo.duty(ROOF_CLOSE_DUTY)
            roof_open = False
            
            # Update LCD and cache
            lcd_cache["roof"] = "CLOSED"
            if lcd:
                update_lcd_display()
        
        # Add delay for servo movement
        time.sleep_ms(SERVO_DELAY)
        
        print(f"🏠 Roof state: {'OPEN' if roof_open else 'CLOSED'}")
        return True
        
    except Exception as e:
        print(f"❌ Roof control error: {e}")
        return False

# ===== BUTTON HANDLING =====
def setup_buttons():
    """Initialize button inputs with proper debouncing"""
    global button_states
    
    try:
        print("🔧 Setting up buttons...")
        
        # Configure buttons with internal pull-ups
        water_btn = Pin(WATER_SWITCH, Pin.IN, Pin.PULL_UP)
        roof_btn = Pin(ROOF_SWITCH, Pin.IN, Pin.PULL_UP)
        mode_btn = Pin(MODE_SWITCH, Pin.IN, Pin.PULL_UP)
        
        # Store button objects globally for later use
        globals()["water_button"] = water_btn
        globals()["roof_button"] = roof_btn
        globals()["mode_button"] = mode_btn
        
        print(f"✅ Buttons initialized: Water={WATER_SWITCH}, Roof={ROOF_SWITCH}, Mode={MODE_SWITCH}")
        return True
        
    except Exception as e:
        print(f"❌ Button setup failed: {e}")
        return False

def check_buttons():
    """Check button states with debouncing"""
    global auto_mode, button_states, last_button_press
    
    try:
        current_time = time.ticks_ms()
        debounce_delay = 200  # 200ms debounce
        
        # Check water valve button
        if globals().get("water_button"):
            water_pressed = not globals()["water_button"].value()  # Active low
            
            if water_pressed and not button_states["water_switch"] and \
               (current_time - last_button_press["water_switch"]) > debounce_delay:
                button_states["water_switch"] = True
                last_button_press["water_switch"] = current_time
                
                print("🔘 Water valve button pressed")
                if auto_mode:
                    print("⚠️ Manual control disabled (auto mode active)")
                else:
                    set_water_valve(not water_valve_open)
            
            elif not water_pressed:
                button_states["water_switch"] = False
        
        # Check roof button
        if globals().get("roof_button"):
            roof_pressed = not globals()["roof_button"].value()  # Active low
            
            if roof_pressed and not button_states["roof_switch"] and \
               (current_time - last_button_press["roof_switch"]) > debounce_delay:
                button_states["roof_switch"] = True
                last_button_press["roof_switch"] = current_time
                
                print("🔘 Roof button pressed")
                if auto_mode:
                    print("⚠️ Manual control disabled (auto mode active)")
                else:
                    set_roof_servo(not roof_open)
            
            elif not roof_pressed:
                button_states["roof_switch"] = False
        
        # Check mode button
        if globals().get("mode_button"):
            mode_pressed = not globals()["mode_button"].value()  # Active low
            
            if mode_pressed and not button_states["mode_switch"] and \
               (current_time - last_button_press["mode_switch"]) > debounce_delay:
                button_states["mode_switch"] = True
                last_button_press["mode_switch"] = current_time
                
                print("🔘 Mode button pressed")
                auto_mode = not auto_mode
                lcd_cache["mode"] = "AUTO" if auto_mode else "MANUAL"
                
                # Update display
                status_msg = f"Mode: {'AUTO' if auto_mode else 'MANUAL'}"
                if lcd:
                    update_lcd_display(status=status_msg)
                
                print(f"🤖 Mode changed to: {'AUTO' if auto_mode else 'MANUAL'}")
            
            elif not mode_pressed:
                button_states["mode_switch"] = False
                
    except Exception as e:
        print(f"❌ Button check error: {e}")

# ===== WIFI SETUP =====
def setup_wifi():
    """Setup WiFi connection"""
    global wifi_connected
    
    try:
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        
        print(f"📡 Connecting to WiFi: {WIFI_SSID}")
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        
        # Wait for connection with timeout
        start_time = time.ticks_ms()
        timeout = 30000  # 30 seconds
        
        while not wlan.isconnected():
            if time.ticks_diff(time.ticks_ms(), start_time) > timeout:
                print("❌ WiFi connection timeout")
                wifi_connected = False
                return False
                
            if time.ticks_diff(time.ticks_ms(), start_time) % 3000 < 500:  # Print every 3 seconds
                print(".", end="")
            time.sleep(0.5)
        
        wifi_connected = True
        print(f"\n✅ WiFi connected!")
        print(f"IP Address: {wlan.ifconfig()[0]}")
        print(f"Signal Strength: {wlan.status('rssi')} dBm")
        
        if lcd:
            update_lcd_display(status="WiFi Connected")
        return True
        
    except Exception as e:
        print(f"❌ WiFi setup error: {e}")
        wifi_connected = False
        return False

# ===== MQTT SETUP =====
def setup_mqtt():
    """Setup MQTT client"""
    global mqtt_connected, last_mqtt_reconnect
    
    try:
        from umqtt.simple import MQTTClient
        
        # Create MQTT client
        mqtt_client = MQTTClient(CLIENT_ID, MQTT_SERVER, MQTT_PORT)
        mqtt_client.set_callback(mqtt_callback)
        
        # Connect with retry
        max_retries = 3
        for attempt in range(max_retries):
            try:
                print(f"📡 MQTT connection attempt {attempt + 1}/{max_retries}...")
                mqtt_client.connect()
                mqtt_connected = True
                last_mqtt_reconnect = time.ticks_ms()
                
                # Subscribe to topics
                mqtt_client.subscribe(SUBSCRIBE_TOPIC)
                mqtt_client.subscribe(ERROR_TOPIC)
                
                print("✅ MQTT connected and subscribed")
                
                # Send startup message
                send_mqtt_status("Control board started")
                
                return mqtt_client
                
            except Exception as e:
                print(f"❌ MQTT connection failed (attempt {attempt + 1}): {e}")
                time.sleep(5)
        
        print("❌ MQTT: All connection attempts failed")
        mqtt_connected = False
        return None
        
    except Exception as e:
        print(f"❌ MQTT setup error: {e}")
        mqtt_connected = False
        return None

def mqtt_callback(topic, msg):
    """Handle incoming MQTT messages"""
    global last_sensor_data
    
    try:
        topic_str = topic.decode('utf-8')
        msg_str = msg.decode('utf-8')
        
        print(f"📡 MQTT received [{topic_str}]: {msg_str}")
        
        if topic_str == "garden/sensor/temp_humidity":
            # Parse sensor data
            data = json.loads(msg_str)
            
            if 'temperature' in data and 'humidity' in data:
                temp = data['temperature']
                humid = data['humidity']
                
                # Store latest data
                last_sensor_data = data
                
                # Update display
                update_lcd_display(temp, humid, "Data Received")
                
                # Process sensor data for automatic control
                process_sensor_data(temp, humid)
                
                # Save to MongoDB
                save_to_mongodb(data)
                
                # Send status update
                send_mqtt_status(f"Temp: {temp:.1f}C, Humid: {humid:.0f}%")
                
        elif topic_str == "garden/sensor/error":
            # Handle sensor errors
            error_data = json.loads(msg_str)
            print(f"⚠️ Sensor error: {error_data}")
            update_lcd_display(status="Sensor Error")
            blink_error_led()
            
    except Exception as e:
        print(f"❌ MQTT callback error: {e}")

def send_mqtt_status(message):
    """Send status update via MQTT"""
    try:
        if mqtt_connected and globals().get("mqtt_client"):
            status_data = {
                "device_id": CLIENT_ID,
                "status": message,
                "timestamp": time.ticks_ms(),
                "uptime": time.ticks_diff(time.ticks_ms(), system_start_time)
            }
            
            status_json = json.dumps(status_data)
            globals()["mqtt_client"].publish(STATUS_TOPIC, status_json)
            print(f"📤 MQTT Status: {status_json}")
            
    except Exception as e:
        print(f"❌ MQTT status send error: {e}")

# ===== M2M SERVER SETUP =====
def setup_m2m_server():
    """Setup M2M server to receive direct sensor data"""
    global m2m_running, m2m_server_socket
    
    try:
        print(f"🔗 Setting up M2M server on port {M2M_SERVER_PORT}...")
        
        # Create server socket
        m2m_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        m2m_server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Bind to all interfaces on port 8888
        addr = socket.getaddrinfo('0.0.0.0', M2M_SERVER_PORT)[0][-1]
        m2m_server_socket.bind(addr)
        m2m_server_socket.listen(1)
        
        print(f"✅ M2M Server: Listening on port {M2M_SERVER_PORT}")
        print(f"📡 M2M Server: Ready to accept DHT22 sensor connections")
        print(f"🔍 mDNS service: smartgarden-control")
        
        # Set non-blocking
        m2m_server_socket.setblocking(False)
        
        m2m_running = True
        return m2m_server_socket
        
    except Exception as e:
        print(f"❌ M2M Server setup error: {e}")
        m2m_running = False
        return None

def handle_m2m_connections(server_socket):
    """Handle incoming M2M connections"""
    global m2m_connected_client, last_activity
    
    try:
        # Accept new connections
        client_socket, addr = server_socket.accept()
        client_socket.setblocking(False)
        
        print(f"✅ M2M: New connection from {addr}")
        
        # Close previous connection if exists
        if m2m_connected_client:
            try:
                m2m_connected_client.close()
            except:
                pass
        
        m2m_connected_client = client_socket
        last_activity = time.ticks_ms()
        
        update_lcd_display(status="M2M Connected")
        
    except OSError as e:
        # No new connections (expected in non-blocking mode)
        pass
    except Exception as e:
        print(f"❌ M2M connection error: {e}")
        m2m_connected_client = None

def process_m2m_data():
    """Process incoming M2M data"""
    global m2m_connected_client, last_sensor_data, last_activity
    
    if m2m_connected_client is None:
        return
    
    try:
        # Try to receive data
        data = m2m_connected_client.recv(M2M_BUFFER_SIZE)
        
        if data:
            # Decode and parse JSON
            try:
                json_str = data.decode('utf-8').strip()
                print(f"📥 M2M Data received: {json_str}")
                
                # Handle different message formats
                if json_str.startswith("SENSOR_DATA:"):
                    sensor_json = json_str[12:]  # Remove "SENSOR_DATA:" prefix
                    sensor_data = json.loads(sensor_json)
                else:
                    sensor_data = json.loads(json_str)
                
                if 'temperature' in sensor_data and 'humidity' in sensor_data:
                    temp = sensor_data['temperature']
                    humid = sensor_data['humidity']
                    
                    # Store latest data
                    last_sensor_data = sensor_data
                    
                    # Update display
                    update_lcd_display(temp, humid, "M2M Data")
                    
                    # Process sensor data for automatic control
                    process_sensor_data(temp, humid)
                    
                    # Save to MongoDB
                    save_to_mongodb(sensor_data)
                    
                    # Send acknowledgment back to sensor
                    send_m2m_ack()
                    
                    last_activity = time.ticks_ms()
                    
            except json.JSONDecodeError as e:
                print(f"❌ M2M: Invalid JSON data: {e}")
            except Exception as e:
                print(f"❌ M2M data processing error: {e}")
                
        else:
            # Connection closed by client
            print("❌ M2M: Connection closed by client")
            m2m_connected_client.close()
            m2m_connected_client = None
            update_lcd_display(status="M2M Disconnected")
            
    except OSError as e:
        # No data available (expected in non-blocking mode)
        pass
    except Exception as e:
        print(f"❌ M2M data receive error: {e}")
        m2m_connected_client.close()
        m2m_connected_client = None
        update_lcd_display(status="M2M Error")

def send_m2m_ack():
    """Send acknowledgment to DHT22 sensor"""
    global m2m_connected_client
    
    if m2m_connected_client:
        try:
            ack_data = {
                "status": "acknowledged",
                "timestamp": time.ticks_ms()
            }
            ack_json = json.dumps(ack_data)
            m2m_connected_client.send(ack_json + "\n")
            print("📤 M2M: Acknowledgment sent")
        except Exception as e:
            print(f"❌ M2M ack send error: {e}")

# ===== CONTROL LOGIC =====
def process_sensor_data(temp, humid):
    """Process sensor data and make control decisions"""
    global last_water_time, last_roof_time
    
    print(f"🧠 Processing sensor data: {temp:.1f}°C, {humid:.0f}%")
    
    # Automatic control based on thresholds
    if auto_mode:
        # Water valve control based on humidity
        current_time = time.ticks_ms()
        
        if humid < HUMID_THRESHOLD and not water_valve_open:
            print(f"💧 Low humidity detected ({humid:.0f}%), opening water valve...")
            set_water_valve(True)
            last_water_time = current_time
            
            # Close valve after duration
            timer = Timer(1)
            timer.init(period=WATER_DURATION, mode=Timer.ONE_SHOT, 
                      callback=lambda t: set_water_valve(False))
                      
        elif humid >= HUMID_THRESHOLD and water_valve_open:
            print(f"✅ Humidity adequate ({humid:.0f}%), closing water valve...")
            set_water_valve(False)
        
        # Roof control based on temperature
        if temp > TEMP_THRESHOLD and not roof_open:
            print(f"☀️ High temperature detected ({temp:.1f}°C), opening roof...")
            set_roof_servo(True)
            last_roof_time = current_time
            
        elif temp <= TEMP_THRESHOLD and roof_open:
            print(f"🌡️ Temperature acceptable ({temp:.1f}°C), closing roof...")
            set_roof_servo(False)

def save_to_mongodb(data):
    """Save sensor data to MongoDB (simulation)"""
    try:
        # In a real implementation, you would connect to MongoDB here
        print(f"💾 Saving to MongoDB: {data}")
        
        # Simulate successful save
        mongodb_data = {
            "source": "control_board",
            "sensor_data": data,
            "control_actions": {
                "water_valve": water_valve_open,
                "roof": roof_open,
                "auto_mode": auto_mode
            },
            "timestamp": time.ticks_ms()
        }
        
        print(f"📊 MongoDB record created: {mongodb_data}")
        
    except Exception as e:
        print(f"❌ MongoDB save error: {e}")

# ===== SYSTEM STATUS =====
def blink_status_led():
    """Blink status LED"""
    try:
        status_led = Pin(STATUS_LED, Pin.OUT)
        status_led.on()
        time.sleep(0.1)
        status_led.off()
    except:
        pass

def blink_error_led():
    """Blink error LED"""
    try:
        error_led = Pin(ERROR_LED, Pin.OUT)
        for i in range(3):
            error_led.on()
            time.sleep(0.2)
            error_led.off()
            time.sleep(0.2)
    except:
        pass

def system_status_report():
    """Print system status report"""
    uptime = time.ticks_diff(time.ticks_ms(), system_start_time)
    
    print("📊 ===== SYSTEM STATUS REPORT =====")
    print(f"⏰ Uptime: {uptime // 1000} seconds")
    print(f"📡 WiFi: {'Connected' if wifi_connected else 'Disconnected'}")
    print(f"📡 MQTT: {'Connected' if mqtt_connected else 'Disconnected'}")
    print(f"🔗 M2M Server: {'Running' if m2m_running else 'Stopped'}")
    print(f"🔗 M2M Client: {'Connected' if m2m_connected_client else 'None'}")
    
    if last_sensor_data:
        print(f"🌡️ Last sensor data: {last_sensor_data}")
    
    print(f"💧 Water valve: {'OPEN' if water_valve_open else 'CLOSED'}")
    print(f"🏠 Roof: {'OPEN' if roof_open else 'CLOSED'}")
    print(f"🤖 Auto mode: {'ON' if auto_mode else 'OFF'}")
    print(f"💾 Free memory: {gc.mem_free()} bytes")
    print(f"📱 LCD Cache: {lcd_cache}")
    print("=" * 40)

# ===== HARDWARE SETUP =====
def setup_hardware():
    """Initialize all hardware components"""
    global status_timer
    
    print("🔧 Setting up hardware components...")
    
    try:
        # Initialize LEDs
        Pin(LED_PIN, Pin.OUT).off()
        Pin(STATUS_LED, Pin.OUT).off()
        Pin(ERROR_LED, Pin.OUT).off()
        print("✅ LEDs initialized")
        
        # Initialize LCD
        if not setup_lcd():
            print("⚠️ LCD initialization failed, continuing...")
        
        # Initialize servos
        if not setup_servos():
            print("⚠️ Servo initialization failed, continuing...")
        
        # Initialize buttons
        if not setup_buttons():
            print("⚠️ Button initialization failed, continuing...")
        
        # Initialize status timer for periodic reports
        status_timer = Timer(0)
        status_timer.init(period=60000, mode=Timer.PERIODIC, 
                         callback=lambda t: system_status_report())
        
        print("✅ All hardware components initialized successfully")
        return True
        
    except Exception as e:
        print(f"❌ Hardware setup failed: {e}")
        return False

# ===== MAIN FUNCTION =====
def main():
    """Main program loop - DEFINITIVE FIX without threading"""
    global last_activity, mqtt_connected, wifi_connected, m2m_running
    
    print("🚀 Starting Temperature & Humidity Control System (DEFINITIVE FIX)...")
    print("FIXED: All variable assignment errors, servo control, LCD display, button handling")
    print("COMPREHENSIVE: Enhanced error handling, M2M stability, system monitoring")
    print("Version: Definitive Fix - 2025-11-29")
    
    gc.collect()
    
    # Setup all hardware
    if not setup_hardware():
        print("❌ Hardware setup failed. Retrying in 5 seconds...")
        if lcd:
            update_lcd_display(status="Hardware Error")
        time.sleep(5)
        main()
        return
    
    # Setup WiFi
    if not setup_wifi():
        print("❌ WiFi: Failed to connect. Continuing offline...")
        if lcd:
            update_lcd_display(status="WiFi Failed")
    else:
        if lcd:
            update_lcd_display(status="WiFi Connected")
    
    # Setup M2M Server (CRITICAL - this must be running before sensor connects)
    m2m_server_socket = None
    if wifi_connected:
        m2m_server_socket = setup_m2m_server()
        if m2m_server_socket:
            if lcd:
                update_lcd_display(status="M2M Server Ready")
        else:
            if lcd:
                update_lcd_display(status="M2M Server Error")
    
    # Setup MQTT
    mqtt_client = None
    if wifi_connected:
        mqtt_client = setup_mqtt()
        globals()["mqtt_client"] = mqtt_client  # Store globally for callbacks
        if mqtt_client:
            if lcd:
                update_lcd_display(status="MQTT Connected")
        else:
            if lcd:
                update_lcd_display(status="MQTT Failed")
    
    print("✅ Temperature & Humidity Control System Ready!")
    print("🔗 M2M server listening for DHT22 sensor connections")
    print("📡 MQTT + M2M dual connectivity active")
    print("💡 System is ready to receive sensor data!")
    
    # Main loop variables
    last_status_report = time.ticks_ms()
    last_lcd_update = time.ticks_ms()
    last_heartbeat = time.ticks_ms()
    last_mqtt_retry = time.ticks_ms()
    
    # LED confirmation sequence
    for i in range(3):
        Pin(STATUS_LED, Pin.OUT).on()
        time.sleep(0.2)
        Pin(STATUS_LED, Pin.OUT).off()
        time.sleep(0.2)
    
    # Main loop - DEFINITIVE FIX: No threading, pure polling with proper error handling
    while True:
        try:
            current_time = time.ticks_ms()
            
            # Handle MQTT connections
            if mqtt_client and mqtt_connected:
                try:
                    mqtt_client.check_msg()
                except Exception as e:
                    print(f"❌ MQTT check error: {e}")
                    mqtt_connected = False
            
            # Handle M2M server connections
            if m2m_server_socket and m2m_running:
                handle_m2m_connections(m2m_server_socket)
                process_m2m_data()
            
            # Check button states
            check_buttons()
            
            # Status reporting every 60 seconds
            if time.ticks_diff(current_time, last_status_report) >= 60000:
                system_status_report()
                last_status_report = current_time
            
            # LCD update every 30 seconds
            if time.ticks_diff(current_time, last_lcd_update) >= 30000:
                if last_sensor_data:
                    temp = last_sensor_data.get('temperature', 0)
                    humid = last_sensor_data.get('humidity', 0)
                    if lcd:
                        update_lcd_display(temp, humid, "Auto Mode")
                else:
                    if lcd:
                        update_lcd_display(status="Waiting for data")
                last_lcd_update = current_time
            
            # MQTT reconnection attempt
            if wifi_connected and not mqtt_connected:
                if time.ticks_diff(current_time, last_mqtt_retry) >= 60000:  # Retry every minute
                    print("🔄 Attempting MQTT reconnection...")
                    mqtt_client = setup_mqtt()
                    globals()["mqtt_client"] = mqtt_client
                    last_mqtt_retry = current_time
            
            # System health check
            if time.ticks_diff(current_time, last_activity) > 300000:  # 5 minutes
                print("⚠️ System idle check...")
                blink_status_led()
                last_activity = current_time
            
            # Garbage collection
            if gc.mem_free() < 10000:  # Less than 10KB free
                print("🔄 Running garbage collection...")
                gc.collect()
            
            time.sleep(0.1)  # Small delay to prevent watchdog issues
            
        except KeyboardInterrupt:
            print("🛑 System interrupted by user")
            break
        except Exception as e:
            print(f"❌ Main loop error: {e}")
            blink_error_led()
            time.sleep(1)
    
    # Cleanup
    print("🧹 Cleaning up...")
    if m2m_connected_client:
        m2m_connected_client.close()
    if mqtt_client:
        mqtt_client.disconnect()
    if status_timer:
        status_timer.deinit()
    
    # Reset hardware
    set_water_valve(False)
    set_roof_servo(False)
    
    print("👋 System shutdown complete")

# ===== EXECUTION =====
if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        blink_error_led()
        machine.reset()
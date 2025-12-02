#!/usr/bin/env python3
"""
Smart Garden Control Board - TEMP/HUM AUTO CONTROL INTEGRATED
ESP32 with LCD, Servos, Buttons - MicroPython

FIXED:
1. Log Spam: Restored optimization checks (if percentage == current_percentage: return) 
   in servo setters to stop continuous logging of stable states.
2. Logic: Both Servo 1 (Water) and Servo 2 (Roof) are controlled by Temperature logic.

Author: MiniMax Agent
Version: LOG OPTIMIZED - 2025-12-02
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
MQTT_CLIENT_ID = "ESP_CONTROL_TEMP_HUM_LOGIC_01"

# MQTT Topics
MQTT_SENSOR_DATA_TOPIC = "garden/sensor/temp_hum_data"
MQTT_CONTROL_COMMANDS_TOPIC = "garden/control/commands"
MQTT_STATUS_TOPIC = "garden/status/control_board"

# Servo Configuration (Sử dụng 10-bit duty cycle: 0-1023)
WATER_SERVO_PIN = 15 
ROOF_SERVO_PIN = 14 

# Dải Duty Cycle ĐÃ KIỂM CHỨNG (40-115)
# Tương đương ~1.0ms pulse (Min an toàn)
WATER_CLOSE_DUTY = 40 
ROOF_CLOSE_DUTY = 40
# Tương đương ~2.0ms pulse (Max an toàn)
WATER_OPEN_DUTY = 115 
ROOF_OPEN_DUTY = 115

SERVO_FREQ = 50 

# LCD Configuration (I2C)
LCD_I2C_ADDRESS = 0x27
LCD_I2C_SDA = 21
LCD_I2C_SCL = 22

# Pins (for completeness, although not all used in logic)
STATUS_LED_PIN = 4 
BUILTIN_LED_PIN = 2 
WATER_BUTTON_PIN = 25 
ROOF_BUTTON_PIN = 26 
MODE_BUTTON_PIN = 27

# ===== AUTO CONTROL THRESHOLDS (Ngưỡng Điều khiển) =====
THRESHOLD = {
    "temperature": {
        "max": 30.0, 
        "min": 18.0 
    },
    "humidity": {
        "max": 75.0, 
        "min": 50.0 
    }
}

# ===== GLOBAL VARIABLES =====
mqtt_client = None
mqtt_connected = False
wifi_connected = False
lcd = None
servo_water = None
servo_roof = None

water_percentage = 0
roof_percentage = 0
auto_mode = True # Mặc định AUTO MODE
current_temperature = 0.0
current_humidity = 0.0
last_temp_update = time.ticks_ms()

mqtt_messages_received = 0
system_start_time = time.ticks_ms()
led_state = False
led_blink_last = time.ticks_ms()
last_status_print = time.ticks_ms()

# Global hardware objects for buttons/LEDs (Không sử dụng trong logic chính)
water_button = Pin(WATER_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
roof_button = Pin(ROOF_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
mode_button = Pin(MODE_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
status_led = None
builtin_led = None

# ===== LCD FUNCTIONS (Giữ nguyên) =====

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
            lcd.putstr("Control Init...")
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
    global lcd, mqtt_connected, water_percentage, roof_percentage, current_temperature, current_humidity
    
    if not lcd:
        return
        
    try:
        lcd.clear()
        
        # Line 1: Temperature and Humidity
        temp_str = f"{current_temperature:.1f}C"
        hum_str = f"{current_humidity:.0f}%"
        mode_str = "A" if auto_mode else "M"
        
        line1 = f"[{mode_str}]T{temp_str} H{hum_str}"
        lcd.move_to(0, 0)
        lcd.putstr(line1[:16])
        
        # Line 2: Control status
        line2 = f"W:{water_percentage}% R:{roof_percentage}%"
        line2 += " ONLINE" if mqtt_connected else " OFFLINE"
        
        lcd.move_to(0, 1)
        lcd.putstr(line2[:16])
        
    except Exception as e:
        print(f"❌ LCD update error: {e}")

# ===== SERVO FUNCTIONS (Log Tối ưu) =====

def setup_servos():
    """Setup servo motors - Sử dụng dải 40-115"""
    global servo_water, servo_roof
    try:
        print("🔧 Setting up servos...")
        
        # --- SERVO WATER (Pin 15) ---
        water_pin = Pin(WATER_SERVO_PIN)
        servo_water = PWM(water_pin, freq=SERVO_FREQ, duty=WATER_CLOSE_DUTY) 
        
        # --- SERVO ROOF (Pin 14) ---
        roof_pin = Pin(ROOF_SERVO_PIN)
        servo_roof = PWM(roof_pin, freq=SERVO_FREQ, duty=ROOF_CLOSE_DUTY)
        
        print(f"✅ Servos initialized (Closed) using 10-bit [{WATER_CLOSE_DUTY}-{WATER_OPEN_DUTY}].")
        return True
    except Exception as e:
        print(f"❌ Servo setup error: {e}")
        return False

def set_water_percentage(percentage):
    """Set water valve percentage (0-100) - FIX: Khôi phục tối ưu hóa và log ngắn gọn"""
    global water_percentage, servo_water
    
    if not servo_water: return
        
    percentage = max(0, min(100, percentage))
    
    # 🎯 TỐI ƯU HÓA: Nếu giá trị không thay đổi, không gửi lệnh và không in log.
    if percentage == water_percentage:
        return 
    
    water_percentage = percentage
    
    # Tính Duty: Ánh xạ 0-100% sang dải an toàn (CLOSE_DUTY - OPEN_DUTY)
    duty = WATER_CLOSE_DUTY + (percentage * (WATER_OPEN_DUTY - WATER_CLOSE_DUTY) / 100)
    
    # .duty() sử dụng độ phân giải 10-bit mặc định
    servo_water.duty(int(duty)) 
    
    # In log ngắn gọn, chỉ khi hành động xảy ra
    action = "BẬT" if percentage == 100 else "TẮT"
    print(f"💧 Water Valve: {action} (Duty: {int(duty)})")
    update_lcd_display()

def set_roof_percentage(percentage):
    """Set roof percentage (0-100) - FIX: Khôi phục tối ưu hóa và log ngắn gọn"""
    global roof_percentage, servo_roof
    
    if not servo_roof: return
        
    percentage = max(0, min(100, percentage))
    
    # 🎯 TỐI ƯU HÓA: Nếu giá trị không thay đổi, không gửi lệnh và không in log.
    if percentage == roof_percentage:
        return
    
    # --- SIMULATE SLOWER MOVEMENT ---
    if abs(percentage - roof_percentage) > 10:
        time.sleep_ms(100) 
    
    roof_percentage = percentage
    
    # Tính Duty: Ánh xạ 0-100% sang dải an toàn (CLOSE_DUTY - OPEN_DUTY)
    duty = ROOF_CLOSE_DUTY + (percentage * (ROOF_OPEN_DUTY - ROOF_CLOSE_DUTY) / 100)
    
    # .duty() sử dụng độ phân giải 10-bit mặc định
    servo_roof.duty(int(duty))
    
    # In log ngắn gọn, chỉ khi hành động xảy ra
    action = "MỞ" if percentage == 100 else "ĐÓNG"
    print(f"🏠 Roof Valve: {action} (Duty: {int(duty)})")
    update_lcd_display()

# ===== MQTT CALLBACK (Giữ nguyên) =====

def mqtt_callback(topic, msg):
    """MQTT callback function for incoming messages"""
    global mqtt_messages_received, current_temperature, current_humidity, last_temp_update
    global water_percentage, roof_percentage, auto_mode
    
    try:
        topic_str = topic.decode('utf-8')
        msg_str = msg.decode('utf-8')
        mqtt_messages_received += 1
        
        # Log tin nhắn MQTT đến, giữ nguyên
        print(f"📨 MQTT: [{topic_str}]: {msg_str}")
        
        if topic_str == MQTT_SENSOR_DATA_TOPIC:
            # Parse sensor data
            try:
                data = json.loads(msg_str)
                temp = data.get('temperature')
                hum = data.get('humidity')
                
                if temp is not None and hum is not None:
                    current_temperature = float(temp)
                    current_humidity = float(hum)
                    last_temp_update = time.ticks_ms()
                    # Log dữ liệu cảm biến, giữ nguyên
                    print(f"📊 Sensor data: T={current_temperature:.1f}C, H={current_humidity:.1f}%")
                    
                    # Chỉ kích hoạt logic tự động nếu auto_mode = True
                    if auto_mode:
                        auto_control_logic() 
                    update_lcd_display()
                else:
                    print("⚠️ Sensor data missing T/H keys.")

            except Exception as e:
                print(f"❌ Sensor data parse error: {e}")
                
        # Logic nhận lệnh điều khiển thủ công từ Server/App (tùy chọn)
        elif topic_str == MQTT_CONTROL_COMMANDS_TOPIC:
            try:
                command = json.loads(msg_str)
                cmd_type = command.get('command', '')
                
                if cmd_type == 'water_valve':
                    # Khi nhận lệnh MQTT, tự động chuyển sang chế độ thủ công
                    auto_mode = False 
                    percentage = command.get('value', 0)
                    set_water_percentage(percentage)
                elif cmd_type == 'roof':
                    # Khi nhận lệnh MQTT, tự động chuyển sang chế độ thủ công
                    auto_mode = False 
                    percentage = command.get('value', 0)
                    set_roof_percentage(percentage)
            except Exception as e:
                print(f"❌ Command parse error: {e}")
                
    except Exception as e:
        print(f"❌ MQTT message handler error: {e}")

# ===== AUTO CONTROL LOGIC (ĐÃ ĐỒNG BỘ THEO NHIỆT ĐỘ) =====

def auto_control_logic():
    """
    Thực hiện logic điều khiển Servo (mái che & van nước).
    CẢ HAI SERVO ĐỀU PHẢN ỨNG VỚI NHIỆT ĐỘ.
    """
    global current_temperature, current_humidity, water_percentage, roof_percentage

    if not auto_mode:
        return
    
    # Quyết định hành động dựa trên Nhiệt độ
    if current_temperature > THRESHOLD["temperature"]["max"]:
        print(f"🔥: TEMPERATURE HIGH ({current_temperature:.1f} > {THRESHOLD['temperature']['max']}). Open Water Valve")
        
        # Nhiệt độ nóng bật van nước
        if water_percentage != 100:
            set_water_percentage(100)

        if current_humidity > THRESHOLD["humidity"]["max"]:
            print(f"💦: HUMIDITY HIGHT ({current_humidity:.1f} > {THRESHOLD['humidity']['max']}).Disable Roof ")
            # Nhiệt độ cao, dộ ẩm cao thì tắt kích hoạt mái che
            if roof_percentage != 0:
                set_roof_percentage(0)
        elif current_humidity < THRESHOLD["humidity"]["min"]:
            print(f"💧: HUMIDITY LOW ({current_humidity:.1f} < {THRESHOLD['humidity']['min']}).Enable Roof ")
            # Nhiệt độ cao, Độ ẩm thấp kích hoạt mái che
            if roof_percentage != 100:
                set_roof_percentage(100)
                    
    elif current_temperature < THRESHOLD["temperature"]["min"]:
        # Log hành động chung, chỉ in khi nhiệt độ nằm ngoài ngưỡng
        print(f"❄️ AUTO: TEMP LOW ({current_temperature:.1f} < {THRESHOLD['temperature']['min']}). Close Water Valve")
        
        # Nhiệt độ lạnh tắt van nước
        if water_percentage != 0:
            set_water_percentage(0)

        if current_humidity > THRESHOLD["humidity"]["max"]:
            print(f"💦: HUMIDITY HIGHT ({current_humidity:.1f} > {THRESHOLD['humidity']['max']}).Disable Roof ")
            # Nhiệt độ lạnh, dộ ẩm cao thì tắt kích hoạt mái che
            if roof_percentage != 0:
                set_roof_percentage(0)
        elif current_humidity < THRESHOLD["humidity"]["min"]:
            print(f"💧: HUMIDITY LOW ({current_humidity:.1f} < {THRESHOLD['humidity']['min']}).Enable Roof ")
            # Nhiệt độ lạnh, Độ ẩm thấp kích hoạt mái che
            if roof_percentage != 100:
                set_roof_percentage(100)
        
        
            
       
        
        
            
    # Không cần else (nếu nằm trong ngưỡng, Servo giữ nguyên trạng thái cuối cùng)

# ===== CÁC HÀM KHÁC (ĐÃ GỠ BỎ LOGIC BUTTON) =====

def setup_wifi():
    """Setup WiFi connection"""
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
    """Setup MQTT client"""
    global mqtt_client, mqtt_connected
    try:
        print("📡 Setting up MQTT client...")
        mqtt_client = MQTTClient(MQTT_CLIENT_ID, MQTT_SERVER, MQTT_PORT, keepalive=60)
        mqtt_client.set_callback(mqtt_callback)
        print("🔌 Attempting MQTT connection...")
        mqtt_client.connect(clean_session=True)
        
        print("✅ MQTT connected successfully!")
        mqtt_connected = True
        
        mqtt_client.subscribe(MQTT_SENSOR_DATA_TOPIC.encode())
        mqtt_client.subscribe(MQTT_CONTROL_COMMANDS_TOPIC.encode())
        
        print(f"✅ Subscribed to: {MQTT_SENSOR_DATA_TOPIC}")
        return True
    except Exception as e:
        print(f"❌ MQTT setup failed: {e}")
        mqtt_connected = False
        return False

def check_mqtt_connection():
    """Check and maintain MQTT connection"""
    global mqtt_client, mqtt_connected
    
    if not mqtt_client: return False
    
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
    """Setup LEDs (Based on diagram - Pin 4)"""
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
    """Update LED status"""
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

# HÀM CHECK_BUTTONS ĐÃ BỊ LOẠI BỎ THEO YÊU CẦU

def print_system_status():
    """Print system status"""
    global current_temperature, current_humidity, water_percentage, roof_percentage, auto_mode, mqtt_connected, system_start_time
    try:
        uptime = (time.ticks_ms() - system_start_time) // 1000
        
        print("\n🔄 SYSTEM STATUS:")
        print(f"  MQTT: {'✅ CONNECTED' if mqtt_connected else '❌ DISCONNECTED'}")
        print(f"  Temperature: {current_temperature:.1f}C")
        print(f"  Humidity: {current_humidity:.1f}%")
        print(f"  Water Valve: {water_percentage}%")
        print(f"  Roof: {roof_percentage}%")
        print(f"  Uptime: {uptime}s")
        print(f"  Auto Mode: {'ON' if auto_mode else 'OFF'}")
        print("=" * 60)
        
    except Exception as e:
        print(f"❌ Status print error: {e}")
        
def main():
    """Main system function"""
    global system_start_time, last_status_print
    
    try:
        print("🌱 Smart Garden Control Board - LOGIC INTEGRATED")
        
        system_start_time = time.ticks_ms()
        last_status_print = time.ticks_ms()
        
        # Initialize hardware
        print("🔧 Initializing hardware...")
        if not setup_lcd(): print("⚠️ LCD failed, continuing without LCD")
        if not setup_servos(): return
        if not setup_leds(): return
        
        # Setup network connections
        print("\n🌐 Setting up network connections...")
        if not setup_wifi(): return
        if not setup_mqtt(): print("⚠️ MQTT setup failed, will retry in background")
        
        print("\n🚀 System ready!")
        
        # Main loop
        while True:
            try:
                current_time = time.ticks_ms()
                
                # Duy trì kết nối MQTT
                check_mqtt_connection()
                
                # Kiểm tra tin nhắn MQTT đến
                if mqtt_client and mqtt_connected:
                    mqtt_client.check_msg() 
                    
                # Thực hiện logic tự động nếu đang ở chế độ Auto và dữ liệu T/H mới
                if auto_mode and (current_time - last_temp_update) < 60000:
                    auto_control_logic()
                
                # check_buttons() 
                
                update_leds()
                
                # Cập nhật LCD (cứ sau 2 giây)
                if current_time % 2000 < 100: 
                    update_lcd_display()
                
                # In trạng thái hệ thống (cứ sau 30 giây)
                if current_time - last_status_print > 30000:
                    print_system_status() 
                    last_status_print = current_time
                
                # Thu gom rác để giải phóng bộ nhớ (cứ sau 1 phút)
                if current_time % 60000 < 1000: gc.collect()
                
                time.sleep_ms(100) # Chu kỳ loop 100ms
                
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
    """Clean up system resources"""
    try:
        print("🧹 Cleaning up...")
        if mqtt_client:
            try: mqtt_client.disconnect()
            except: pass
        if servo_water: servo_water.deinit()
        if servo_roof: servo_roof.deinit()
        print("✅ Cleaanup complete")
    except Exception as e:
        print(f"❌ Cleanup error: {e}")

# ===== BOOTSTRAP =====
if __name__ == "__main__":
    main()
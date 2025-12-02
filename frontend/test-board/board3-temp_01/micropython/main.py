# file: main.py - Smart Garden Board 3: Temperature/Humidity + M2M Control (MicroPython)
# Chức năng: Điều khiển LCD, Servo, Buttons, giao tiếp M2M với thiết bị khác
# Áp dụng: MicroPython, LCD I2C, Servo Control, M2M Communication

import network
import time
import json
import gc
from machine import Pin, PWM, I2C, unique_id, reset
from umqtt.simple import MQTTClient
from servo import Servo
import dht

# WiFi Configuration
WIFI_SSID = "Wokwi-GUEST"
WIFI_PASSWORD = ""

# MQTT Configuration
MQTT_SERVER = "broker.hivemq.com"
MQTT_PORT = 1883
CLIENT_ID = "ESP_MPY_BOARD3_CONTROL_01"

SUBSCRIBE_TOPICS = [
    b"garden/control/mpy_lcd",
    b"garden/control/mpy_servo",
    b"garden/control/mpy_buttons",
    b"garden/control/mpy_config"
]
STATUS_TOPIC = b"garden/status/board3_mpy"

# Hardware Configuration
LCD_SDA_PIN = 21
LCD_SCL_PIN = 22
VALVE_SERVO_PIN = 12
ROOF_SERVO_PIN = 13
BUTTON1_PIN = 25
BUTTON2_PIN = 26
BUTTON3_PIN = 27
BUTTON4_PIN = 14
LED_PIN = 2

# I2C Configuration for M2M Communication
I2C_SDA_M2M_PIN = 19
I2C_SCL_M2M_PIN = 18

# Global variables
wifi = None
mqtt_client = None
wifi_connected = False
mqtt_connected = False
reconnect_attempts = 0

# Hardware objects
servo_valve = None
servo_roof = None
lcd = None
buttons = {}
led = None

# Control variables
current_valve_angle = 0
current_roof_angle = 0
auto_mode = True
lcd_update_counter = 0
last_sensor_data = {}
m2m_data_buffer = []

def setup_hardware():
    """Khởi tạo phần cứng"""
    global servo_valve, servo_roof, lcd, buttons, led
    
    try:
        print("=== Smart Garden - Board 3 MicroPython Control ===")
        print("Client ID: " + str(CLIENT_ID))
        
        # Initialize LED
        led = Pin(LED_PIN, Pin.OUT)
        led.off()
        
        # Initialize Servos
        servo_valve = Servo(Pin(VALVE_SERVO_PIN))
        servo_roof = Servo(Pin(ROOF_SERVO_PIN))
        
        # Set initial positions
        servo_valve.write_angle(0)
        servo_roof.write_angle(0)
        current_valve_angle = 0
        current_roof_angle = 0
        
        # Initialize I2C for LCD
        i2c = I2C(0, scl=Pin(LCD_SCL_PIN), sda=Pin(LCD_SDA_PIN))
        lcd = I2CLCD(i2c, 20, 4)  # 20x4 LCD
        
        # Initialize Buttons
        buttons['button1'] = Pin(BUTTON1_PIN, Pin.IN, Pin.PULL_UP)
        buttons['button2'] = Pin(BUTTON2_PIN, Pin.IN, Pin.PULL_UP)
        buttons['button3'] = Pin(BUTTON3_PIN, Pin.IN, Pin.PULL_UP)
        buttons['button4'] = Pin(BUTTON4_PIN, Pin.IN, Pin.PULL_UP)
        
        # Initialize M2M I2C
        i2c_m2m = I2C(1, scl=Pin(I2C_SCL_M2M_PIN), sda=Pin(I2C_SDA_M2M_PIN))
        
        print("✅ Hardware initialized successfully")
        return True
        
    except Exception as e:
        print("❌ Hardware setup error:", str(e))
        return False

def setup_wifi():
    """Kết nối WiFi"""
    global wifi, wifi_connected
    
    try:
        print("WiFi: Initializing...")
        wifi = network.WLAN(network.STA_IF)
        wifi.active(True)
        
        print(f"WiFi: Connecting to {WIFI_SSID}")
        wifi.connect(WIFI_SSID, WIFI_PASSWORD)
        
        # Đợi kết nối
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
        print("❌ WiFi: Connection failed after", max_attempts, "attempts")
        return False
        
    except Exception as e:
        wifi_connected = False
        print("❌ WiFi: Setup error:", str(e))
        return False

def mqtt_callback(topic, msg):
    """MQTT callback function"""
    global last_sensor_data, m2m_data_buffer
    
    try:
        if not topic or not msg:
            print("❌ MQTT: Invalid callback data")
            return
            
        topic_str = topic.decode('utf-8', 'ignore')
        message = msg.decode('utf-8', 'ignore')
        
        print(f"[MQTT] Receive: {topic_str} = '{message}'")
        
        if topic_str == "garden/control/mpy_lcd":
            handle_lcd_command(message)
        elif topic_str == "garden/control/mpy_servo":
            handle_servo_command(message)
        elif topic_str == "garden/control/mpy_buttons":
            handle_button_command(message)
        elif topic_str == "garden/control/mpy_config":
            handle_config_command(message)
        elif "garden/sensor/" in topic_str:
            # Nhận dữ liệu cảm biến từ C++ code
            handle_sensor_data(topic_str, message)
            
    except Exception as e:
        print("❌ MQTT callback error:", str(e))

def handle_lcd_command(message):
    """Xử lý lệnh điều khiển LCD"""
    global lcd_update_counter
    
    try:
        if lcd is None:
            return
            
        if message.startswith("TEXT:"):
            # Format: TEXT:row,col,message
            parts = message.split(":", 1)[1].split(",", 2)
            if len(parts) == 3:
                row = int(parts[0])
                col = int(parts[1])
                text = parts[2]
                lcd.text(text, row, col)
                print(f"[LCD] Set text: row {row}, col {col} = {text}")
                
        elif message == "CLEAR":
            lcd.clear()
            lcd_update_counter = 0
            print("[LCD] Cleared display")
            
        elif message == "BACKLIGHT_ON":
            lcd.backlight(True)
            print("[LCD] Backlight ON")
            
        elif message == "BACKLIGHT_OFF":
            lcd.backlight(False)
            print("[LCD] Backlight OFF")
            
    except Exception as e:
        print("❌ LCD command error:", str(e))

def handle_servo_command(message):
    """Xử lý lệnh điều khiển servo"""
    global current_valve_angle, current_roof_angle
    
    try:
        if message.startswith("VALVE:"):
            # Format: VALVE:angle
            angle = int(message.split(":")[1])
            angle = max(0, min(180, angle))
            
            if servo_valve:
                servo_valve.write_angle(angle)
                current_valve_angle = angle
                print(f"[SERVO] Valve angle: {angle}°")
                
        elif message.startswith("ROOF:"):
            # Format: ROOF:angle
            angle = int(message.split(":")[1])
            angle = max(0, min(180, angle))
            
            if servo_roof:
                servo_roof.write_angle(angle)
                current_roof_angle = angle
                print(f"[SERVO] Roof angle: {angle}°")
                
        elif message == "VALVE_OPEN":
            if servo_valve:
                servo_valve.write_angle(90)
                current_valve_angle = 90
                print("[SERVO] Valve opened")
                
        elif message == "VALVE_CLOSE":
            if servo_valve:
                servo_valve.write_angle(0)
                current_valve_angle = 0
                print("[SERVO] Valve closed")
                
        elif message == "ROOF_OPEN":
            if servo_roof:
                servo_roof.write_angle(180)
                current_roof_angle = 180
                print("[SERVO] Roof opened")
                
        elif message == "ROOF_CLOSE":
            if servo_roof:
                servo_roof.write_angle(0)
                current_roof_angle = 0
                print("[SERVO] Roof closed")
                
    except Exception as e:
        print("❌ Servo command error:", str(e))

def handle_button_command(message):
    """Xử lý lệnh button"""
    if message == "BUTTON_TEST":
        test_all_buttons()
    elif message == "BUTTON_STATUS":
        send_button_status()

def handle_config_command(message):
    """Xử lý lệnh cấu hình"""
    global auto_mode
    
    try:
        config = json.loads(message)
        
        if "auto_mode" in config:
            auto_mode = config["auto_mode"]
            print(f"[CONFIG] Auto mode: {auto_mode}")
            
    except Exception as e:
        print("❌ Config command error:", str(e))

def handle_sensor_data(topic, message):
    """Xử lý dữ liệu cảm biến từ C++ code"""
    global last_sensor_data
    
    try:
        data = json.loads(message)
        sensor_type = topic.split("/")[2]
        
        last_sensor_data[sensor_type] = data
        
        # Cập nhật LCD nếu có dữ liệu mới
        update_lcd_with_sensor_data(sensor_type, data)
        
        # Thêm vào buffer M2M
        add_to_m2m_buffer(sensor_type, data)
        
        print(f"[SENSOR] Received {sensor_type} data: {data}")
        
    except Exception as e:
        print("❌ Sensor data error:", str(e))

def update_lcd_with_sensor_data(sensor_type, data):
    """Cập nhật LCD với dữ liệu cảm biến"""
    if lcd is None:
        return
        
    try:
        global lcd_update_counter
        lcd_update_counter += 1
        
        if sensor_type == "light":
            lcd.text(f"Light: {data.get('l', 0)}", 0, 0)
            
        elif sensor_type == "temp_humidity":
            temp = data.get('t', 0)
            hum = data.get('h', 0)
            lcd.text(f"Temp: {temp:.1f}C Hum:{hum:.1f}%", 0, 1)
            
        elif sensor_type == "soil_moisture":
            moisture = data.get('percentage', 0)
            lcd.text(f"Soil: {moisture:.0f}%", 0, 2)
            
        elif sensor_type == "npk":
            n = data.get('n_ppm', 0)
            p = data.get('p_ppm', 0)
            k = data.get('k_ppm', 0)
            lcd.text(f"NPK: N{n:.0f}P{p:.0f}K{k:.0f}", 0, 3)
            
    except Exception as e:
        print("❌ LCD update error:", str(e))

def add_to_m2m_buffer(sensor_type, data):
    """Thêm dữ liệu vào buffer M2M"""
    global m2m_data_buffer
    
    m2m_entry = {
        "sensor_type": sensor_type,
        "data": data,
        "timestamp": time.ticks_ms(),
        "source": "C++_BOARD3"
    }
    
    m2m_data_buffer.append(m2m_entry)
    
    # Giữ buffer trong giới hạn
    if len(m2m_data_buffer) > 20:
        m2m_data_buffer.pop(0)

def test_all_buttons():
    """Test tất cả buttons"""
    print("[BUTTON] Testing all buttons...")
    
    for button_name, button_pin in buttons.items():
        if not button_pin.value():  # Button pressed
            print(f"[BUTTON] {button_name}: PRESSED")
            led.on()
            time.sleep(0.2)
            led.off()
        else:
            print(f"[BUTTON] {button_name}: Released")

def send_button_status():
    """Gửi trạng thái các buttons"""
    if not mqtt_connected or not mqtt_client:
        return
        
    button_status = {}
    for button_name, button_pin in buttons.items():
        button_status[button_name] = not button_pin.value()  # Inverted logic
    
    status_msg = {
        "device_id": CLIENT_ID,
        "buttons": button_status,
        "timestamp": time.ticks_ms()
    }
    
    try:
        mqtt_client.publish(b"garden/status/board3_buttons", json.dumps(status_msg).encode())
        print("[BUTTON] Status sent:", button_status)
    except Exception as e:
        print("❌ Button status error:", str(e))

def setup_mqtt():
    """Khởi tạo MQTT"""
    global mqtt_client, mqtt_connected, reconnect_attempts
    
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
        
        max_retries = 3
        for attempt in range(max_retries):
            try:
                mqtt_client.connect(clean_session=True)
                mqtt_connected = True
                reconnect_attempts = 0
                print("✅ MQTT: Connected successfully")
                
                # Subscribe to topics
                for topic in SUBSCRIBE_TOPICS:
                    mqtt_client.subscribe(topic)
                    print(f"MQTT: Subscribed to {topic.decode()}")
                
                # Subscribe to sensor data from C++ code
                sensor_topics = [
                    b"garden/sensor/light",
                    b"garden/sensor/temp_humidity",
                    b"garden/sensor/soil_moisture",
                    b"garden/sensor/npk"
                ]
                
                for topic in sensor_topics:
                    mqtt_client.subscribe(topic)
                    print(f"MQTT: Subscribed to sensor: {topic.decode()}")
                
                # Send initial status
                mqtt_client.publish(STATUS_TOPIC, b"ONLINE")
                
                # Send device info
                info = {
                    "device_id": CLIENT_ID,
                    "type": "micropython_controller",
                    "version": "2.0-micropython",
                    "features": "lcd_control,servo_control,m2m_communication,button_handling",
                    "ip": wifi.ifconfig()[0],
                    "uptime": time.ticks_ms()
                }
                
                mqtt_client.publish(b"garden/system/device_info", json.dumps(info).encode())
                
                return True
                
            except Exception as conn_error:
                print(f"❌ MQTT connection attempt {attempt+1} failed:", str(conn_error))
                reconnect_attempts += 1
                
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

def send_m2m_data():
    """Gửi dữ liệu M2M"""
    global m2m_data_buffer
    
    if len(m2m_data_buffer) == 0:
        return
        
    try:
        # Tạo dữ liệu tổng hợp
        m2m_payload = {
            "device_id": CLIENT_ID,
            "type": "m2m_data_package",
            "data_count": len(m2m_data_buffer),
            "data": m2m_data_buffer.copy(),
            "timestamp": time.ticks_ms()
        }
        
        # Gửi qua MQTT
        mqtt_client.publish(b"garden/m2m/board3_data", json.dumps(m2m_payload).encode())
        
        # Gửi qua I2C (mô phỏng)
        send_i2c_data(m2m_payload)
        
        print(f"[M2M] Sent {len(m2m_data_buffer)} data packages")
        
        # Xóa buffer
        m2m_data_buffer.clear()
        
    except Exception as e:
        print("❌ M2M send error:", str(e))

def send_i2c_data(data):
    """Gửi dữ liệu qua I2C"""
    try:
        # Mô phỏng gửi I2C data
        i2c = I2C(1)
        json_str = json.dumps(data)
        
        # Trong thực tế sẽ dùng:
        # i2c.writeto(0x42, json_str.encode())
        
        print("[I2C] Would send M2M data to address 0x42")
        
    except Exception as e:
        print("❌ I2C send error:", str(e))

def handle_physical_buttons():
    """Xử lý buttons vật lý"""
    # Button 1: Toggle valve
    if not buttons['button1'].value():
        print("[PHYSICAL] Button 1 - Toggle valve")
        
        if current_valve_angle > 0:
            handle_servo_command("VALVE_CLOSE")
        else:
            handle_servo_command("VALVE_OPEN")
        
        led.on()
        time.sleep(0.2)
        led.off()
        time.sleep(0.1)
    
    # Button 2: Toggle roof
    if not buttons['button2'].value():
        print("[PHYSICAL] Button 2 - Toggle roof")
        
        if current_roof_angle < 90:
            handle_servo_command("ROOF_OPEN")
        else:
            handle_servo_command("ROOF_CLOSE")
        
        led.on()
        time.sleep(0.2)
        led.off()
        time.sleep(0.1)
    
    # Button 3: Toggle auto mode
    if not buttons['button3'].value():
        print("[PHYSICAL] Button 3 - Toggle auto mode")
        
        global auto_mode
        auto_mode = not auto_mode
        
        print(f"[PHYSICAL] Auto mode: {auto_mode}")
        
        led.on()
        time.sleep(0.5)
        led.off()
        time.sleep(0.1)
    
    # Button 4: Send status
    if not buttons['button4'].value():
        print("[PHYSICAL] Button 4 - Send status")
        
        send_button_status()
        
        led.on()
        time.sleep(0.3)
        led.off()
        time.sleep(0.1)

def send_status_report():
    """Gửi báo cáo trạng thái"""
    if not mqtt_connected or not mqtt_client:
        return
        
    try:
        status = {
            "device_id": CLIENT_ID,
            "status": "ONLINE",
            "valve_angle": current_valve_angle,
            "roof_angle": current_roof_angle,
            "auto_mode": auto_mode,
            "m2m_buffer_size": len(m2m_data_buffer),
            "uptime": time.ticks_ms(),
            "free_heap": gc.mem_free() if hasattr(gc, 'mem_free') else 0,
            "wifi_connected": wifi_connected,
            "mqtt_connected": mqtt_connected
        }
        
        mqtt_client.publish(STATUS_TOPIC, json.dumps(status).encode())
        print("📊 Status report sent")
        
    except Exception as e:
        print("❌ Status report error:", str(e))

def main():
    """Hàm chính"""
    global last_activity, mqtt_connected, wifi_connected
    
    print("🚀 Starting Board 3 MicroPython Controller...")
    
    gc.collect()
    
    # Initialize hardware
    if not setup_hardware():
        print("❌ Hardware initialization failed")
        return
    
    # Setup WiFi
    if not setup_wifi():
        print("❌ WiFi failed. Continuing offline...")
    
    # Setup MQTT
    if not setup_mqtt():
        print("❌ MQTT failed. Will retry in main loop...")
    
    print("✅ Board 3 MicroPython is ready!")
    
    # Main loop
    last_status_report = time.ticks_ms()
    last_m2m_send = time.ticks_ms()
    last_activity = time.ticks_ms()
    
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
            
            # Handle physical buttons
            handle_physical_buttons()
            
            # Send M2M data every minute
            if time.ticks_diff(current_time, last_m2m_send) > 60000:
                send_m2m_data()
                last_m2m_send = current_time
            
            # Status report every 5 minutes
            if time.ticks_diff(current_time, last_status_report) > 300000:
                send_status_report()
                last_status_report = current_time
            
            # Handle auto mode
            if auto_mode and last_sensor_data:
                handle_auto_mode()
            
            time.sleep_ms(100)
            
        except KeyboardInterrupt:
            print("\n🛑 Program interrupted by user")
            break
        except Exception as e:
            print("❌ Main loop error:", str(e))
            print("🔄 Attempting recovery...")
            time.sleep(2)

def handle_auto_mode():
    """Xử lý chế độ tự động"""
    global current_valve_angle, current_roof_angle
    
    try:
        # Auto valve based on soil moisture
        if "soil_moisture" in last_sensor_data:
            soil_data = last_sensor_data["soil_moisture"]
            moisture_pct = soil_data.get("percentage", 50)
            
            if moisture_pct < 30 and current_valve_angle == 0:
                handle_servo_command("VALVE_OPEN")
                print("[AUTO] Low soil moisture - Opening valve")
            elif moisture_pct > 70 and current_valve_angle > 0:
                handle_servo_command("VALVE_CLOSE")
                print("[AUTO] High soil moisture - Closing valve")
        
        # Auto roof based on temperature
        if "temp_humidity" in last_sensor_data:
            temp_data = last_sensor_data["temp_humidity"]
            temperature = temp_data.get("t", 25)
            
            if temperature > 32 and current_roof_angle < 180:
                handle_servo_command("ROOF_OPEN")
                print("[AUTO] High temperature - Opening roof")
            elif temperature < 20 and current_roof_angle > 0:
                handle_servo_command("ROOF_CLOSE")
                print("[AUTO] Low temperature - Closing roof")
                
    except Exception as e:
        print("❌ Auto mode error:", str(e))

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("💥 Fatal error:", str(e))
        print("🔄 System will reboot in 10 seconds...")
        time.sleep(10)
        reset()
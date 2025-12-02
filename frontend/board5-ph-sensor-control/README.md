# 🌱 Board 7: pH Sensor Control - Smart Garden IoT

## 📋 **Tổng Quan**
Board 7 là hệ thống điều khiển pH tự động cho Smart Garden, sử dụng ESP32 để đo độ pH của đất/nước và tự động điều chỉnh bằng hệ thống dosing acid/alkaline.

## 🎯 **Chức Năng Chính**
- 📊 **Đo độ pH real-time** (0-14 pH range)
- 🎛️ **Tự động điều chỉnh pH** bằng acid/alkaline dosing
- 📱 **Remote control** qua MQTT
- 🖥️ **LCD display** hiển thị thông tin pH
- 🚨 **Alert system** với buzzer và LEDs
- 🔧 **Calibration** system cho sensor
- ⚡ **Auto/Manual mode** switch

## 🔧 **Phần Cứng**

### **Components:**
- **ESP32 DevKit v1** - Main controller
- **pH Sensor** - Đo độ pH (analog + digital output)
- **2x Continuous Servos** - Acid & Alkaline dosing
- **3x Status LEDs** (Red: Acid, Blue: Alkaline, Green: Optimal)
- **1x Status LED** (Yellow: System status)
- **LCD 16x2** - Hiển thị thông tin
- **Calibration Button** - Manual calibration
- **Threshold Potentiometer** - Adjust pH setpoint
- **Auto/Manual Switch** - Control mode selection
- **Buzzer** - Alert notifications

### **Pin Connections:**
```
ESP32 Pin    | Component
-------------|------------------
GPIO34       | pH Sensor Analog
GPIO35       | pH Sensor Digital
GPIO4        | Acid Dosing Servo
GPIO5        | Alkaline Dosing Servo
GPIO25       | Acid LED (Red)
GPIO26       | Alkaline LED (Blue)
GPIO27       | Optimal LED (Green)
GPIO2        | Status LED (Yellow)
GPIO23       | Calibration Button
GPIO32       | Threshold Potentiometer
GPIO33       | Auto/Manual Switch
GPIO13       | Alert Buzzer
GPIO21       | LCD SDA (I2C)
GPIO22       | LCD SCL (I2C)
```

## 📡 **MQTT Communication**

### **Topics:**
```
Publishing:
- garden/sensor/ph - Real-time pH data
- garden/log - System logs and alerts

Subscribing:
- garden/control/ph_ - Control commands
```

### **Data Format:**
```json
{
  "device_id": "ESP_BOARD7_PH_01",
  "ph_value": 7.2,
  "ph_status": "optimal",
  "digital_value": 1,
  "analog_raw": 2150,
  "threshold": 7.0,
  "auto_mode": true,
  "calibration": 0.1,
  "acid_dosing_active": false,
  "alkaline_dosing_active": false,
  "timestamp": 1640995200000
}
```

### **Control Commands:**
```json
// Set auto mode
{"action": "auto_mode", "enabled": true}

// Calibrate sensor
{"action": "calibrate"}

// Manual dosing
{"action": "manual_dosing", "type": "acid", "duration": 3000}

// Set pH threshold
{"action": "set_threshold", "threshold": 6.5}
```

## 🎛️ **Logic Điều Khiển**

### **pH Ranges:**
- **Strong Acidic**: pH < 5.0 (🔴 Red LED)
- **Acidic**: pH 5.0-6.0 (🔴 Red LED)
- **Optimal**: pH 6.5-7.5 (🟢 Green LED)
- **Alkaline**: pH 7.5-8.0 (🔵 Blue LED)
- **Strong Alkaline**: pH > 8.0 (🔵 Blue LED)

### **Dosing Logic:**
```
IF pH < (threshold - 0.5) THEN
  → Start ALKALINE dosing (increase pH)
  
IF pH > (threshold + 0.5) THEN
  → Start ACID dosing (decrease pH)
  
IF 6.5 ≤ pH ≤ 7.5 THEN
  → STOP all dosing (optimal range)
  
Dosing duration: 5 seconds
Cooldown period: 30 seconds
```

### **Manual Control:**
- **Manual Mode**: User controls dosing via MQTT
- **Auto Mode**: System automatically adjusts pH
- **Calibration**: Use buffer solution pH 7.0 for reference

## 📊 **Dashboard Integration**

### **Real-time Data:**
- Current pH value (0-14)
- pH status (Acidic/Optimal/Alkaline)
- Dosing status (Active/Inactive)
- Threshold setting
- Auto/Manual mode status

### **Control Interface:**
- Manual acid dosing button
- Manual alkaline dosing button
- Auto mode toggle
- Threshold adjustment slider
- Calibration trigger

### **Alerts:**
- High/Low pH warnings
- Dosing system status
- Sensor calibration reminders
- System connectivity status

## 🔧 **Setup & Calibration**

### **Initial Setup:**
1. **Install Components** - Connect all hardware as per diagram
2. **Upload Code** - Flash ESP32 with board7 code
3. **Network Config** - Connect to WiFi and MQTT broker
4. **Baseline Calibration** - Use pH 7.0 buffer solution
5. **Threshold Setting** - Adjust potentiometer for desired pH
6. **Test Dosing** - Verify acid/alkaline servos work

### **Calibration Process:**
1. Prepare pH 7.0 buffer solution
2. Place sensor in buffer
3. Press calibration button OR send MQTT command
4. System takes 10 readings and calculates offset
5. LEDs blink to confirm calibration

### **Maintenance:**
- **Daily**: Check pH readings and dosing status
- **Weekly**: Verify calibration accuracy
- **Monthly**: Clean sensor probe
- **Quarterly**: Replace buffer solutions

## 🚨 **Safety Features**

### **Emergency Stop:**
- Manual emergency stop via button
- Automatic stop if pH goes out of safe range (0-14)
- Dosing timeout protection (max 5 seconds)
- System cooldown between doses (30 seconds)

### **Alerts:**
- Buzzer sound for critical pH levels
- LED indicators for current pH status
- MQTT notifications for remote monitoring
- LCD display for local status

### **Redundancy:**
- Manual mode backup if auto mode fails
- Multiple timeout protections
- Sensor validation checks
- Network connectivity monitoring

## 📱 **Mobile App Integration**

### **Key Features:**
- Real-time pH monitoring
- Push notifications for alerts
- Remote manual control
- Historical pH data charts
- Calibration reminders
- Dosing system diagnostics

### **User Interface:**
```
┌─────────────────────────┐
│  🌱 pH Control Center   │
├─────────────────────────┤
│ Current pH: 7.2 📊      │
│ Status: Optimal 🟢      │
│ Auto: ON 🔄             │
├─────────────────────────┤
│ [Acid Dosing] [Alkali]  │
│ [Calibrate] [Settings]  │
├─────────────────────────┤
│ 📈 24h History Chart    │
└─────────────────────────┘
```

## 🔬 **Scientific Background**

### **pH Scale:**
- **0-6.9**: Acidic (H+ ions dominate)
- **7**: Neutral (balanced H+ and OH-)
- **7.1-14**: Alkaline (OH- ions dominate)

### **Plant pH Requirements:**
- **Most vegetables**: 6.0-7.5
- **Tomatoes**: 6.0-6.8
- **Potatoes**: 5.5-6.5
- **Carrots**: 6.0-6.8
- **Leafy greens**: 6.0-7.0

### **Dosing Chemicals:**
- **Acid dosing**: Phosphoric acid (H₃PO₄) or sulfuric acid (H₂SO₄)
- **Alkaline dosing**: Sodium hydroxide (NaOH) or calcium carbonate (CaCO₃)
- **Concentration**: 0.1-1.0 M solutions recommended

## 🎯 **Expected Performance**

### **Accuracy:**
- **pH reading**: ±0.1 pH units
- **Response time**: < 5 seconds
- **Dosing precision**: ±0.2 pH units
- **Calibration stability**: ±0.05 pH units/month

### **Reliability:**
- **Uptime**: > 99%
- **Dosing success rate**: > 95%
- **Network connectivity**: > 98%
- **Sensor lifetime**: 6-12 months

## 📈 **Data Analytics**

### **Metrics Tracked:**
- pH trend analysis
- Dosing frequency patterns
- System efficiency metrics
- Plant health correlation
- Environmental factor impact

### **Insights Provided:**
- Optimal dosing schedules
- Seasonal pH adjustments
- Nutrient availability predictions
- Plant health recommendations
- System optimization suggestions

---

## 🎉 **Board 7 Ready for Integration!**

Board 7 pH Sensor Control là một hệ thống hoàn chỉnh cho việc quản lý pH trong Smart Garden. Với khả năng đo chính xác, điều khiển tự động và giao diện thân thiện, hệ thống này sẽ đảm bảo pH luôn ở mức tối ưu cho sự phát triển của cây trồng.

**🌱 Happy Gardening with Perfect pH! 🌱**
# 🌱 Smart Garden IoT System - ESP32 NodeJS Wokwi

> **Hệ thống IoT vườn thông minh hoàn chỉnh với 6 board ESP32, MQTT, NodeJS API, và giao diện web/mobile**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ESP32](https://img.shields.io/badge/ESP32-Compatible-green.svg)](https://www.espressif.com/en/products/socs/esp32)
[![MQTT](https://img.shields.io/badge/MQTT-5.0-blue.svg)](https://mqtt.org/)
[![NodeJS](https://img.shields.io/badge/NodeJS-18+-brightgreen.svg)](https://nodejs.org/)

## 📋 Mục Lục

- [Tổng Quan Hệ Thống](#tổng-quan-hệ-thống)
- [Kiến Trúc Hệ Thống](#kiến-trúc-hệ-thống)
- [Các Thành Phần](#các-thành-phần)
- [Hướng Dẫn Cài Đặt](#hướng-dẫn-cài-đặt)
- [Hướng Dẫn Sử Dụng](#hướng-dẫn-sử-dụng)
- [API Documentation](#api-documentation)
- [Demo và Mô Phỏng](#demo-và-mô-phỏng)
- [Đóng Góp](#đóng-góp)
- [Giấy Phép](#giấy-phép)

## 🌟 Tổng Quan Hệ Thống

Smart Garden IoT System là một hệ thống vườn thông minh toàn diện được thiết kế để tự động hóa việc chăm sóc cây trồng thông qua các cảm biến IoT và điều khiển từ xa.

### 🎯 Tính Năng Chính

- **🌡️ Giám sát môi trường thời gian thực** (nhiệt độ, độ ẩm, ánh sáng)
- **💧 Tưới nước tự động** dựa trên độ ẩm đất
- **🔬 Phân tích dinh dưỡng NPK** trong đất
- **🏠 Điều khiển mái che và đèn chiếu sáng** thông minh
- **📱 Giao diện web/mobile** để giám sát và điều khiển
- **🔄 Giao tiếp M2M** giữa các thiết bị
- **🚨 Hệ thống cảnh báo** thông minh
- **📊 Lưu trữ và phân tích dữ liệu** với MongoDB

## 🏗️ Kiến Trúc Hệ Thống

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   ESP32 Boards  │    │   MQTT Broker    │    │   NodeJS API    │
│                 │    │                  │    │                 │
│ • Board 1: Light│    │ • broker.hivemq │    │ • REST API      │
│ • Board 2: Soil │◄──►│ • Port: 1883    │◄──►│ • WebSocket     │
│ • Board 3: Temp │    │ • Topic Structure│    │ • Analytics     │
│ • Board 4: NPK  │    │ • Auto Logic    │    │ • Control CMD   │
│ • Board 5: Pump │    │ • Alert System  │    │                 │
│ • Board 6: Env  │    │                  │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Wokwi Sim     │    │   MongoDB Atlas  │    │   Web/Mobile    │
│                 │    │                  │    │                 │
│ • Virtual HW    │    │ • Sensor Data   │    │ • Dashboard     │
│ • Real-time     │    │ • Device Status │    │ • Control Panel │
│ • Testing       │    │ • Alerts        │    │ • Real-time     │
│ • Development   │    │ • Analytics     │    │ • Notifications │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## 🔧 Các Thành Phần

### 📱 Frontend ESP32 (6 Boards)

#### Board 1: Light Sensor Control (C++)

- **Chức năng**: Đo ánh sáng, điều khiển đèn LED PWM, relay, servo
- **Công nghệ**: C++, I2C/SPI, PWM, Interrupt, WatchDog
- **Thiết bị**: LDR, LED, Relay, 2x Servo, 3x Switch, Potentiometer

#### Board 2: Soil Moisture Control (C++)

- **Chức năng**: Đo độ ẩm đất, tưới nước tự động
- **Công nghệ**: C++, I2C/SPI, Digital Analog, Servo
- **Thiết bị**: Soil moisture sensor, Servo valve, Relay pump, Calibration

#### Board 3: Temperature + M2M Control (C++ + MicroPython)

- **Chức năng**: Cảm biến nhiệt độ, giao tiếp M2M, LCD hiển thị
- **Công nghệ**: C++ cho cảm ccp, MicroPython cho điều khiển, M2M
- **Thiết bị**: DHT22, LCD I2C, 2x Servo, 3x Buttons, M2M I2C

#### Board 4: NPK Sensor Control (C++)

- **Chức năng**: Đo dinh dưỡng NPK, điều khiển tưới phân
- **Công nghệ**: C++, Digital Analog, LCD, Servo
- **Thiết bị**: NPK sensor, LCD display, Servo valve, Calibration

#### Board 5: Pump PWM Control (C++)

- **Chức năng**: Điều khiển bơm với PWM, WatchDog timer
- **Công nghệ**: C++, PWM, WatchDog, Multiple relays, Flow sensor
- **Thiết bị**: Pump relay, Main valve, Emergency valve, Flow sensor, 3x Buttons

#### Board 6: Environmental Monitor (C++)

- **Chức năng**: Giám sát môi trường tổng hợp, I2C/SPI hub
- **Công nghệ**: C++, Multiple I2C/SPI devices, Data aggregation
- **Thiết bị**: Multi-sensor array, I2C communication, SPI data exchange

### 🖥️ Backend NodeJS

#### MQTT Broker Server

- **Vị trí**: `backend/broker/server.js`
- **Chức năng**:
  - Xử lý tất cả dữ liệu MQTT
  - Logic tự động hóa thông minh
  - Quản lý thiết bị và trạng thái
  - Hệ thống cảnh báo và thông báo
  - M2M communication handling

#### REST API Server

- **Vị trí**: `backend/api/server.js`
- **Chức năng**:
  - REST API cho web và mobile
  - Real-time WebSocket data
  - Analytics và reporting
  - Device control endpoints
  - Alert management

### 🌐 Frontend Web (NextJS)

#### Dashboard Application

- **Vị trí**: `frontend/web/`
- **Chức năng**:
  - Giao diện giám sát thời gian thực
  - Panel điều khiển thiết bị
  - Biểu đồ và analytics
  - Quản lý cảnh báo
  - Responsive design

## 🚀 Hướng Dẫn Cài Đặt

### Yêu Cầu Hệ Thống

- **Node.js** >= 18.0.0
- **MongoDB Atlas** account
- **Wokwi Simulator** account (for ESP32 simulation)
- **MQTT Broker**: broker.hivemq.com (public)

### 1. Clone Repository

```bash
git clone <repository-url>
cd smart-garden-iot-system
```

### 2. Backend Setup

```bash
cd backend
npm install

# Tạo file .env
cp .env.example .env

# Chỉnh sửa file .env với thông tin MongoDB Atlas của bạn
# MONGODB_URI=mongodb+srv://username:password@cluster.mongodb.net/smartgarden
```

### 3. Frontend Setup

```bash
cd ../frontend/web
npm install

# Cấu hình API URL trong .env.local
echo "NEXT_PUBLIC_API_URL=http://localhost:3001/api" > .env.local
```

### 4. ESP32 Boards Setup

Mỗi board có thể chạy trong Wokwi:

```bash
# Board 1: Light Control
frontend/board1-light-sensor-control/

# Board 2: Soil Moisture
frontend/board2-soil-sensor-control/

# Board 3: Temperature + M2M
frontend/board3-temp-m2m-control/
# - main.ino (C++ for sensors)
# - micropython/main.py (MicroPython for control)

# Board 4: NPK Control
frontend/board4-npk-sensor-control/

# Board 5: Pump Control
frontend/board5-pump-pwm-control/

# Board 6: Environmental Monitor
frontend/board6-env-monitor/
```

## 🏃‍♂️ Chạy Hệ Thống

### 1. Khởi động Backend

```bash
cd backend

# Chạy MQTT Broker
npm run start:broker

# Chạy API Server (terminal mới)
npm run start:api

# Hoặc chạy cả hai
npm run start:all
```

### 2. Khởi động Frontend Web

```bash
cd frontend/web
npm run dev
```

### 3. Chạy ESP32 Boards trên Wokwi

1. Mở [Wokwi Simulator](https://wokwi.com)
2. Upload file `diagram.json` của từng board
3. Upload code tương ứng (`.ino` cho C++, `main.py` cho MicroPython)
4. Start simulation

## 📊 API Documentation

### REST Endpoints

#### Sensor Data

```
GET /api/sensors/current          # Dữ liệu cảm biến hiện tại
GET /api/sensors/history/:type    # Lịch sử dữ liệu cảm biến
```

#### Device Control

```
GET  /api/devices/status          # Trạng thái thiết bị
POST /api/control/:device         # Gửi lệnh điều khiển
```

#### Alerts

```
GET  /api/alerts                  # Danh sách cảnh báo
POST /api/alerts/:id/acknowledge  # Xác nhận cảnh báo
```

#### Analytics

```
GET /api/analytics/overview       # Tổng quan analytics
```

#### Configuration

```
GET  /api/config/thresholds       # Lấy ngưỡng cảnh báo
POST /api/config/thresholds       # Cập nhật ngưỡng
```

### MQTT Topics

#### Sensor Data (Publish)

```
garden/sensor/light              # Dữ liệu ánh sáng
garden/sensor/temp_humidity      # Dữ liệu nhiệt độ/độ ẩm
garden/sensor/soil_moisture      # Dữ liệu độ ẩm đất
garden/sensor/npk                # Dữ liệu dinh dưỡng NPK
garden/sensor/pump_status        # Trạng thái bơm
garden/sensor/environment        # Dữ liệu môi trường
```

#### Control Commands (Subscribe)

```
garden/control/light             # Điều khiển đèn
garden/control/water_valve       # Điều khiển van nước
garden/control/roof_servo        # Điều khiển mái che
garden/control/pump_start        # Khởi động bơm
garden/control/pump_stop         # Dừng bơm
```

#### Device Status

```
garden/status/*                  # Trạng thái thiết bị
garden/alerts/*                  # Cảnh báo hệ thống
garden/system/*                  # Thông tin hệ thống
```

## 🧪 Demo và Mô Phỏng

### Wokwi Simulation

Mỗi ESP32 board có `diagram.json` riêng để mô phỏng trong Wokwi Simulator:

#### 🏗️ Các Board ESP32 trong Wokwi

1. **Board 1 (Light Control)**: `<filepath>`frontend/board1-light-sensor-control/diagram.json`</filepath>`

   - **Hardware**: LDR sensor + PWM LED + 2x Servo + 3x Switches + Relay
   - **Chức năng**: Tự động điều khiển đèn và mái che theo ánh sáng
   - **Pins**: LDR(32), LED PWM(2), Servo Valve(4), Servo Roof(13), Switches(25,26,27), Relay(5)
2. **Board 2 (Soil Moisture)**: `<filepath>`frontend/board2-soil-sensor-control/diagram.json`</filepath>`

   - **Hardware**: Soil moisture sensor + Servo valve + Relay pump + LCD + Potentiometers
   - **Chức năng**: Tưới nước tự động theo độ ẩm đất với LCD hiển thị
   - **Pins**: Soil(35/34), Servo Valve(4), Relay Pump(5), LCD I2C(21/22), Buttons(25)
3. **Board 3 (Temperature + M2M)**: `<filepath>`frontend/board3-temp-m2m-control/diagram.json`</filepath>`

   - **Hardware**: DHT22 + LCD 20x4 + 2x Servo + 3x Buttons + M2M Communication
   - **Chức năng**: Đo nhiệt độ/độ ẩm, hiển thị LCD, giao tiếp M2M với board khác
   - **Pins**: DHT22(4), LCD I2C(21/22), Valve Servo(12), Roof Servo(13), Buttons(25,26,27)
4. **Board 4 (NPK Sensor)**: `<filepath>`frontend/board4-npk-sensor-control/diagram.json`</filepath>`

   - **Hardware**: NPK sensor + LCD + Servo valve + Calibration potentiometer
   - **Chức năng**: Phân tích dinh dưỡng NPK và tưới phân tự động
   - **Pins**: N(34), P(35), K(32), Servo Valve(4), LCD I2C(21/22), Button(25)
5. **Board 5 (Pump Control)**: `<filepath>`frontend/board5-pump-pwm-control/diagram.json`</filepath>`

   - **Hardware**: PWM pump + Flow sensor + 3x Relays + 3x Buttons + Pressure sensor
   - **Chức năng**: Điều khiển bơm an toàn với WatchDog timer và giám sát lưu lượng
   - **Pins**: PWM Pump(4), Main Valve Relay(5), Emergency Valve Relay(18), Flow(19), Pressure(34)
6. **Board 6 (Environmental Monitor)**: `<filepath>`frontend/board6-env-monitor/diagram.json`</filepath>`

   - **Hardware**: Multi-sensor array (LDR, Temperature, Soil) + 4x I2C devices + SPI communication
   - **Chức năng**: Tổng hợp dữ liệu môi trường và làm communication hub
   - **Pins**: LDR(32), Soil(35), I2C devices (21/22), SPI (23/19/18/5), Mode Button(25)

#### 🔧 Custom Chips cho Wokwi

Để mô phỏng chính xác, hệ thống bao gồm các custom chip:

- **NPK Sensor**: `<filepath>`frontend/custom-chips/npk-sensor.js`</filepath>`

  - Mô phỏng cảm biến dinh dưỡng NPK với đầu ra analog cho từng chất
  - Hỗ trợ cấu hình giá trị ppm và hiệu chỉnh
- **Soil Moisture Sensor**: `<filepath>`frontend/custom-chips/soil-moisture-sensor.js`</filepath>`

  - Mô phỏng cảm biến độ ẩm đất với đầu ra digital và analog
  - Có thể điều chỉnh ngưỡng và độ nhạy

#### 🚀 Hướng Dẫn Chạy Mô Phỏng

1. **Mở Wokwi Simulator**: Truy cập [https://wokwi.com](https://wokwi.com)
2. **Upload Diagram**: Chọn file `diagram.json` của board cần mô phỏng
3. **Upload Code**:

   - File `main.ino` cho các board C++
   - File `main.py` cho board MicroPython (Board 3 controller)
4. **Configure Custom Chips** (nếu cần):

   - Copy nội dung file custom chip vào Wokwi
   - Cấu hình giá trị mặc định cho cảm biến
5. **Start Simulation**: Nhấn nút "Start" để bắt đầu mô phỏng

#### 📊 Monitoring trong Wokwi

- **Serial Monitor**: Xem log hoạt động của ESP32
- **Pin States**: Theo dõi trạng thái các pin
- **Custom Variables**: Giá trị của cảm biến và biến điều khiển
- **MQTT Messages**: Monitor giao tiếp MQTT trong real-time

### Real-time Testing

1. **Web Dashboard**: Mở `http://localhost:3000` để xem dashboard
2. **MQTT Testing**: Sử dụng MQTT Explorer để monitor messages
3. **API Testing**: Test các endpoints với Postman hoặc curl

## 🛠️ Tính Năng Nâng Cao

### Automation Logic

Hệ thống có logic tự động hóa thông minh:

```javascript
// Ví dụ: Tự động tưới khi đất khô
if (soilMoisture < 30%) {
  openValve();
  startPump();
  
  setTimeout(() => {
    closeValve();
    stopPump();
  }, 5000);
}

// Điều khiển mái che theo nhiệt độ
if (temperature > 32°C) {
  openRoof();
} else if (temperature < 20°C) {
  closeRoof();
}
```

### M2M Communication

Board 3 (MicroPython) giao tiếp với Board 1 (C++) qua I2C:

```python
# MicroPython code
def send_m2m_data(data):
    i2c.writeto(0x42, json.dumps(data).encode())
```

### WatchDog System

Board 5 có WatchDog timer để đảm bảo an toàn:

```cpp
// ESP32 WatchDog
esp_task_wdt_init(15, true);  // 15 seconds timeout
esp_task_wdt_add(NULL);
```

### Alert System

Hệ thống cảnh báo thông minh với phân cấp độ ưu tiên:

- **CRITICAL**: Máy bơm chạy quá lâu, nhiệt độ nguy hiểm
- **WARNING**: Độ ẩm thấp, ánh sáng yếu
- **INFO**: Thiết bị online/offline, bảo trì định kỳ

## 📱 Mobile App (Tùy chọn)

Có thể phát triển mobile app sử dụng React Native hoặc Flutter với cùng API endpoints.

## 🔧 Customization

### Thêm Cảm Biến Mới

1. Tạo board mới trong `frontend/`
2. Thêm MQTT topics mới
3. Cập nhật automation logic
4. Thêm API endpoints nếu cần

### Tùy Chỉnh Ngưỡng

```javascript
// backend/broker/automation.js
const THRESHOLDS = {
  LIGHT: { LOW: 300, HIGH: 900 },
  TEMPERATURE: { MIN: 18, MAX: 35 },
  HUMIDITY: { MIN: 40, MAX: 80 },
  SOIL_MOISTURE: { CRITICAL: 30, LOW: 40, OPTIMAL: 60 }
};
```

### Thêm Tính Năng Web

Mở rộng dashboard trong `frontend/web/pages/index.tsx` với các component mới.

## 🤝 Đóng Góp

1. Fork repository
2. Tạo feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit changes (`git commit -m 'Add AmazingFeature'`)
4. Push to branch (`git push origin feature/AmazingFeature`)
5. Mở Pull Request

## 📝 TODO / Future Features

- [ ] Mobile app (React Native)
- [ ] Machine learning cho dự đoán
- [ ] Weather API integration
- [ ] Email/SMS notifications
- [ ] Multi-garden support
- [ ] Advanced analytics dashboard
- [ ] Energy monitoring
- [ ] Plant disease detection
- [ ] Automated fertilization system

## 📞 Hỗ Trợ

Nếu có vấn đề hoặc câu hỏi:

1. Tạo Issue trong repository
2. Check documentation
3. Review code examples

## 📄 Giấy Phép

Distributed under the MIT License. See `LICENSE` for more information.

## 🏆 Credits

- **ESP32 Development**: Espressif Systems
- **MQTT Broker**: HiveMQ Public Broker
- **Simulation**: Wokwi Simulator
- **Database**: MongoDB Atlas
- **Framework**: Next.js, Node.js, Express.js

---

**🌱 Smart Garden IoT System - Growing Plants with Intelligence 🌱**

Made with ❤️ for sustainable agriculture and IoT innovation.

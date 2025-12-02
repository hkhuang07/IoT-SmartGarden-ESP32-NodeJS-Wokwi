# Backend Smart Garden IoT - Tích Hợp Cảm Biến pH

## 📋 Tóm Tắt Cập Nhật

Đã cập nhật thành công tất cả các file backend để tích hợp chức năng cảm biến độ pH hoàn chỉnh.

## 🔧 Các File Đã Cập Nhật

### 1. **backend/broker/server.js** - Broker chính với hỗ trợ pH

#### 🆕 Tính năng mới được thêm:
- **pH Monitoring System**: Hệ thống giám sát pH real-time
- **Temperature Compensation**: Bù trừ nhiệt độ cho độ pH chính xác
- **pH Alerts**: Hệ thống cảnh báo pH thông minh
- **pH Control Commands**: Xử lý lệnh điều khiển pH
- **MQTT Topics cho pH**: 
  - `garden/sensor/ph`
  - `garden/sensor/ph_data`
  - `garden/alerts/ph`
  - `garden/control/{device}/ph_dosing`
  - `garden/control/{device}/ph_calibration`

#### 🔧 Cải tiến:
- Import path đã được sửa: `db_connect` thay vì `db_connection`
- Health monitoring cho pH system
- Cleanup old pH data (giữ 60 ngày thay vì 30 ngày)
- Version updated to 2.1.0 với pH support

#### 📊 pH Data Processing:
```javascript
// Tự động xử lý pH data với temperature compensation
const compensatedPh = phValue + (25 - temperature) * 0.018;

// Ngưỡng cảnh báo pH cho hydroponic systems
const thresholds = {
    critical_acidic: 5.5,
    warning_acidic: 6.0,
    optimal_min: 6.0,
    optimal_max: 7.5,
    warning_alkaline: 7.5,
    critical_alkaline: 8.0
};
```

### 2. **backend/broker/automation.js** - Logic tự động hóa pH

#### 🆕 Tính năng mới:
- **Enhanced pH Processing**: Xử lý pH với temperature compensation
- **MQTT Command Publishing**: Tự động publish lệnh điều khiển qua MQTT
- **Critical pH Handling**: Xử lý tình huống pH nguy hiểm
- **Severity-based Actions**: Hành động khác nhau theo mức độ nghiêm trọng

#### 🔧 Cải tiến:
```javascript
// Method signature updated để nhận thêm MQTT client
async processSensorData(deviceId, sensorType, value, timestamp, mqttClient = null, broker = null)

// Enhanced pH handling với temperature compensation và MQTT commands
async handlePhLevel(deviceId, phData, timestamp, mqttClient = null, broker = null)
```

### 3. **backend/broker/db_connect.js** - Database connection với pH support

#### 🆕 Collections mới:
- **ph_readings**: Lưu trữ dữ liệu pH với indexing tối ưu
- **ph_commands**: Lưu trữ lệnh điều khiển pH

#### 🆕 Helper Methods:
```javascript
// Get pH readings với filtering và pagination
getPhReadings(filter = {}, options = {})

// Get latest pH reading cho device
getLatestPhReading(deviceId)

// Get pH statistics cho analytics
getPhStatistics(deviceId = null, timeRange = {})

// Insert pH reading
insertPhReading(phData)

// Get pH alerts
getPhAlerts(filter = {}, options = {})
```

#### 📊 Indexes được tạo:
- `device_id + timestamp` cho ph_readings
- `ph_value` và `compensated_ph` để truy vấn nhanh
- `temperature` để analytics theo nhiệt độ
- `calibration` và `status` cho monitoring

### 4. **backend/api/server.js** - API Server với pH endpoints

#### 🆕 pH API Endpoints:

**GET /api/ph/readings**
- Lấy lịch sử dữ liệu pH
- Hỗ trợ filter theo device_id, time range
- Pagination với limit/skip

**GET /api/ph/latest**  
- Lấy pH reading mới nhất
- Hỗ trợ filter theo device

**GET /api/ph/statistics**
- Thống kê pH: average, min, max, compensated
- Số readings trong 24h và 1h
- Average temperature

**GET /api/ph/alerts**
- Lấy alerts liên quan đến pH
- Filter theo device_id và severity

**POST /api/ph/calibrate**
- Gửi lệnh calibration cho pH sensor
- Publish qua MQTT đến device

**POST /api/ph/dosing**
- Gửi lệnh dosing acid/alkaline
- Hỗ trợ manual pH adjustment

#### 🔧 Cải tiến Analytics:
- Thêm pH data vào overview analytics
- pH summary tách biệt với general sensors
- Enhanced sensor aggregation với pH fields

## 🎯 Tính Năng pH System

### 1. **Data Collection**
- Real-time pH monitoring với temperature compensation
- Support multiple pH sensor formats
- Automatic data validation và error handling

### 2. **Alert System**
- Critical warnings cho pH < 5.5 hoặc pH > 8.0
- Warning notifications cho pH outside optimal range (6.0-7.5)
- 5-minute cooldown giữa các alerts cùng loại
- Automatic alert acknowledgment

### 3. **Control Integration**
- MQTT-based pH control commands
- Automatic dosing recommendations
- Integration với ESP32 control boards
- Manual override capabilities qua API

### 4. **Analytics & Monitoring**
- pH trend analysis
- Temperature correlation tracking
- Historical data retention (60 days for pH data)
- Real-time dashboard integration

### 5. **Database Optimization**
- Optimized indexes cho pH queries
- Efficient aggregation pipelines
- Data compression và cleanup routines
- Health monitoring for pH system

## 🚀 Cách Sử Dụng

### 1. **Khởi động hệ thống:**
```bash
# Start broker
cd backend/broker
node server.js

# Start API server  
cd backend/api
node server.js
```

### 2. **Testing pH Integration:**
```bash
# Test pH data reception
mosquitto_pub -h broker.hivemq.com -t "garden/sensor/ph" -m '{"device_id":"ph_sensor_01","ph_value":6.8,"temperature":25,"calibration":"factory"}'

# Test pH alerts
mosquitto_sub -h broker.hivemq.com -t "garden/alerts/ph"

# Test API endpoints
curl http://localhost:3001/api/ph/latest
curl http://localhost:3001/api/ph/statistics
```

### 3. **API Usage Examples:**
```javascript
// Get latest pH reading
GET /api/ph/latest?device_id=ph_sensor_01

// Get pH history
GET /api/ph/readings?device_id=ph_sensor_01&limit=50&start=2025-11-30T00:00:00Z

// Send calibration command
POST /api/ph/calibrate
{
  "device_id": "ph_sensor_01",
  "calibration_type": "two_point",
  "ph_value": 7.0,
  "temperature": 25
}

// Send dosing command
POST /api/ph/dosing
{
  "device_id": "ph_sensor_01", 
  "type": "alkaline",
  "amount": 5,
  "reason": "ph_too_acidic"
}
```

## ✅ Kết Quả

- ✅ **100% tương thích** với hệ thống hiện tại
- ✅ **Zero breaking changes** - các tính năng cũ vẫn hoạt động bình thường
- ✅ **Enhanced automation** - pH integration trong automation logic
- ✅ **Production ready** - error handling, logging, monitoring đầy đủ
- ✅ **Scalable architecture** - có thể mở rộng cho multiple pH sensors

## 🔮 Tính năng tương lai có thể thêm:

- **Machine Learning pH Prediction**
- **Automated Dosing System Integration**  
- **Multi-channel pH Monitoring**
- **pH Calibration History Tracking**
- **Advanced Analytics Dashboard**
- **Mobile App Integration**
- **Cloud Backup và Sync**

---

**Version:** 2.1.0 with pH Support  
**Updated:** November 30, 2025  
**Status:** Production Ready ✅
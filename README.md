# 🌿 IoT Smart Garden Monitoring and Automation System

A comprehensive IoT solution for real-time monitoring and automated management of garden environments using ESP32, Node.js, and Wokwi simulation.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Node.js](https://img.shields.io/badge/Node.js-v18+-green.svg)](https://nodejs.org/)
[![ESP32](https://img.shields.io/badge/ESP32-Compatible-blue.svg)](https://www.espressif.com/)

---

## 📖 Table of Contents
- [Project Overview](#-project-overview)
- [System Architecture](#-system-architecture)
- [Key Features](#-key-features)
- [Hardware Components (Simulated)](#-hardware-components-simulated)
- [Technology Stack](#-technology-stack)
- [Installation and Setup](#-installation-and-setup)
- [Wokwi Simulation Links](#-wokwi-simulation-links)
- [Database Schema](#-database-schema)
- [API Documentation](#-api-documentation)
- [Project Structure](#-project-structure)
- [Configuration](#-configuration)
- [Usage](#-usage)
- [Troubleshooting](#-troubleshooting)
- [Contributing](#-contributing)
- [Future Improvements](#-future-improvements)
- [License](#-license)
- [Author](#-author)

---

## 🌟 Project Overview

This project focuses on modernizing agriculture through IoT technology. It provides a centralized dashboard to monitor critical environmental factors such as temperature, humidity, soil moisture, pH levels, and light intensity. The system doesn't just monitor; it automates irrigation and environmental control to ensure optimal plant growth conditions.

### Why This Project?
- **Smart Agriculture**: Reduce water waste and optimize resource usage
- **Remote Monitoring**: Check your garden status from anywhere
- **Automated Care**: Let the system handle routine watering and ventilation
- **Data-Driven Decisions**: Historical data helps understand plant needs better

---

## 🏗 System Architecture

The system consists of three main layers:

### 1. Perception Layer (ESP32 Nodes)
Multiple ESP32 boards act as sensors and controllers, each handling specific tasks:
- Light sensing and control
- Soil moisture monitoring and irrigation
- Temperature/humidity tracking and ventilation
- pH level monitoring
- NPK (Nitrogen, Phosphorus, Potassium) sensing

### 2. Network Layer (MQTT Broker)
Uses HiveMQ as the communication bridge between hardware and backend, enabling:
- Real-time bidirectional communication
- Reliable message delivery
- Scalable device management

### 3. Application Layer (Node.js & Next.js)
- **Backend**: Node.js server handles MQTT logic, automation rules, and database storage
- **Frontend**: Modern, responsive Next.js dashboard for real-time data visualization and manual control

```
┌─────────────────────────────────────────────────────────┐
│                    Frontend (Next.js)                    │
│              Real-time Dashboard & Controls              │
└───────────────────────┬─────────────────────────────────┘
                        │ HTTP/WebSocket
┌───────────────────────┴─────────────────────────────────┐
│              Backend (Node.js + Express)                 │
│         Automation Logic │ Database │ API                │
└───────────────────────┬─────────────────────────────────┘
                        │ MQTT Protocol
┌───────────────────────┴─────────────────────────────────┐
│                  MQTT Broker (HiveMQ)                    │
└───────────────────────┬─────────────────────────────────┘
                        │
        ┌───────────────┼───────────────┬─────────────┐
        │               │               │             │
┌───────┴──────┐ ┌──────┴─────┐ ┌──────┴─────┐ ┌────┴─────┐
│   ESP32      │ │   ESP32    │ │   ESP32    │ │  ESP32   │
│ Light Board  │ │ Soil Board │ │ DHT Board  │ │ pH/NPK   │
└──────────────┘ └────────────┘ └────────────┘ └──────────┘
```

---

## ✨ Key Features

### Monitoring Capabilities
- 🌡️ **Temperature Tracking**: Real-time air temperature monitoring
- 💧 **Humidity Sensing**: Ambient humidity levels
- 🌱 **Soil Moisture**: Ground water content measurement
- ⚗️ **pH Monitoring**: Soil/water acidity levels
- ☀️ **Light Intensity**: Lux measurements for optimal lighting
- 🧪 **NPK Levels**: Essential nutrient monitoring (Nitrogen, Phosphorus, Potassium)

### Automation Features
- 💦 **Smart Irrigation**: Automatic pump activation based on soil moisture and pH
- 🌬️ **Climate Control**: Servo-controlled ventilation based on temperature/humidity
- 💡 **Lighting Control**: Automated grow lights based on ambient light levels
- ⚠️ **Alert System**: Visual notifications and database logging for abnormal readings

### User Control
- 🎮 **Manual Override**: Direct device control (pumps, lights, fans) from web interface
- 📊 **Data Visualization**: Charts and graphs for historical trends
- 📱 **Responsive Design**: Access from desktop, tablet, or mobile
- 🔧 **Multi-Board Management**: Distributed logic across specialized ESP32 nodes

---

## 🛠 Hardware Components (Simulated)

| Component | Function | Board Location |
| :--- | :--- | :--- |
| **ESP32** | Main microcontroller for all nodes | All Boards |
| **DHT22** | Air temperature and humidity sensor | Board 3 |
| **LDR Sensor** | Light intensity detection | Board 1 |
| **Soil Moisture Sensor** | Ground water level detection | Board 2 |
| **pH Sensor** | Soil/water acidity monitoring | Board 5 |
| **NPK Sensor** | N-P-K nutrient level detection | Board 4 |
| **Relay Module** | High-power device switching | Board 2 |
| **Water Pump** | Irrigation system actuator | Board 2 |
| **Servo Motor** | Greenhouse vent/shade control | Board 3 |
| **I2C LCD 16x2** | Local data display | Boards 2, 3, 5 |
| **LED Indicators** | Status indication | All Boards |

---

## 💻 Technology Stack

### Frontend
- **Framework**: Next.js 14 (React-based)
- **Styling**: Tailwind CSS
- **Animations**: Framer Motion
- **Charts**: Recharts / Chart.js
- **State Management**: React Context API / Zustand

### Backend
- **Runtime**: Node.js (v18+)
- **Framework**: Express.js
- **Real-time**: Socket.io / WebSockets
- **MQTT Client**: mqtt.js

### Communication
- **Protocol**: MQTT (Message Queuing Telemetry Transport)
- **Broker**: HiveMQ Cloud / Local Mosquitto
- **Topics Structure**: 
  - `garden/sensor/#` - Sensor data
  - `garden/control/#` - Device commands
  - `garden/status/#` - Device status

### Database
- **Primary**: MongoDB (NoSQL)
- **Collections**: sensor_readings, device_status, alerts, notifications
- **Export**: JSON format for data portability

### Simulation
- **Platform**: [Wokwi](https://wokwi.com/)
- **Language**: C++ (Arduino Framework)
- **IDE**: Arduino IDE / PlatformIO

---

## 🚀 Installation and Setup

### Prerequisites
- Node.js v18 or higher ([Download](https://nodejs.org/))
- MongoDB installed locally or MongoDB Atlas account
- MQTT Broker (HiveMQ Cloud or local Mosquitto)
- Git for version control
- (Optional) MQTT Explorer for testing

### 1. Clone the Repository
```bash
git clone https://github.com/yourusername/iot-smartgarden-esp32-nodejs-wokwi.git
cd iot-smartgarden-esp32-nodejs-wokwi
```

### 2. Backend Setup
```bash
cd backend
npm install

# Create .env file
cp .env.example .env

# Edit .env with your configuration
nano .env
```

**Required Environment Variables:**
```env
# MQTT Configuration
MQTT_BROKER=broker.hivemq.com
MQTT_PORT=1883
MQTT_USERNAME=your_username
MQTT_PASSWORD=your_password

# Database
MONGODB_URI=mongodb://localhost:27017/smartgarden
# Or use MongoDB Atlas:
# MONGODB_URI=mongodb+srv://username:password@cluster.mongodb.net/smartgarden

# Server
PORT=3001
NODE_ENV=development

# Automation Thresholds
SOIL_MOISTURE_MIN=30
SOIL_MOISTURE_MAX=70
TEMP_MAX=35
HUMIDITY_MIN=40
```

**Start the Backend Server:**
```bash
node broker/server.js
# Or use nodemon for development:
npm run dev
```

### 3. Frontend Setup
```bash
cd frontend/web
npm install

# Create .env.local file
cp .env.local.example .env.local

# Edit with your backend URL
nano .env.local
```

**Frontend Environment Variables:**
```env
NEXT_PUBLIC_API_URL=http://localhost:3001
NEXT_PUBLIC_MQTT_WS_URL=ws://localhost:8080
```

**Start the Development Server:**
```bash
npm run dev
```

The frontend will be available at `http://localhost:3000`

### 4. ESP32 Code Configuration

Each board has its own Arduino sketch in the `frontend/board-X` directories.

**Steps for Each Board:**
1. Open the `.ino` file in Arduino IDE or Wokwi
2. Update WiFi credentials:
```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
```

3. Update MQTT broker settings:
```cpp
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_user = "your_username";
const char* mqtt_password = "your_password";
```

4. Verify the MQTT topics match your backend configuration
5. Upload to ESP32 or run in Wokwi simulator

### 5. Database Initialization

The database will be automatically initialized on first run. To manually seed data:

```bash
cd backend
node scripts/seed-database.js
```

---

## 🔗 Wokwi Simulation Links

Access the complete simulations for each board module. Each link opens a fully configured Wokwi project:

### Board Modules
- **Board 1**: [Light Sensor & Control System](https://wokwi.com/projects/your-project-id-1)
  - LDR sensor for ambient light detection
  - Relay-controlled grow lights
  - Automatic lighting based on time/lux levels

- **Board 2**: [Soil Moisture & Irrigation System](https://wokwi.com/projects/your-project-id-2)
  - Capacitive soil moisture sensor
  - Water pump with relay control
  - LCD display for moisture readings

- **Board 3**: [Temperature/Humidity & Ventilation](https://wokwi.com/projects/your-project-id-3)
  - DHT22 sensor for climate monitoring
  - Servo motor for vent control
  - Automatic ventilation triggers

- **Board 4**: [NPK Nutrient Monitoring](https://wokwi.com/projects/your-project-id-4)
  - Custom NPK sensor simulation
  - Real-time nutrient level reporting
  - Alert generation for deficiencies

- **Board 5**: [pH Level Monitoring](https://wokwi.com/projects/your-project-id-5)
  - pH sensor with analog reading
  - LCD display for pH values
  - Water quality alerts

**Note**: Update the `wokwi-project-link.txt` file with your actual project URLs after creating them.

---

## 🗄 Database Schema

### Collections Overview

#### 1. `sensor_readings`
Stores all sensor data with timestamps.

```javascript
{
  _id: ObjectId,
  boardId: String,        // "board-1", "board-2", etc.
  sensorType: String,     // "temperature", "humidity", "soil_moisture", etc.
  value: Number,
  unit: String,           // "°C", "%", "pH", "lux"
  timestamp: Date,
  location: String        // Optional: "greenhouse-1"
}
```

#### 2. `device_status`
Current state of all controllable devices.

```javascript
{
  _id: ObjectId,
  deviceId: String,       // "pump-1", "fan-1", "light-1"
  deviceType: String,     // "pump", "fan", "light", "servo"
  status: String,         // "on", "off", "auto"
  lastToggled: Date,
  triggeredBy: String,    // "manual", "automation", "schedule"
  boardId: String
}
```

#### 3. `alerts`
History of threshold breaches and warnings.

```javascript
{
  _id: ObjectId,
  alertType: String,      // "high_temp", "low_moisture", "ph_warning"
  severity: String,       // "info", "warning", "critical"
  message: String,
  sensorValue: Number,
  threshold: Number,
  boardId: String,
  resolved: Boolean,
  timestamp: Date,
  resolvedAt: Date
}
```

#### 4. `notifications`
User-facing system messages.

```javascript
{
  _id: ObjectId,
  title: String,
  message: String,
  type: String,           // "success", "info", "warning", "error"
  read: Boolean,
  createdAt: Date,
  relatedAlert: ObjectId  // Reference to alerts collection
}
```

#### 5. `automation_rules`
User-defined automation configurations.

```javascript
{
  _id: ObjectId,
  ruleName: String,
  condition: {
    sensorType: String,
    operator: String,     // "greater_than", "less_than", "equals"
    value: Number
  },
  action: {
    deviceId: String,
    command: String       // "turn_on", "turn_off", "set_value"
  },
  enabled: Boolean,
  createdAt: Date
}
```

---

## 📡 API Documentation

### REST Endpoints

#### Sensor Data
```
GET    /api/sensors/latest          - Get latest readings from all sensors
GET    /api/sensors/:type/history   - Get historical data for a sensor type
POST   /api/sensors/reading         - Submit new sensor reading (internal)
```

#### Device Control
```
GET    /api/devices                 - List all devices and their status
POST   /api/devices/:id/toggle      - Toggle device on/off
POST   /api/devices/:id/auto        - Enable/disable auto mode
GET    /api/devices/:id/history     - Get device operation history
```

#### Alerts & Notifications
```
GET    /api/alerts                  - Get all alerts
GET    /api/alerts/active           - Get unresolved alerts
POST   /api/alerts/:id/resolve      - Mark alert as resolved
GET    /api/notifications           - Get all notifications
POST   /api/notifications/:id/read  - Mark notification as read
```

#### Automation Rules
```
GET    /api/rules                   - List all automation rules
POST   /api/rules                   - Create new automation rule
PUT    /api/rules/:id               - Update existing rule
DELETE /api/rules/:id               - Delete rule
```

### MQTT Topics

#### Publishing (Sensors → Backend)
```
garden/sensor/temperature     - Temperature readings (°C)
garden/sensor/humidity        - Humidity readings (%)
garden/sensor/soil_moisture   - Soil moisture (%)
garden/sensor/ph              - pH levels (0-14)
garden/sensor/light           - Light intensity (lux)
garden/sensor/npk/nitrogen    - Nitrogen level
garden/sensor/npk/phosphorus  - Phosphorus level
garden/sensor/npk/potassium   - Potassium level
```

#### Subscribing (Backend → Devices)
```
garden/control/pump           - Pump commands (on/off)
garden/control/fan            - Fan commands (on/off)
garden/control/light          - Light commands (on/off)
garden/control/servo          - Servo position (0-180)
garden/status/request         - Request status update
```

---

## 📁 Project Structure

```
iot-smartgarden-esp32-nodejs-wokwi/
├── backend/
│   ├── broker/
│   │   ├── server.js              # Main server entry point
│   │   ├── automation.js          # Automation logic
│   │   └── mqttClient.js          # MQTT connection handler
│   ├── models/
│   │   ├── SensorReading.js       # Mongoose schema
│   │   ├── DeviceStatus.js
│   │   ├── Alert.js
│   │   └── Notification.js
│   ├── routes/
│   │   ├── sensors.js             # Sensor API routes
│   │   ├── devices.js             # Device control routes
│   │   └── alerts.js              # Alert management routes
│   ├── controllers/
│   │   ├── sensorController.js
│   │   ├── deviceController.js
│   │   └── alertController.js
│   ├── middleware/
│   │   ├── auth.js                # Authentication middleware
│   │   └── errorHandler.js
│   ├── utils/
│   │   ├── logger.js              # Winston logger config
│   │   └── validators.js
│   ├── scripts/
│   │   └── seed-database.js       # Database seeding
│   ├── package.json
│   └── .env.example
│
├── frontend/
│   ├── web/
│   │   ├── app/
│   │   │   ├── page.js            # Main dashboard
│   │   │   ├── layout.js
│   │   │   └── globals.css
│   │   ├── components/
│   │   │   ├── Dashboard/
│   │   │   │   ├── SensorCard.jsx
│   │   │   │   ├── DeviceControl.jsx
│   │   │   │   └── AlertPanel.jsx
│   │   │   ├── Charts/
│   │   │   │   ├── LineChart.jsx
│   │   │   │   └── GaugeChart.jsx
│   │   │   └── Layout/
│   │   │       ├── Header.jsx
│   │   │       └── Sidebar.jsx
│   │   ├── lib/
│   │   │   ├── mqtt.js            # MQTT client for browser
│   │   │   └── api.js             # API helper functions
│   │   ├── hooks/
│   │   │   ├── useSensorData.js
│   │   │   └── useDeviceControl.js
│   │   ├── public/
│   │   ├── package.json
│   │   └── next.config.js
│   │
│   ├── board-1/                   # Light control board
│   │   ├── board-1.ino
│   │   ├── diagram.json           # Wokwi circuit
│   │   └── wokwi.toml
│   ├── board-2/                   # Soil & irrigation board
│   │   ├── board-2.ino
│   │   ├── diagram.json
│   │   └── wokwi.toml
│   ├── board-3/                   # Climate control board
│   │   ├── board-3.ino
│   │   ├── diagram.json
│   │   └── wokwi.toml
│   ├── board-4/                   # NPK monitoring board
│   │   ├── board-4.ino
│   │   ├── diagram.json
│   │   └── wokwi.toml
│   └── board-5/                   # pH monitoring board
│       ├── board-5.ino
│       ├── diagram.json
│       └── wokwi.toml
│
├── docs/
│   ├── API.md                     # Detailed API documentation
│   ├── HARDWARE.md                # Hardware setup guide
│   ├── DEPLOYMENT.md              # Production deployment guide
│   └── TROUBLESHOOTING.md         # Common issues and solutions
│
├── .gitignore
├── LICENSE
├── README.md
├── wokwi-project-link.txt         # Links to all Wokwi simulations
└── package.json                   # Root package.json for scripts
```

---

## ⚙️ Configuration

### Automation Thresholds

Customize automation behavior in `backend/broker/automation.js`:

```javascript
const THRESHOLDS = {
  soilMoisture: {
    min: 30,        // Start irrigation below 30%
    max: 70,        // Stop irrigation above 70%
    critical: 20    // Critical alert below 20%
  },
  temperature: {
    max: 35,        // Open vents above 35°C
    min: 15,        // Close vents below 15°C
    critical: 40    // Critical alert above 40°C
  },
  humidity: {
    min: 40,        // Alert if humidity below 40%
    max: 85         // Alert if humidity above 85%
  },
  ph: {
    min: 6.0,       // Ideal pH minimum
    max: 7.5,       // Ideal pH maximum
    critical_low: 5.0,
    critical_high: 8.5
  },
  light: {
    min: 200,       // Turn on grow lights below 200 lux
    max: 50000      // Maximum safe light intensity
  }
};
```

### MQTT Topic Configuration

Modify topic structure in `backend/broker/mqttClient.js` if needed:

```javascript
const TOPICS = {
  sensor: {
    temperature: 'garden/sensor/temperature',
    humidity: 'garden/sensor/humidity',
    soilMoisture: 'garden/sensor/soil_moisture',
    // ... add more topics
  },
  control: {
    pump: 'garden/control/pump',
    fan: 'garden/control/fan',
    // ... add more topics
  }
};
```

---

## 🎯 Usage

### Starting the System

1. **Start MongoDB** (if running locally):
```bash
mongod --dbpath /path/to/data/db
```

2. **Start Backend Server**:
```bash
cd backend
npm start
```

3. **Start Frontend**:
```bash
cd frontend/web
npm run dev
```

4. **Start ESP32 Simulations**:
   - Open Wokwi links for each board
   - Click "Start Simulation" on each board
   - Verify MQTT connections in backend logs

### Using the Dashboard

#### Monitoring
- View real-time sensor data on the main dashboard
- Check historical trends using the chart views
- Monitor device status and last activity times

#### Manual Control
- Toggle devices using the control panel switches
- Switch between "Auto" and "Manual" modes
- View automation rules and their current status

#### Alerts
- Check the alert panel for any warnings
- Click on alerts to see detailed information
- Mark alerts as resolved after addressing issues

### Testing Automation

1. **Simulate Dry Soil**:
   - In Wokwi Board 2, adjust soil moisture sensor to low value
   - Watch backend logs for automation trigger
   - Verify pump turns on automatically
   - Check dashboard for status update

2. **Simulate High Temperature**:
   - In Wokwi Board 3, increase DHT22 temperature reading
   - Observe servo motor opening vents
   - Verify fan activation if threshold exceeded

3. **Test Manual Override**:
   - Toggle pump manually from dashboard
   - Verify ESP32 receives command
   - Check that automation respects manual mode

---

## 🔧 Troubleshooting

### Common Issues

#### MQTT Connection Failed
```
Error: Connection refused: Not authorized
```
**Solution**: Check MQTT credentials in `.env` file and ESP32 code

#### Database Connection Error
```
MongoNetworkError: failed to connect to server
```
**Solution**: 
- Verify MongoDB is running: `sudo systemctl status mongod`
- Check connection string in `.env`
- Ensure firewall allows MongoDB port (27017)

#### ESP32 WiFi Connection Issues
**Solution**:
- Verify WiFi credentials in `.ino` files
- Check that SSID is 2.4GHz (ESP32 doesn't support 5GHz)
- Ensure router allows new device connections

#### Sensor Reading "NaN" or Invalid
**Solution**:
- Check sensor wiring in Wokwi diagram
- Verify sensor library versions match
- Add delays between sensor readings

#### Frontend API Calls Failing
**Solution**:
- Check `NEXT_PUBLIC_API_URL` in `.env.local`
- Verify backend server is running
- Check browser console for CORS errors
- Ensure backend has CORS enabled for frontend URL

### Debug Mode

Enable verbose logging in backend:
```javascript
// In broker/server.js
const DEBUG = true;

if (DEBUG) {
  console.log('MQTT Message:', topic, message);
}
```

Enable MQTT debug in ESP32:
```cpp
// In board-X.ino
#define DEBUG true

#if DEBUG
  Serial.println("Debug: MQTT connected");
#endif
```

### Getting Help

1. Check `docs/TROUBLESHOOTING.md` for detailed solutions
2. Review GitHub Issues for similar problems
3. Join our Discord community (link in repository)
4. Contact: your.email@example.com

---

## 🤝 Contributing

We welcome contributions! Here's how you can help:

### Reporting Bugs
1. Check existing issues first
2. Use the bug report template
3. Include system info, logs, and steps to reproduce

### Suggesting Features
1. Open a feature request issue
2. Describe the use case and benefits
3. Include mockups or diagrams if applicable

### Pull Requests
1. Fork the repository
2. Create a feature branch: `git checkout -b feature/amazing-feature`
3. Commit changes: `git commit -m 'Add amazing feature'`
4. Push to branch: `git push origin feature/amazing-feature`
5. Open a Pull Request

### Development Guidelines
- Follow existing code style
- Add comments for complex logic
- Update documentation for new features
- Test thoroughly before submitting
- Ensure all boards work in Wokwi simulation

---

## 🔮 Future Improvements

### Planned Features
- [ ] **Cloud Integration**: Google Cloud IoT Core / AWS IoT
- [ ] **Machine Learning**: Predictive analytics for plant health
- [ ] **Mobile App**: React Native cross-platform application
- [ ] **Voice Control**: Alexa/Google Assistant integration
- [ ] **Weather API**: Integration for predictive watering
- [ ] **Camera Integration**: ESP32-CAM for plant monitoring
- [ ] **Solar Monitoring**: Track solar panel performance
- [ ] **Multi-Garden Support**: Manage multiple garden locations
- [ ] **User Authentication**: Multi-user access with roles
- [ ] **Data Export**: CSV/PDF report generation

### Research Areas
- [ ] Computer vision for pest detection
- [ ] AI-based optimal watering schedules
- [ ] Energy consumption optimization
- [ ] Hydroponics system support
- [ ] Integration with agricultural databases

### Community Requests
Vote on features in our [GitHub Discussions](https://github.com/yourusername/repo/discussions)

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

### MIT License Summary
- ✅ Commercial use
- ✅ Modification
- ✅ Distribution
- ✅ Private use
- ⚠️ Liability and warranty disclaimers apply

---

## 👨‍💻 Author

**Huỳnh Quốc Huy**

- GitHub: [@yourusername](https://github.com/yourusername)
- Email: your.email@example.com
- LinkedIn: [Your Name](https://linkedin.com/in/yourprofile)
- Portfolio: [yourwebsite.com](https://yourwebsite.com)

### Acknowledgments
- Thanks to the Wokwi team for the excellent simulation platform
- HiveMQ for providing free MQTT broker services
- The open-source community for libraries and inspiration
- [Add any other acknowledgments]

---

## 📊 Project Stats

![GitHub stars](https://img.shields.io/github/stars/yourusername/repo?style=social)
![GitHub forks](https://img.shields.io/github/forks/yourusername/repo?style=social)
![GitHub issues](https://img.shields.io/github/issues/yourusername/repo)
![GitHub pull requests](https://img.shields.io/github/issues-pr/yourusername/repo)

---

## 🌐 Links

- **Repository**: [github.com/yourusername/iot-smartgarden-esp32-nodejs-wokwi](https://github.com/yourusername/iot-smartgarden-esp32-nodejs-wokwi)
- **Documentation**: [docs.yourproject.com](https://docs.yourproject.com)
- **Live Demo**: [demo.yourproject.com](https://demo.yourproject.com)
- **Wokwi Projects**: See `wokwi-project-link.txt`

---

## 📞 Support

If you find this project helpful, please consider:
- ⭐ Starring the repository
- 🐛 Reporting bugs
- 💡 Suggesting new features
- 📖 Improving documentation
- 💰 [Sponsoring the project](https://github.com/sponsors/yourusername)

---

**Made with ❤️ for sustainable agriculture and IoT education**

*Last updated: December 2024*
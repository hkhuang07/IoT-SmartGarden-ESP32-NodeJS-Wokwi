# 📁 Smart Garden IoT Project Structure

```
smart-garden-iot-system/
├── 📁 frontend/                          # ESP32 Code & Web Interface
│   ├── 📁 board1-light-sensor-control/   # Board 1: Light Control (C++)
│   │   ├── main.ino                      # Main Arduino code
│   │   ├── diagram.json                  # Wokwi simulation diagram
│   │   └── README.md                     # Board documentation
│   │
│   ├── 📁 board2-soil-sensor-control/    # Board 2: Soil Moisture (C++)
│   │   ├── main.ino                      # Main Arduino code  
│   │   ├── diagram.json                  # Wokwi simulation diagram
│   │   └── README.md                     # Board documentation
│   │
│   ├── 📁 board3-temp-m2m-control/       # Board 3: Temp + M2M (C++ + MicroPython)
│   │   ├── main.ino                      # C++ code for sensors
│   │   ├── diagram.json                  # Wokwi simulation diagram
│   │   └── 📁 micropython/
│   │       ├── main.py                   # MicroPython control code
│   │       ├── i2c_lcd.py                # I2C LCD library
│   │       ├── servo.py                  # Servo control library
│   │       └── README.md                 # MicroPython documentation
│   │
│   ├── 📁 board4-npk-sensor-control/     # Board 4: NPK Control (C++)
│   │   ├── main.ino                      # Main Arduino code
│   │   ├── diagram.json                  # Wokwi simulation diagram
│   │   ├── soil-moisture.chip.json       # Custom NPK sensor chip
│   │   ├── soil-moisture.chip.c          # NPK sensor C implementation
│   │   └── README.md                     # Board documentation
│   │
│   ├── 📁 board5-pump-pwm-control/       # Board 5: Pump PWM Control (C++)
│   │   ├── main.ino                      # Main Arduino code
│   │   ├── diagram.json                  # Wokwi simulation diagram
│   │   └── README.md                     # Board documentation
│   │
│   ├── 📁 board6-env-monitor/            # Board 6: Environmental Monitor (C++)
│   │   ├── main.ino                      # Main Arduino code
│   │   ├── diagram.json                  # Wokwi simulation diagram
│   │   └── README.md                     # Board documentation
│   │
│   └── 📁 web/                           # NextJS Web Application
│       ├── 📁 pages/
│       │   ├── index.tsx                 # Main dashboard page
│       │   ├── _app.tsx                  # App wrapper
│       │   └── _document.tsx             # Document setup
│       ├── 📁 components/                # Reusable components
│       ├── 📁 styles/                    # CSS/Tailwind styles
│       ├── package.json                  # Dependencies
│       ├── next.config.js                # NextJS configuration
│       └── .env.local                    # Environment variables
│
├── 📁 backend/                           # NodeJS Backend Services
│   ├── 📁 broker/                        # MQTT Broker Service
│   │   ├── server.js                     # Main broker server
│   │   ├── config.js                     # System configuration
│   │   ├── db_connection.js              # MongoDB connection
│   │   └── automation.js                 # Automation logic
│   │
│   ├── 📁 api/                           # REST API Service
│   │   └── server.js                     # Express API server
│   │
│   ├── package.json                      # Backend dependencies
│   ├── .env.example                      # Environment template
│   └── 📁 logs/                          # Log files (generated)
│
├── 📁 docs/                              # Documentation
│   ├── API_DOCUMENTATION.md              # API reference
│   ├── MQTT_TOPICS.md                    # MQTT topic structure
│   ├── HARDWARE_SETUP.md                 # Hardware assembly guide
│   └── TROUBLESHOOTING.md                # Common issues & solutions
│
├── 📁 tests/                             # Test files
│   ├── 📁 unit/                          # Unit tests
│   ├── 📁 integration/                   # Integration tests
│   └── 📁 e2e/                          # End-to-end tests
│
├── 📁 deployment/                        # Deployment configurations
│   ├── 📁 docker/                        # Docker configurations
│   ├── 📁 kubernetes/                    # K8s manifests
│   └── 📁 cloud/                         # Cloud deployment scripts
│
├── .gitignore                            # Git ignore rules
├── .env.example                          # Environment template
├── LICENSE                               # MIT License
├── README.md                             # Main project documentation
└── PROJECT_STRUCTURE.md                  # This file
```

## 🗂️ Chi Tiết Từng Thành Phần

### 📱 Frontend ESP32 Boards

| Board | Chức Năng | Công Nghệ | Code Files |
|-------|-----------|-----------|------------|
| **Board 1** | Light Control | C++, PWM, I2C, SPI | `main.ino`, `diagram.json` |
| **Board 2** | Soil Moisture | C++, Digital Analog, Servo | `main.ino`, `diagram.json` |
| **Board 3** | Temperature + M2M | C++ + MicroPython, I2C | `main.ino`, `micropython/main.py` |
| **Board 4** | NPK Control | C++, Digital Analog, LCD | `main.ino`, `diagram.json` |
| **Board 5** | Pump PWM | C++, WatchDog, PWM | `main.ino`, `diagram.json` |
| **Board 6** | Env Monitor | C++, I2C/SPI Hub | `main.ino`, `diagram.json` |

### 🖥️ Backend Services

| Service | Port | Chức Năng | File |
|---------|------|-----------|------|
| **MQTT Broker** | - | Xử lý MQTT, Automation | `backend/broker/server.js` |
| **REST API** | 3001 | Web/Mobile API | `backend/api/server.js` |
| **Database** | - | MongoDB Atlas | `db_connection.js` |

### 🌐 Web Application

| Page | URL | Chức Năng |
|------|-----|-----------|
| **Dashboard** | `/` | Real-time monitoring, Control panel |
| **Analytics** | `/analytics` | Data analysis, Charts |
| **Alerts** | `/alerts` | Alert management |
| **Settings** | `/settings` | Configuration |

## 🔧 File Quan Trọng

### Configuration Files
- `backend/.env.example` - Environment variables template
- `frontend/web/next.config.js` - NextJS configuration
- `frontend/web/package.json` - Web dependencies
- `backend/package.json` - Backend dependencies

### Documentation
- `README.md` - Main project documentation
- `PROJECT_STRUCTURE.md` - This file
- `docs/` - Detailed documentation

### Simulation Files
- `frontend/*/diagram.json` - Wokwi simulation diagrams
- `frontend/board4-npk-sensor-control/soil-moisture.chip.*` - Custom sensor chips

## 🚀 Quick Start Commands

```bash
# Start Backend Services
cd backend
npm install
npm run start:all

# Start Web Application
cd ../frontend/web
npm install
npm run dev

# Run ESP32 Simulation
# 1. Open Wokwi.com
# 2. Upload diagram.json from any board
# 3. Upload corresponding .ino or .py file
# 4. Start simulation
```

## 📊 Data Flow

```
ESP32 Boards → MQTT Topics → Broker Server → MongoDB → API Server → Web/Mobile
     ↓               ↓            ↓            ↓          ↓           ↓
  Sensors      broker.hivemq  Automation   Data      REST API   Dashboard
  Actuators    (1883)        Logic        Storage    (3001)     Real-time
```

## 🔗 External Dependencies

| Service | URL | Purpose |
|---------|-----|---------|
| **HiveMQ Broker** | broker.hivemq.com:1883 | Public MQTT broker |
| **MongoDB Atlas** | cloud.mongodb.com | Database hosting |
| **Wokwi Simulator** | wokwi.com | ESP32 simulation |
| **GitHub Repository** | - | Code repository |

## 🎯 Next Steps

1. **Setup MongoDB Atlas** - Create cluster and update connection string
2. **Configure Environment** - Copy `.env.example` to `.env` and update values
3. **Start Backend** - Run MQTT broker and API server
4. **Launch Web App** - Start NextJS development server
5. **Run Simulations** - Upload board diagrams to Wokwi and start coding
6. **Test System** - Monitor MQTT messages and test web interface

---

**📁 Happy Coding! Build your smart garden with IoT innovation! 🌱**
# 🌿 IoT Smart Garden & PHP E-Commerce Platform

This repository contains the integrated final project, combining an **IoT Smart Garden** system built on ESP32/Wokwi with a **PHP Web Technology** platform for e-commerce and real-time data visualization.

---

## 🧭 Project Architecture Overview

The system follows a typical **IoT End-to-End Architecture**, where the Node.js server acts as the central hub for data persistence and command routing.

### Key Components

| Component | Technology | Primary Role | Project Focus |
| :--- | :--- | :--- | :--- |
| **IoT Clients (Nodes)** | ESP32 (Wokwi/Arduino Code) | Collects sensor data (Temp, Humidity, Soil Moisture) and acts on commands (Water Valve, Servo). | IoT Graduation Project |
| **Broker/Data Handler** | **Node.js (Aedes MQTT Broker) & MongoDB Atlas** | Handles MQTT communication (Topic `smartgarden/sensor/#`), processes JSON payloads, and persists data to the NoSQL database. | IoT/Backend Integration |
| **Data Visualization & E-commerce** | **PHP (MySQL) & MongoDB PHP Driver** | Manages sales inventory and transactions (MySQL) and displays real-time and historical sensor data (MongoDB) on a web dashboard. | PHP Web Technology Project |
| **Tunneling** | **Cloudflare Tunnel / Ngrok / LocalTunnel** | Provides a public TCP endpoint (e.g., `mqtt-broker.hkhuang07.me:8883`) to connect Wokwi (cloud simulator) with the local Node.js Broker. | Deployment Solution |

---

## 💻 Backend: Node.js MQTT Broker & MongoDB Handler

The core of the data pipeline is managed by the Node.js application, using the `aedes` library to act as a self-hosted MQTT broker.

### Topics Handled

| Topic Prefix | Role | Action |
| :--- | :--- | :--- |
| `smartgarden/sensor/#` | Data Publishing | **Data Handler** subscribes, parses JSON payload, and inserts the full document (with timestamp, deviceId) into the **`sensor_readings`** collection in MongoDB Atlas. |
| `smartgarden/control/#` | Command Subscription | Broker routes commands (e.g., `ON`/`OFF`) from the PHP Web App or Mobile App directly to the relevant ESP32 device. |

### Data Persistance Logic (`server.js`)

The `server.js` file handles incoming sensor data and saves it to MongoDB:

```javascript
// Example Data Handling Logic from server.js
aedes.on("publish", async function (packet, client) {
    if (client && packet.topic.startsWith("smartgarden/sensor/")) {
        try {
            const data = JSON.parse(packet.payload.toString());
            const doc = {
                topic: packet.topic,
                deviceId: client.id || "unknown",
                timestamp: new Date(),
                data: data, // Contains keys like { "t": 28.5, "m": 450 }
            };
            await db.collection("sensor_readings").insertOne(doc);
            console.log(`[DB] Data saved from', ${doc.deviceId}`);
        } catch (error) {
            console.error(`[HANDLER ERROR] Processing error: ${error.message}`);
        }
    }
});
```

### Setup Instructions (Node.js)

1.  Navigate to the **`iot-nodejs-server`** directory.
2.  Install dependencies: `npm install aedes mongodb`
3.  Ensure `db_connection.js` is correctly configured with your **MongoDB Atlas URI**.
4.  Run the Broker: `npm start` (or `node server.js`)

---

## 🤖 Frontend: IoT Devices (ESP32/Wokwi)

The project utilizes modular ESP32 clients (Nodes) for a distributed sensor network, coded in the Arduino framework or MicroPython within Wokwi.

### Implemented Sensor Nodes

| Node/Module | Files Involved | Function |
| :--- | :--- | :--- |
| **Temperature/Humidity** | `temperature_humidity_sensor/sketch.ino` | Reads **DHT22** data and publishes to `smartgarden/sensor/temp_humidity`. |
| **Soil Moisture (Custom)** | `soil_moisture/main.py`, `.chip.c`, `.chip.json` | Reads simulated **ADC value** (from Wokwi slider) and publishes to `smartgarden/sensor/soil_moisture`. |
| **Actuator/Logic** | `actuator_node/sketch.ino` (Planned) | **Subscribes** to commands (`smartgarden/control/#`) to turn on the water pump, LED lights, and manage auto-irrigation logic. |

---

## 📊 PHP Web Platform (Planned)

The PHP component is designed to leverage both databases for a comprehensive experience.

* **MySQL Database (`php_sales`):** Stores structured data for **e-commerce** (products, users, orders).
* **MongoDB Integration:** Uses the **MongoDB PHP Driver** to connect directly to Atlas, retrieve time-series sensor data, and display it via **Chart.js** on the admin dashboard.

---

## 🛠️ Deployment Status

| Phase | Status | Notes |
| :--- | :--- | :--- |
| **Phase 1: IoT Backend** | **Completed** | Node.js Broker is running, MongoDB is connected, and Data Handler is functional. |
| **Phase 1: IoT Clients** | **In Progress** | DHT22 and Soil Moisture Sensor nodes are coded and ready for connection test. |
| **Phase 1: Tunneling** | **Pending Activation** | Nameservers for **`hkhuang07.me`** are being updated on Cloudflare. The connection will use Cloudflare Tunnel once active. |
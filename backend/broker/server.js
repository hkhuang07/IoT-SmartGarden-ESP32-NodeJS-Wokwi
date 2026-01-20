const mqtt = require('mqtt');
const { connectToDatabase } = require("./db_connect");
const SmartGardenAutomation = require('./automation');
const automationLogic = new SmartGardenAutomation();

//const PUBLIC_BROKER = 'mqtt://broker.hivemq.com';
const PUBLIC_BROKER = 'mqtt://52.58.31.240';

let db;
let mqttClient;

// Topic structure:
// garden/sensor/* - All sensor data (including pH)
// garden/control/* - All control commands  
// garden/status/* - Device status
// garden/system/* - System messages
// garden/m2m/* - Machine-to-machine communication
// garden/alerts/* - Alert notifications

class SmartGardenBroker {
    constructor() {
        this.devices = new Map();
        this.commandHistory = [];
        this.alertCache = new Map();
        this.lastHeartbeat = new Map();
        this.phMonitoring = new Map(); // pH monitoring data
        this.sensorMonitoring = new Map(); // General sensor monitoring data
    }

    async start() {
        try {
            console.log('🚀 Starting Smart Garden IoT System...');
            
            // Connect to MongoDB
            db = await connectToDatabase();
            console.log('[DB] Connected to MongoDB Atlas');

            // Connect to MQTT Broker
            this.connectMQTT();
            
            // Start health monitoring
            this.startHealthMonitoring();
            
            console.log('✅ Smart Garden Broker is running!');
            console.log('==================================');
            
        } catch (error) {
            console.error('[BROKER ERROR] Failed to start:', error);
            process.exit(1);
        }
    }

    connectMQTT() {
        console.log(`[MQTT] Connecting to: ${PUBLIC_BROKER}`);
        
        mqttClient = mqtt.connect(PUBLIC_BROKER, {
            clientId: 'SmartGarden-Broker-' + Date.now(),
            clean: true,
            reconnectPeriod: 3000,
            keepalive: 60,
            connectTimeout: 10000
        });

        mqttClient.on('connect', () => {
            console.log(`[MQTT] ✅ Connected to broker: ${PUBLIC_BROKER}`);
            this.subscribeToTopics();
            this.publishSystemStatus('ONLINE');
        });

        mqttClient.on('error', (err) => {
            console.error(`[MQTT ERROR] Connection failed:`, err.message);
        });

        mqttClient.on('close', () => {
            console.log('[MQTT] Connection closed');
        });

        mqttClient.on('reconnect', () => {
            console.log('[MQTT] Reconnecting...');
        });

        mqttClient.on('message', (topic, payload) => {
            this.handleMessage(topic, payload);
        });
    }

    subscribeToTopics() {
        const topics = [
            'garden/sensor/#',           // All sensor data
            'garden/control/#',          // All control commands
            'garden/status/#',           // All device status
            'garden/system/device_info', // Device information
            'garden/m2m/#',              // M2M communication
            'garden/alerts/#',           // Alert notifications
            // Specific sensor topics
            'garden/sensor/ph',          // pH sensor data
            'garden/sensor/ph_data',     // Extended pH data format
            'garden/sensor/light',       // Light sensor data
            'garden/sensor/light_data',  // Extended light data
            'garden/sensor/temp_hum',    // Temperature/humidity sensor data
            'garden/sensor/temp_hum_data', // Extended temp/hum data
            'garden/sensor/soil_moisture', // Soil moisture sensor data
            'garden/sensor/soil_moisture_data', // Extended soil moisture data
            'garden/sensor/npk',         // NPK sensor data
            'garden/sensor/npk_data'     // Extended NPK data
        ];

        topics.forEach(topic => {
            mqttClient.subscribe(topic, (err) => {
                if (err) {
                    console.error(`[MQTT] Subscribe failed for ${topic}:`, err);
                } else {
                    console.log(`[MQTT] ✅ Subscribed to: ${topic}`);
                }
            });
        });
        console.log('==================================');
    }

    async handleMessage(topic, payload) {
        const payloadString = payload.toString();
        const topicParts = topic.split('/');
        
        try {
            console.log(`[MQTT] 📨 Topic: ${topic} | Payload: ${payloadString}`);

            // Update device last seen
            this.updateDeviceActivity(topic, payloadString);

            if (topic.startsWith('garden/sensor/')) {
                await this.handleSensorData(topic, payloadString);
            }
            else if (topic.startsWith('garden/control/')) {
                await this.handleControlCommand(topic, payloadString);
            }
            else if (topic.startsWith('garden/status/')) {
                await this.handleDeviceStatus(topic, payloadString);
            }
            else if (topic.startsWith('garden/system/')) {
                await this.handleSystemMessage(topic, payloadString);
            }
            else if (topic.startsWith('garden/m2m/')) {
                await this.handleM2MMessage(topic, payloadString);
            }
            else if (topic.startsWith('garden/alerts/')) {
                await this.handleAlertMessage(topic, payloadString);
            }

        } catch (error) {
            console.error(`[MQTT ERROR] Processing failed:`, error.message);
        }
    }

    async handleSensorData(topic, payloadString) {
        const topicParts = topic.split('/');
        const sensorType = topicParts[2];
        const timestamp = new Date();

        try {
            const data = JSON.parse(payloadString);
            
            // Create sensor document
            const sensorDoc = {
                topic: topic,
                sensor_type: sensorType,
                timestamp: timestamp,
                data: data,
                device_id: data.device_id || 'unknown',
                processed: false
            };

            // Save to database
            await db.collection("sensor_readings").insertOne(sensorDoc);
            console.log(`[DB] 📊 Saved sensor data: ${sensorType}`);

            // Special handling for specific sensor types
            switch (sensorType) {
                case 'ph':
                case 'ph_level':
                case 'ph_data':
                    await this.handlePhSensorData(data, timestamp);
                    break;
                case 'light':
                case 'light_data':
                    await this.handleLightSensorData(data, timestamp);
                    break;
                case 'temp_hum':
                case 'temp_hum_data':
                    await this.handleTempHumSensorData(data, timestamp);
                    break;
                case 'soil_moisture':
                case 'soil_moisture_data':
                    await this.handleSoilMoistureSensorData(data, timestamp);
                    break;
                case 'npk':
                case 'npk_data':
                    await this.handleNpkSensorData(data, timestamp);
                    break;
            }

            // Run automation logic
            await automationLogic.processSensorData(
                data.device_id || 'unknown', 
                sensorType, 
                data, 
                timestamp.getTime(),
                mqttClient,
                this
            );

            // Update device status
            await this.updateDeviceStatus(sensorType, 'ONLINE', timestamp);

        } catch (error) {
            console.error(`[SENSOR ERROR] ${sensorType}:`, error.message);
        }
    }

    async handleControlCommand(topic, payloadString) {
        const topicParts = topic.split('/');
        const controlType = topicParts[2];
        
        console.log(`[CONTROL] 🎛️ ${controlType}: ${payloadString}`);

        // Log control command
        const commandDoc = {
            topic: topic,
            control_type: controlType,
            command: payloadString,
            timestamp: new Date(),
            executed: true
        };

        await db.collection("control_commands").insertOne(commandDoc);
        console.log(`[DB] 📝 Command logged: ${controlType}`);

        // Store in command history (limit to last 100)
        this.commandHistory.unshift(commandDoc);
        if (this.commandHistory.length > 100) {
            this.commandHistory = this.commandHistory.slice(0, 100);
        }

        // Handle specific pH control commands
        if (controlType === 'ph_calibration' || controlType === 'ph_dosing') {
            await this.handlePhControlCommand(controlType, payloadString);
        }
    }

    async handleDeviceStatus(topic, payloadString) {
        const topicParts = topic.split('/');
        const deviceType = topicParts[2];
        
        console.log(`[STATUS] 📡 ${deviceType}: ${payloadString}`);

        // Update device status in database
        await this.updateDeviceStatus(deviceType, payloadString, new Date());
    }

    async handleSystemMessage(topic, payloadString) {
        if (topic === 'garden/system/device_info') {
            try {
                const deviceInfo = JSON.parse(payloadString);
                console.log(`[SYSTEM] 📱 Device info: ${deviceInfo.device_id} (${deviceInfo.type})`);
                
                // 🔧 FIXED: Validate device ID and handle null/empty values
                let deviceId = deviceInfo.device_id;
                
                // If deviceId is null, undefined, or empty, generate one
                if (!deviceId || deviceId === 'null' || deviceId === 'undefined') {
                    // Generate device ID from topic and type
                    deviceId = `ESP_AUTO_${deviceInfo.type || 'device'}_${Date.now()}`;
                    console.log(`[SYSTEM] 🔧 Generated device ID for null device: ${deviceId}`);
                }
                
                // Ensure deviceId is a valid string
                deviceId = deviceId.toString().replace(/[^a-zA-Z0-9_]/g, '_');
                
                // Store device information with proper error handling
                try {
                    await db.collection("devices").updateOne(
                        { device_id: deviceId },
                        { 
                            $set: {
                                ...deviceInfo,
                                device_id: deviceId, // Ensure device_id is set
                                last_seen: new Date(),
                                online: true,
                                updated_at: new Date()
                            },
                            $setOnInsert: {
                                created_at: new Date()
                            }
                        },
                        { upsert: true }
                    );
                    console.log(`[DB] ✅ Device info saved: ${deviceId}`);
                } catch (dbError) {
                    if (dbError.code === 11000) {
                        // Handle duplicate key error gracefully
                        console.log(`[DB] ⚠️ Device already exists: ${deviceId}, updating...`);
                        await db.collection("devices").updateOne(
                            { device_id: deviceId },
                            { 
                                $set: {
                                    last_seen: new Date(),
                                    online: true,
                                    updated_at: new Date()
                                }
                            }
                        );
                    } else {
                        throw dbError; // Re-throw if it's not a duplicate key error
                    }
                }
                
            } catch (error) {
                console.error(`[SYSTEM ERROR] Device info parse:`, error.message);
                console.error(`[SYSTEM ERROR] Stack trace:`, error.stack);
            }
        }
    }

    async handleM2MMessage(topic, payloadString) {
        console.log(`[M2M] 🤝 M2M Communication: ${payloadString}`);
        
        // Store M2M communication data
        const m2mDoc = {
            topic: topic,
            message: payloadString,
            timestamp: new Date(),
            processed: false
        };

        await db.collection("m2m_communications").insertOne(m2mDoc);

        // If it's board 3 data from micropython, process it
        if (payloadString.includes('board3_mpy')) {
            this.processMPyData(payloadString);
        }
    }

    async handleAlertMessage(topic, payloadString) {
        console.log(`[ALERT] 🚨 ${topic}: ${payloadString}`);
        
        const alertDoc = {
            topic: topic,
            message: payloadString,
            timestamp: new Date(),
            severity: this.getAlertSeverity(topic),
            acknowledged: false
        };

        await db.collection("alerts").insertOne(alertDoc);

        // Check for alert flooding
        const alertKey = `${topic}:${payloadString}`;
        if (this.alertCache.has(alertKey)) {
            const lastAlert = this.alertCache.get(alertKey);
            if (Date.now() - lastAlert < 300000) { // 5 minutes
                console.log(`[ALERT] ⚠️ Alert flooding detected for: ${alertKey}`);
                return; // Skip duplicate alert
            }
        }

        this.alertCache.set(alertKey, Date.now());

        // Clean old alerts from cache
        this.cleanAlertCache();

        // Send notification
        await this.sendNotification(alertDoc);
    }

    processMPyData(payloadString) {
        try {
            const data = JSON.parse(payloadString);
            
            // Handle button data from micropython
            if (data.buttons) {
                console.log(`[MPY] 🔘 Button states:`, data.buttons);
                
                // Update button states in database
                db.collection("button_states").updateOne(
                    { device_id: data.device_id },
                    {
                        $set: {
                            buttons: data.buttons,
                            timestamp: new Date(data.timestamp || Date.now())
                        }
                    },
                    { upsert: true }
                );
            }

            // Handle pH data from micropython if present
            if (data.ph_data) {
                console.log(`[MPY] 🧪 pH data received:`, data.ph_data);
                this.handlePhSensorData(data.ph_data, new Date());
            }
        } catch (error) {
            console.error(`[MPY ERROR] Process data:`, error.message);
        }
    }

    async updateDeviceStatus(deviceType, status, timestamp) {
        try {
            const updateDoc = {
                device_type: deviceType,
                status: status,
                last_update: timestamp,
                online: status !== 'OFFLINE' && status !== 'UNKNOWN'
            };

            await db.collection("device_status").updateOne(
                { device_type: deviceType },
                { 
                    $set: updateDoc, 
                    $setOnInsert: { created_at: timestamp }
                },
                { upsert: true }
            );

            console.log(`[DB] 📊 Device status updated: ${deviceType} = ${status}`);
        } catch (error) {
            console.error(`[STATUS ERROR] ${deviceType}:`, error.message);
        }
    }

    async handlePhSensorData(data, timestamp) {
        try {
            const deviceId = data.device_id || 'unknown';
            const phValue = data.ph_value || data.ph || data.value;
            const temperature = data.temperature || data.temp || 25; // Default 25°C
            const calibration = data.calibration || 'factory';
            const status = data.status || 'normal';

            // Store pH monitoring data
            const phData = {
                device_id: deviceId,
                ph_value: phValue,
                temperature: temperature,
                calibration: calibration,
                status: status,
                timestamp: timestamp,
                voltage: data.voltage || null,
                raw_value: data.raw_value || null,
                compensated_ph: this.calculateTemperatureCompensatedPH(phValue, temperature)
            };

            // Save to pH collection
            await db.collection("ph_readings").insertOne(phData);
            console.log(`[PH] 📊 pH reading saved: ${phValue} (${temperature}°C)`);

            // Update pH monitoring cache
            this.phMonitoring.set(deviceId, {
                latest: phData,
                history: this.phMonitoring.get(deviceId)?.history || [],
                lastAlert: this.phMonitoring.get(deviceId)?.lastAlert || null
            });

            // Keep only last 100 readings in memory
            const monitoring = this.phMonitoring.get(deviceId);
            monitoring.history.unshift(phData);
            if (monitoring.history.length > 100) {
                monitoring.history = monitoring.history.slice(0, 100);
            }

            // Check for pH alerts
            await this.checkPhAlerts(deviceId, phData);

        } catch (error) {
            console.error(`[PH ERROR] Error processing pH data:`, error.message);
        }
    }

    calculateTemperatureCompensatedPH(phValue, temperature) {
        // Simplified temperature compensation for pH
        // Formula: pH_compensated = pH_measured + (25 - temperature) * 0.018
        const compensationFactor = 0.018; // Standard compensation factor
        return +(phValue + (25 - temperature) * compensationFactor).toFixed(2);
    }

    async checkPhAlerts(deviceId, phData) {
        try {
            const currentPh = phData.compensated_ph || phData.ph_value;
            const alertKey = `ph-${deviceId}`;
            const lastAlert = this.phMonitoring.get(deviceId)?.lastAlert;

            // pH thresholds for hydroponic systems
            const thresholds = {
                critical_acidic: 5.5,
                warning_acidic: 6.0,
                optimal_min: 6.0,
                optimal_max: 7.5,
                warning_alkaline: 7.5,
                critical_alkaline: 8.0
            };

            let alertMessage = null;
            let severity = null;

            if (currentPh < thresholds.critical_acidic) {
                severity = 'critical';
                alertMessage = `pH critically low: ${currentPh} (Risk of nutrient lockout)`;
            } else if (currentPh < thresholds.warning_acidic) {
                severity = 'warning';
                alertMessage = `pH too acidic: ${currentPh} (Consider alkaline adjustment)`;
            } else if (currentPh > thresholds.critical_alkaline) {
                severity = 'critical';
                alertMessage = `pH critically high: ${currentPh} (Risk of nutrient precipitation)`;
            } else if (currentPh > thresholds.warning_alkaline) {
                severity = 'warning';
                alertMessage = `pH too alkaline: ${currentPh} (Consider acid adjustment)`;
            }

            if (alertMessage && severity) {
                // Check if we recently sent similar alert (5 minutes cooldown)
                const now = Date.now();
                if (!lastAlert || (now - lastAlert.timestamp) > 300000) {
                    // Create alert
                    await this.createPhAlert(deviceId, severity, alertMessage, phData);
                    
                    // Update last alert time
                    const monitoring = this.phMonitoring.get(deviceId);
                    if (monitoring) {
                        monitoring.lastAlert = {
                            timestamp: now,
                            severity: severity,
                            message: alertMessage
                        };
                    }

                    // Publish alert to MQTT
                    mqttClient.publish('garden/alerts/ph', JSON.stringify({
                        device_id: deviceId,
                        ph_value: currentPh,
                        temperature: phData.temperature,
                        severity: severity,
                        message: alertMessage,
                        timestamp: new Date()
                    }));

                    console.log(`[PH ALERT] 🚨 ${severity.toUpperCase()}: ${alertMessage}`);
                }
            }

        } catch (error) {
            console.error(`[PH ERROR] Error checking pH alerts:`, error.message);
        }
    }

    async createPhAlert(deviceId, severity, message, phData) {
        const alertDoc = {
            device_id: deviceId,
            alert_type: 'ph_level',
            severity: severity,
            message: message,
            timestamp: new Date(),
            ph_value: phData.ph_value,
            compensated_ph: phData.compensated_ph,
            temperature: phData.temperature,
            acknowledged: false,
            data: phData
        };

        await db.collection("alerts").insertOne(alertDoc);
    }

    async handlePhControlCommand(controlType, payloadString) {
        try {
            const data = JSON.parse(payloadString);
            const commandDoc = {
                command_type: controlType,
                parameters: data,
                timestamp: new Date(),
                status: 'received'
            };

            await db.collection("ph_commands").insertOne(commandDoc);
            console.log(`[PH CONTROL] 📋 pH command logged: ${controlType}`);

        } catch (error) {
            console.error(`[PH CONTROL ERROR]:`, error.message);
        }
    }

    // ===== NEW SENSOR HANDLERS =====

    async handleLightSensorData(data, timestamp) {
        try {
            const deviceId = data.device_id || 'unknown';
            const lightValue = data.light_value || data.lux || data.light;
            const status = data.status || 'normal';

            const lightData = {
                device_id: deviceId,
                light_value: lightValue,
                status: status,
                timestamp: timestamp,
                raw_value: data.raw_value || null,
                unit: data.unit || 'lux'
            };

            // Save to light readings collection
            await db.collection("light_readings").insertOne(lightData);
            console.log(`[LIGHT] 📊 Light reading saved: ${lightValue} lux`);

            // Check for light alerts
            await this.checkLightAlerts(deviceId, lightData);

        } catch (error) {
            console.error(`[LIGHT ERROR] Error processing light data:`, error.message);
        }
    }

    async handleTempHumSensorData(data, timestamp) {
        try {
            const deviceId = data.device_id || 'unknown';
            const temperature = data.temperature || data.temp || data.t;
            const humidity = data.humidity || data.hum || data.h;
            const status = data.status || 'normal';

            const tempHumData = {
                device_id: deviceId,
                temperature: temperature,
                humidity: humidity,
                status: status,
                timestamp: timestamp,
                heat_index: data.heat_index || null,
                dew_point: data.dew_point || null
            };

            // Save to temp_hum readings collection
            await db.collection("temp_hum_readings").insertOne(tempHumData);
            console.log(`[TEMP_HUM] 📊 Temp/Hum reading saved: ${temperature}°C / ${humidity}%`);

            // Check for temperature and humidity alerts
            await this.checkTempHumAlerts(deviceId, tempHumData);

        } catch (error) {
            console.error(`[TEMP_HUM ERROR] Error processing temp/hum data:`, error.message);
        }
    }

    async handleSoilMoistureSensorData(data, timestamp) {
        try {
            const deviceId = data.device_id || 'unknown';
            const moistureValue = data.moisture_value || data.moisture || data.m;
            const status = data.status || 'normal';

            const soilMoistureData = {
                device_id: deviceId,
                moisture_value: moistureValue,
                status: status,
                timestamp: timestamp,
                raw_value: data.raw_value || null,
                voltage: data.voltage || null,
                unit: data.unit || '%'
            };

            // Save to soil moisture readings collection
            await db.collection("soil_moisture_readings").insertOne(soilMoistureData);
            console.log(`[SOIL_MOISTURE] 📊 Soil moisture reading saved: ${moistureValue}%`);

            // Check for soil moisture alerts
            await this.checkSoilMoistureAlerts(deviceId, soilMoistureData);

        } catch (error) {
            console.error(`[SOIL_MOISTURE ERROR] Error processing soil moisture data:`, error.message);
        }
    }

    async handleNpkSensorData(data, timestamp) {
        try {
            const deviceId = data.device_id || 'unknown';
            const nValue = data.nitrogen || data.n || data.n_ppm || 0;
            const pValue = data.phosphorus || data.p || data.p_ppm || 0;
            const kValue = data.potassium || data.k || data.k_ppm || 0;
            const status = data.status || 'normal';

            const npkData = {
                device_id: deviceId,
                nitrogen: nValue,
                phosphorus: pValue,
                potassium: kValue,
                status: status,
                timestamp: timestamp,
                raw_n: data.raw_n || null,
                raw_p: data.raw_p || null,
                raw_k: data.raw_k || null,
                unit: data.unit || 'ppm'
            };

            // Save to NPK readings collection
            await db.collection("npk_readings").insertOne(npkData);
            console.log(`[NPK] 📊 NPK reading saved: N:${nValue} P:${pValue} K:${kValue} ppm`);

            // Check for NPK alerts
            await this.checkNpkAlerts(deviceId, npkData);

        } catch (error) {
            console.error(`[NPK ERROR] Error processing NPK data:`, error.message);
        }
    }

    // ===== ALERT CHECKING METHODS =====

    async checkLightAlerts(deviceId, lightData) {
        const lightValue = lightData.light_value;
        const alertKey = `light-${deviceId}`;
        const lastAlert = this.sensorMonitoring.get(alertKey)?.lastAlert;

        const thresholds = {
            critical_low: 200,      // Too dark for plant growth
            warning_low: 300,       // Low light, consider grow light
            optimal_min: 400,       // Minimum optimal light
            optimal_max: 800,       // Maximum optimal light
            warning_high: 900,      // High light, monitor for stress
            critical_high: 1000     // Critical high light
        };

        let alertMessage = null;
        let severity = null;

        if (lightValue < thresholds.critical_low) {
            severity = 'critical';
            alertMessage = `Light critically low: ${lightValue} lux (Insufficient for plant growth)`;
        } else if (lightValue < thresholds.warning_low) {
            severity = 'warning';
            alertMessage = `Light too low: ${lightValue} lux (Consider activating grow lights)`;
        } else if (lightValue > thresholds.critical_high) {
            severity = 'critical';
            alertMessage = `Light critically high: ${lightValue} lux (Risk of light stress)`;
        } else if (lightValue > thresholds.warning_high) {
            severity = 'warning';
            alertMessage = `Light too high: ${lightValue} lux (Monitor for plant stress)`;
        }

        if (alertMessage && severity) {
            await this.createSensorAlert(deviceId, 'light', severity, alertMessage, lightData);
        }
    }

    async checkTempHumAlerts(deviceId, tempHumData) {
        const temperature = tempHumData.temperature;
        const humidity = tempHumData.humidity;

        // Temperature alerts
        if (temperature > 35) {
            await this.createSensorAlert(deviceId, 'temperature', 'critical', 
                `Temperature critically high: ${temperature}°C`, tempHumData);
        } else if (temperature > 30) {
            await this.createSensorAlert(deviceId, 'temperature', 'warning', 
                `Temperature too high: ${temperature}°C`, tempHumData);
        } else if (temperature < 15) {
            await this.createSensorAlert(deviceId, 'temperature', 'critical', 
                `Temperature critically low: ${temperature}°C`, tempHumData);
        } else if (temperature < 18) {
            await this.createSensorAlert(deviceId, 'temperature', 'warning', 
                `Temperature too low: ${temperature}°C`, tempHumData);
        }

        // Humidity alerts
        if (humidity > 85) {
            await this.createSensorAlert(deviceId, 'humidity', 'warning', 
                `Humidity too high: ${humidity}% (Risk of fungal diseases)`, tempHumData);
        } else if (humidity < 30) {
            await this.createSensorAlert(deviceId, 'humidity', 'warning', 
                `Humidity too low: ${humidity}% (Risk of plant stress)`, tempHumData);
        }
    }

    async checkSoilMoistureAlerts(deviceId, soilData) {
        const moistureValue = soilData.moisture_value;
        const alertKey = `soil-${deviceId}`;
        const lastAlert = this.sensorMonitoring.get(alertKey)?.lastAlert;

        const thresholds = {
            critical_low: 20,       // Very dry, immediate watering needed
            warning_low: 30,        // Dry, watering recommended
            optimal_min: 40,        // Minimum optimal moisture
            optimal_max: 70,        // Maximum optimal moisture
            warning_high: 80,       // High moisture, monitor
            critical_high: 90       // Waterlogged
        };

        let alertMessage = null;
        let severity = null;

        if (moistureValue < thresholds.critical_low) {
            severity = 'critical';
            alertMessage = `Soil moisture critically low: ${moistureValue}% (Immediate watering required)`;
        } else if (moistureValue < thresholds.warning_low) {
            severity = 'warning';
            alertMessage = `Soil moisture low: ${moistureValue}% (Watering recommended)`;
        } else if (moistureValue > thresholds.critical_high) {
            severity = 'critical';
            alertMessage = `Soil moisture critically high: ${moistureValue}% (Waterlogging risk)`;
        } else if (moistureValue > thresholds.warning_high) {
            severity = 'warning';
            alertMessage = `Soil moisture high: ${moistureValue}% (Monitor for overwatering)`;
        }

        if (alertMessage && severity) {
            await this.createSensorAlert(deviceId, 'soil_moisture', severity, alertMessage, soilData);
        }
    }

    async checkNpkAlerts(deviceId, npkData) {
        const n = npkData.nitrogen;
        const p = npkData.phosphorus;
        const k = npkData.potassium;

        const thresholds = {
            nitrogen: { low: 20, critical: 10 },
            phosphorus: { low: 10, critical: 5 },
            potassium: { low: 30, critical: 15 }
        };

        // Check Nitrogen
        if (n < thresholds.nitrogen.critical) {
            await this.createSensorAlert(deviceId, 'npk_nitrogen', 'critical', 
                `Nitrogen critically low: ${n} ppm`, npkData);
        } else if (n < thresholds.nitrogen.low) {
            await this.createSensorAlert(deviceId, 'npk_nitrogen', 'warning', 
                `Nitrogen low: ${n} ppm`, npkData);
        }

        // Check Phosphorus
        if (p < thresholds.phosphorus.critical) {
            await this.createSensorAlert(deviceId, 'npk_phosphorus', 'critical', 
                `Phosphorus critically low: ${p} ppm`, npkData);
        } else if (p < thresholds.phosphorus.low) {
            await this.createSensorAlert(deviceId, 'npk_phosphorus', 'warning', 
                `Phosphorus low: ${p} ppm`, npkData);
        }

        // Check Potassium
        if (k < thresholds.potassium.critical) {
            await this.createSensorAlert(deviceId, 'npk_potassium', 'critical', 
                `Potassium critically low: ${k} ppm`, npkData);
        } else if (k < thresholds.potassium.low) {
            await this.createSensorAlert(deviceId, 'npk_potassium', 'warning', 
                `Potassium low: ${k} ppm`, npkData);
        }
    }

    async createSensorAlert(deviceId, sensorType, severity, message, sensorData) {
        const alertDoc = {
            device_id: deviceId,
            alert_type: sensorType,
            severity: severity,
            message: message,
            timestamp: new Date(),
            acknowledged: false,
            sensor_data: sensorData
        };

        await db.collection("alerts").insertOne(alertDoc);

        // Publish alert to MQTT
        mqttClient.publish(`garden/alerts/${sensorType}`, JSON.stringify({
            device_id: deviceId,
            sensor_type: sensorType,
            severity: severity,
            message: message,
            timestamp: new Date(),
            data: sensorData
        }));

        console.log(`[${sensorType.toUpperCase()} ALERT] 🚨 ${severity.toUpperCase()}: ${message}`);
    }

    updateDeviceActivity(topic, payloadString) {
        const deviceId = this.extractDeviceId(topic, payloadString);
        if (deviceId) {
            this.devices.set(deviceId, {
                lastSeen: Date.now(),
                lastTopic: topic,
                lastPayload: payloadString
            });
        }
    }

    extractDeviceId(topic, payloadString) {
        try {
            const data = JSON.parse(payloadString);
            return data.device_id;
        } catch (error) {
            return null;
        }
    }

    getAlertSeverity(topic) {
        const severityMap = {
            'emergency': 'CRITICAL',
            'critical': 'CRITICAL',
            'high': 'WARNING',
            'warning': 'WARNING',
            'low': 'INFO',
            'info': 'INFO',
            'ph': 'WARNING' // pH alerts default to warning
        };

        for (const [key, severity] of Object.entries(severityMap)) {
            if (topic.toLowerCase().includes(key)) {
                return severity;
            }
        }
        return 'INFO';
    }

    cleanAlertCache() {
        const now = Date.now();
        const expireTime = 3600000; // 1 hour

        for (const [key, timestamp] of this.alertCache.entries()) {
            if (now - timestamp > expireTime) {
                this.alertCache.delete(key);
            }
        }
    }

    async sendNotification(alertDoc) {
        // In a real implementation, this would send notifications
        // via email, SMS, push notifications, etc.
        console.log(`[NOTIFICATION] 📱 Would send notification: ${alertDoc.severity} - ${alertDoc.message}`);
        
        // Store notification status
        await db.collection("notifications").insertOne({
            ...alertDoc,
            sent: true,
            sent_at: new Date()
        });
    }

    publishSystemStatus(status) {
        const statusMsg = {
            broker: status,
            timestamp: new Date(),
            version: '3.0.0', // Updated version with full sensor support
            uptime: process.uptime(),
            devices_online: this.getOnlineDevicesCount(),
            features: ['mqtt', 'mongodb', 'automation', 'ph_monitoring', 'light_monitoring', 
                      'temp_hum_monitoring', 'soil_moisture_monitoring', 'npk_monitoring']
        };

        mqttClient.publish('garden/system/status', JSON.stringify(statusMsg));
        console.log(`[SYSTEM] 📡 System status published: ${status}`);
    }

    getOnlineDevicesCount() {
        let count = 0;
        const now = Date.now();
        const timeout = 300000; // 5 minutes

        for (const [deviceId, info] of this.devices.entries()) {
            if (now - info.lastSeen < timeout) {
                count++;
            }
        }
        return count;
    }

    startHealthMonitoring() {
        // Monitor device health every minute
        setInterval(() => {
            this.checkDeviceHealth();
        }, 60000);

        // Send system status every 5 minutes
        setInterval(() => {
            this.publishSystemStatus('HEALTHY');
        }, 300000);

        // Clean up old data every hour
        setInterval(() => {
            this.cleanupOldData();
        }, 3600000);

        // Monitor all sensor systems health every 2 minutes
        setInterval(() => {
            this.checkSensorSystemHealth();
        }, 120000);
    }

    checkDeviceHealth() {
        const now = Date.now();
        const timeout = 300000; // 5 minutes
        const offlineDevices = [];

        for (const [deviceId, info] of this.devices.entries()) {
            if (now - info.lastSeen > timeout) {
                offlineDevices.push(deviceId);
                this.updateDeviceStatus('unknown', 'OFFLINE', new Date());
            }
        }

        if (offlineDevices.length > 0) {
            console.log(`[HEALTH] ⚠️ Offline devices: ${offlineDevices.join(', ')}`);
            
            // Mark devices as offline in database
            db.collection("device_status").updateMany(
                { device_id: { $in: offlineDevices } },
                { 
                    $set: { 
                        status: 'OFFLINE', 
                        last_update: new Date(),
                        online: false
                    } 
                }
            );
        }
    }

    async checkSensorSystemHealth() {
        try {
            // Check pH sensor readings are recent
            for (const [deviceId, monitoring] of this.phMonitoring.entries()) {
                const latestReading = monitoring.latest;
                if (latestReading) {
                    const timeSinceReading = Date.now() - new Date(latestReading.timestamp).getTime();
                    
                    // Alert if no pH reading for 15 minutes
                    if (timeSinceReading > 15 * 60 * 1000) {
                        await this.createPhAlert(deviceId, 'warning', 
                            `No pH readings for ${Math.floor(timeSinceReading / 60000)} minutes`, 
                            { ph_value: 0, temperature: 0 });
                    }
                }
            }

            // Check other sensor health
            const healthChecks = [
                { collection: 'light_readings', type: 'light', timeout: 15 * 60 * 1000 },
                { collection: 'temp_hum_readings', type: 'temp_hum', timeout: 10 * 60 * 1000 },
                { collection: 'soil_moisture_readings', type: 'soil_moisture', timeout: 15 * 60 * 1000 },
                { collection: 'npk_readings', type: 'npk', timeout: 60 * 60 * 1000 }
            ];

            for (const check of healthChecks) {
                const latestReading = await db.collection(check.collection)
                    .find({})
                    .sort({ timestamp: -1 })
                    .limit(1)
                    .toArray();

                if (latestReading.length > 0) {
                    const timeSinceReading = Date.now() - new Date(latestReading[0].timestamp).getTime();
                    
                    if (timeSinceReading > check.timeout) {
                        console.log(`[${check.type.toUpperCase()} HEALTH] ⚠️ No readings for ${Math.floor(timeSinceReading / 60000)} minutes`);
                    }
                }
            }
        } catch (error) {
            console.error(`[SENSOR HEALTH ERROR]:`, error.message);
        }
    }

    async cleanupOldData() {
        try {
            const thirtyDaysAgo = new Date(Date.now() - 30 * 24 * 60 * 60 * 1000);
            
            // Clean old sensor readings
            const sensorResult = await db.collection("sensor_readings").deleteMany({
                timestamp: { $lt: thirtyDaysAgo }
            });
            
            // Clean old sensor readings (keep different times based on importance)
            const sixtyDaysAgo = new Date(Date.now() - 60 * 24 * 60 * 60 * 1000); // pH: 60 days
            const phResult = await db.collection("ph_readings").deleteMany({
                timestamp: { $lt: sixtyDaysAgo }
            });

            // Light readings: 30 days
            //const thirtyDaysAgo = new Date(Date.now() - 30 * 24 * 60 * 60 * 1000);
            const lightResult = await db.collection("light_readings").deleteMany({
                timestamp: { $lt: thirtyDaysAgo }
            });

            // Temp/Hum readings: 30 days
            const tempHumResult = await db.collection("temp_hum_readings").deleteMany({
                timestamp: { $lt: thirtyDaysAgo }
            });

            // Soil moisture readings: 45 days
            const fortyFiveDaysAgo = new Date(Date.now() - 45 * 24 * 60 * 60 * 1000);
            const soilResult = await db.collection("soil_moisture_readings").deleteMany({
                timestamp: { $lt: fortyFiveDaysAgo }
            });

            // NPK readings: 30 days
            const npkResult = await db.collection("npk_readings").deleteMany({
                timestamp: { $lt: thirtyDaysAgo }
            });
            
            // Clean old alerts
            const alertResult = await db.collection("alerts").deleteMany({
                timestamp: { $lt: thirtyDaysAgo },
                acknowledged: true
            });
            
            console.log(`[CLEANUP] 🧹 Cleaned old data: ${sensorResult.deletedCount} general readings, ` +
                       `${phResult.deletedCount} pH readings, ${lightResult.deletedCount} light readings, ` +
                       `${tempHumResult.deletedCount} temp/hum readings, ${soilResult.deletedCount} soil readings, ` +
                       `${npkResult.deletedCount} NPK readings, ${alertResult.deletedCount} alerts`);
            
        } catch (error) {
            console.error(`[CLEANUP ERROR]:`, error.message);
        }
    }

    stop() {
        console.log('🛑 Stopping Smart Garden Broker...');
        
        if (mqttClient) {
            this.publishSystemStatus('OFFLINE');
            mqttClient.end();
        }
        
        if (db) {
            db.client.close();
        }
        
        console.log('✅ Smart Garden Broker stopped');
    }
}

// Create and start broker
const broker = new SmartGardenBroker();

// Handle graceful shutdown
process.on('SIGINT', () => {
    console.log('\n🛑 Received SIGINT, shutting down gracefully...');
    broker.stop();
    process.exit(0);
});

process.on('SIGTERM', () => {
    console.log('\n🛑 Received SIGTERM, shutting down gracefully...');
    broker.stop();
    process.exit(0);
});

// Start the broker
broker.start().catch(console.error);

module.exports = SmartGardenBroker;
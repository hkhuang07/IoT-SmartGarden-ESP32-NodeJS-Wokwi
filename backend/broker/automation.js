/**
 * Smart Garden Automation Logic
 * Handles automatic control based on sensor data and predefined rules
 */

const AUTOMATION_RULES = {
    // Sensor thresholds
    THRESHOLDS: {
        soilMoisture: {
            critical: 30,    // Cần tưới ngay
            low: 40,         // Cần tưới sớm
            optimal: 60      // Độ ẩm tối ưu
        },
        temperature: {
            min: 18,         // Nhiệt độ tối thiểu
            max: 35,         // Nhiệt độ tối đa
            optimal: 25      // Nhiệt độ tối ưu
        },
        humidity: {
            min: 40,         // Độ ẩm tối thiểu
            max: 80,         // Độ ẩm tối đa
            optimal: 60      // Độ ẩm tối ưu
        },
        light: {
            min: 300,        // Ánh sáng tối thiểu
            max: 900,        // Ánh sáng tối đa
            optimal: 600     // Ánh sáng tối ưu
        }
    },
    
    // Pump control rules
    PUMP_CONTROL: {
        maxContinuousTime: 30 * 60 * 1000,    // 30 phút tối đa
        minCooldown: 5 * 60 * 1000,           // 5 phút tối thiểu giữa các lần
        emergencyStop: 45 * 60 * 1000         // Dừng khẩn cấp sau 45 phút
    },
    
    // Alert settings
    ALERTS: {
        cooldown: 5 * 60 * 1000,              // 5 phút giữa các alert cùng loại
        maxPerHour: 10,                        // Tối đa 10 alert mỗi giờ
        severity: {
            low: 'info',
            medium: 'warning',
            high: 'error',
            critical: 'critical'
        }
    }
};

class SmartGardenAutomation {
    constructor() {
        this.devices = new Map();
        this.alertCache = new Map();
        this.commandHistory = new Map();
        this.lastPumpActivation = new Map();
        this.sensorData = new Map();
    }
    
    /**
     * Process incoming sensor data and trigger automation rules
     */
    async processSensorData(deviceId, sensorType, value, timestamp, mqttClient = null, broker = null) {
        try {
            // Store sensor data
            const key = `${deviceId}-${sensorType}`;
            this.sensorData.set(key, { value, timestamp });
            
            // Process specific automation rules based on sensor type
            switch (sensorType) 
            {
                //case 'soilMoisture':
                case 'soil_moisture':
                case 'soil_moisture_data':
                    return await this.handleSoilMoisture(deviceId, value, timestamp);
                case 'temp_hum':
                case 'temp_hum_data':
                    return await this.handleTempHum(deviceId, value, timestamp);
                case 'temperature':
                    return await this.handleTemperature(deviceId, value, timestamp);
                case 'humidity':
                    return await this.handleHumidity(deviceId, value, timestamp);
                case 'light':
                case 'light_data':
                    return await this.handleLight(deviceId, value, timestamp);
                case 'ph':
                case 'ph_level':
                case 'ph_data':
                    return await this.handlePhLevel(deviceId, value, timestamp, mqttClient, broker);
                case 'npk':
                case 'npk_data':
                    return await this.handleNpkLevels(deviceId, value, timestamp);
                default:
                    console.log(`🤖 Unknown sensor type: ${sensorType}`);
                    return null;
            }
        } catch (error) {
            console.error('❌ Error processing sensor data:', error);
            return null;
        }
    }
    
    /**
     * Handle soil moisture automation
     */
    async handleSoilMoisture(deviceId, moisture, timestamp) {
        const threshold = AUTOMATION_RULES.THRESHOLDS.soilMoisture;
        
        if (moisture <= threshold.critical) {
            await this.triggerPump(deviceId, 'critical-low-moisture', timestamp);
            await this.createAlert(deviceId, 'soil-moisture', 'critical', 
                `Soil moisture critically low (${moisture}%)`, timestamp);
            return { action: 'pump-activated', reason: 'critical-low-moisture' };
        }
        
        if (moisture <= threshold.low) {
            await this.triggerPump(deviceId, 'low-moisture', timestamp);
            await this.createAlert(deviceId, 'soil-moisture', 'warning', 
                `Soil moisture low (${moisture}%)`, timestamp);
            return { action: 'pump-activated', reason: 'low-moisture' };
        }
        
        if (moisture >= threshold.optimal) {
            await this.deactivatePump(deviceId, 'optimal-moisture-reached', timestamp);
            return { action: 'pump-deactivated', reason: 'optimal-moisture-reached' };
        }
        
        return { action: 'no-action', reason: 'moisture-within-range' };
    }
    
    /**
     * Handle temperature and humidity automation
     */
    async handleTempHum(deviceId, tempHumData, timestamp) {
        const threshold = AUTOMATION_RULES.THRESHOLDS;
        
        let actions = [];
        
        // Handle temperature
        if (tempHumData.temperature > threshold.temperature.max) {
            await this.triggerVentilation(deviceId, 'high-temperature', timestamp);
            await this.createAlert(deviceId, 'temperature', 'warning', 
                `Temperature too high (${tempHumData.temperature}°C)`, timestamp);
            actions.push('ventilation-activated-high-temp');
        } else if (tempHumData.temperature < threshold.temperature.min) {
            await this.createAlert(deviceId, 'temperature', 'warning', 
                `Temperature too low (${tempHumData.temperature}°C)`, timestamp);
            actions.push('heating-recommended');
        }
        
        // Handle humidity
        if (tempHumData.humidity > threshold.humidity.max) {
            await this.triggerVentilation(deviceId, 'high-humidity', timestamp);
            await this.createAlert(deviceId, 'humidity', 'warning', 
                `Humidity too high (${tempHumData.humidity}%)`, timestamp);
            actions.push('ventilation-activated-high-humidity');
        } else if (tempHumData.humidity < threshold.humidity.min) {
            await this.triggerHumidifier(deviceId, 'low-humidity', timestamp);
            await this.createAlert(deviceId, 'humidity', 'warning', 
                `Humidity too low (${tempHumData.humidity}%)`, timestamp);
            actions.push('humidifier-activated');
        }
        
        // Calculate heat index if available
        if (tempHumData.heat_index && tempHumData.heat_index > 35) {
            await this.createAlert(deviceId, 'heat-index', 'warning', 
                `High heat index detected (${tempHumData.heat_index}°C)`, timestamp);
            actions.push('cooling-required');
        }
            
        return {
            action: actions.length > 0 ? actions.join(',') : 'no-action',
            reason: 'temp-hum-within-range',
            temperature: tempHumData.temperature,
            humidity: tempHumData.humidity,
            heat_index: tempHumData.heat_index || null
        };
    }
    
    /**
     * Handle humidity automation
     */
    async handleHumidity(deviceId, humidity, timestamp) {
        const threshold = AUTOMATION_RULES.THRESHOLDS.humidity;
        
        if (humidity > threshold.max) {
            await this.triggerVentilation(deviceId, 'high-humidity', timestamp);
            await this.createAlert(deviceId, 'humidity', 'warning', 
                `Humidity too high (${humidity}%)`, timestamp);
            return { action: 'ventilation-activated', reason: 'high-humidity' };
        }
        
        if (humidity < threshold.min) {
            await this.createAlert(deviceId, 'humidity', 'warning', 
                `Humidity too low (${humidity}%)`, timestamp);
            return { action: 'alert-created', reason: 'low-humidity' };
        }
        
        return { action: 'no-action', reason: 'humidity-within-range' };
    }

     /**
     * Handle humidity automation
     */
    async handleTemperature(deviceId, temperature, timestamp) {
        const threshold = AUTOMATION_RULES.THRESHOLDS.temperature;
        
        if (temperature > threshold.max) {
            await this.triggerVentilation(deviceId, 'high-temperature', timestamp);
            await this.createAlert(deviceId, 'temperature', 'warning', 
                `Temperature too high (${temperature}°C)`, timestamp);
            return { action: 'ventilation-activated', reason: 'high-temperature' };
        }
        
        if (temperature < threshold.min) {
            await this.createAlert(deviceId, 'temperature', 'warning', 
                `Temperature too low (${temperature}°C)`, timestamp);
            return { action: 'alert-created', reason: 'low-temperature' };
        }
        
        return { action: 'no-action', reason: 'temperature-within-range' };
    }
    
    /**
     * Handle light automation
     */
    async handleLight(deviceId, light, timestamp) {
        const threshold = AUTOMATION_RULES.THRESHOLDS.light;
        
        if (light < threshold.min) {
            await this.triggerGrowLight(deviceId, 'low-light', timestamp);
            return { action: 'grow-light-activated', reason: 'low-light' };
        }
        
        if (light >= threshold.optimal) {
            await this.deactivateGrowLight(deviceId, 'optimal-light-reached', timestamp);
            return { action: 'grow-light-deactivated', reason: 'optimal-light-reached' };
        }
        
        return { action: 'no-action', reason: 'light-within-range' };
    }
    
    /**
     * Trigger pump activation with safety checks
     */
    async triggerPump(deviceId, reason, timestamp) {
        try {
            // Check if pump was recently activated
            const lastActivation = this.lastPumpActivation.get(deviceId);
            if (lastActivation) {
                const timeSinceLastActivation = timestamp - lastActivation;
                if (timeSinceLastActivation < AUTOMATION_RULES.PUMP_CONTROL.minCooldown) {
                    console.log(`🚫 Pump activation blocked: too soon since last activation (${Math.floor(timeSinceLastActivation / 1000)}s ago)`);
                    return false;
                }
            }
            
            // Store activation time
            this.lastPumpActivation.set(deviceId, timestamp);
            
            // Create command for pump activation
            const command = {
                deviceId,
                command: 'activate-pump',
                parameters: { reason, duration: 300000 }, // 5 minutes default
                timestamp,
                commandId: `pump-${deviceId}-${timestamp}`
            };
            
            console.log(`💧 Pump activated for device ${deviceId}: ${reason}`);
            return command;
        } catch (error) {
            console.error('❌ Error triggering pump:', error);
            return false;
        }
    }
    
    /**
     * Deactivate pump
     */
    async deactivatePump(deviceId, reason, timestamp) {
        console.log(`⏹️ Pump deactivated for device ${deviceId}: ${reason}`);
        return {
            deviceId,
            command: 'deactivate-pump',
            parameters: { reason },
            timestamp,
            commandId: `pump-stop-${deviceId}-${timestamp}`
        };
    }
    
    /**
     * Trigger ventilation system
     */
    async triggerVentilation(deviceId, reason, timestamp) {
        console.log(`🌬️ Ventilation activated for device ${deviceId}: ${reason}`);
        return {
            deviceId,
            command: 'activate-ventilation',
            parameters: { reason },
            timestamp,
            commandId: `vent-${deviceId}-${timestamp}`
        };
    }
    
    /**
     * Trigger grow light
     */
    async triggerGrowLight(deviceId, reason, timestamp) {
        console.log(`💡 Grow light activated for device ${deviceId}: ${reason}`);
        return {
            deviceId,
            command: 'activate-grow-light',
            parameters: { reason },
            timestamp,
            commandId: `light-${deviceId}-${timestamp}`
        };
    }
    
    /**
     * Deactivate grow light
     */
    async deactivateGrowLight(deviceId, reason, timestamp) {
        console.log(`🌑 Grow light deactivated for device ${deviceId}: ${reason}`);
        return {
            deviceId,
            command: 'deactivate-grow-light',
            parameters: { reason },
            timestamp,
            commandId: `light-stop-${deviceId}-${timestamp}`
        };
    }
    
    /**
     * Trigger humidifier
     */
    async triggerHumidifier(deviceId, reason, timestamp) {
        console.log(`💧 Humidifier activated for device ${deviceId}: ${reason}`);
        return {
            deviceId,
            command: 'activate-humidifier',
            parameters: { reason },
            timestamp,
            commandId: `humidifier-${deviceId}-${timestamp}`
        };
    }
    
    /**
     * Trigger nutrient dosing
     */
    async triggerNutrientDosing(deviceId, nutrientType, amount, reason, timestamp) {
        console.log(`🧪 Nutrient dosing: ${nutrientType} (${amount}ml) for device ${deviceId}: ${reason}`);
        return {
            deviceId,
            command: 'nutrient-dosing',
            parameters: { 
                nutrient_type: nutrientType,
                amount: amount,
                reason: reason
            },
            timestamp,
            commandId: `nutrient-${deviceId}-${nutrientType}-${timestamp}`
        };
    }
    
    /**
     * Trigger pH adjustment
     */
    async triggerPhAdjustment(deviceId, phDirection, amount, reason, timestamp) {
        console.log(`⚗️ pH adjustment: ${phDirection} (${amount}ml) for device ${deviceId}: ${reason}`);
        return {
            deviceId,
            command: 'ph-adjustment',
            parameters: { 
                ph_direction: phDirection, // 'up' or 'down'
                amount: amount,
                reason: reason
            },
            timestamp,
            commandId: `ph-${deviceId}-${phDirection}-${timestamp}`
        };
    }
    
    /**
     * Create alert with cooldown management
     */
    async createAlert(deviceId, alertType, severity, message, timestamp) {
        try {
            const alertKey = `${deviceId}-${alertType}`;
            const lastAlert = this.alertCache.get(alertKey);
            
            // Check cooldown
            if (lastAlert && (timestamp - lastAlert.timestamp) < AUTOMATION_RULES.ALERTS.cooldown) {
                console.log(`🚫 Alert blocked (cooldown): ${alertType} for ${deviceId}`);
                return false;
            }
            
            // Store alert in cache
            this.alertCache.set(alertKey, { timestamp, type: alertType, severity });
            
            const alert = {
                deviceId,
                alertType,
                severity: AUTOMATION_RULES.ALERTS.severity[severity] || 'info',
                message,
                timestamp,
                acknowledged: false,
                alertId: `alert-${deviceId}-${alertType}-${timestamp}`
            };
            
            console.log(`⚠️ Alert created: ${alertType} (${severity}) for ${deviceId}: ${message}`);
            return alert;
        } catch (error) {
            console.error('❌ Error creating alert:', error);
            return false;
        }
    }
    
    /**
     * Handle pH level automation
     */
    async handlePhLevel(deviceId, phData, timestamp, mqttClient = null, broker = null) {
        try {
            const phValue = typeof phData === 'object' ? phData.ph_value : phData;
            const temperature = typeof phData === 'object' ? (phData.temperature || 25) : 25;
            
            // pH thresholds for hydroponic systems (more specific than general plant health)
            const optimalRange = { min: 6.0, max: 7.5 };
            const critical_acidic = 5.5;
            const critical_alkaline = 8.0;
            
            // Temperature compensation for pH readings
            const compensatedPh = phValue + (25 - temperature) * 0.018;
            
            let actions = [];
            let severity = 'info';
            
            if (compensatedPh < critical_acidic) {
                severity = 'critical';
                await this.createAlert(deviceId, 'ph-level', 'critical', 
                    `pH critically low (${compensatedPh.toFixed(1)} @ ${temperature}°C) - Risk of nutrient lockout`, timestamp);
                actions.push('emergency-alkaline-dosing');
                
                // Publish emergency command via MQTT if available
                if (mqttClient) {
                    mqttClient.publish(`garden/control/${deviceId}/ph_dosing`, JSON.stringify({
                        type: 'alkaline',
                        severity: 'critical',
                        current_ph: compensatedPh,
                        target_ph: optimalRange.min,
                        temperature: temperature,
                        timestamp: new Date()
                    }));
                }
                
            } else if (compensatedPh < optimalRange.min) {
                severity = 'warning';
                await this.createAlert(deviceId, 'ph-level', 'warning', 
                    `pH too acidic (${compensatedPh.toFixed(1)} @ ${temperature}°C) - Consider alkaline adjustment`, timestamp);
                actions.push('alkaline-dosing-recommended');
                
            } else if (compensatedPh > critical_alkaline) {
                severity = 'critical';
                await this.createAlert(deviceId, 'ph-level', 'critical', 
                    `pH critically high (${compensatedPh.toFixed(1)} @ ${temperature}°C) - Risk of nutrient precipitation`, timestamp);
                actions.push('emergency-acid-dosing');
                
                // Publish emergency command via MQTT if available
                if (mqttClient) {
                    mqttClient.publish(`garden/control/${deviceId}/ph_dosing`, JSON.stringify({
                        type: 'acid',
                        severity: 'critical',
                        current_ph: compensatedPh,
                        target_ph: optimalRange.max,
                        temperature: temperature,
                        timestamp: new Date()
                    }));
                }
                
            } else if (compensatedPh > optimalRange.max) {
                severity = 'warning';
                await this.createAlert(deviceId, 'ph-level', 'warning', 
                    `pH too alkaline (${compensatedPh.toFixed(1)} @ ${temperature}°C) - Consider acid adjustment`, timestamp);
                actions.push('acid-dosing-recommended');
            } else {
                // pH is in optimal range
                actions.push('ph-optimal');
                console.log(`[pH] ✅ Optimal pH maintained: ${compensatedPh.toFixed(1)} @ ${temperature}°C`);
            }
            
            return { 
                action: actions.join(','), 
                reason: `ph-${severity}`,
                phValue: phValue,
                compensated_ph: compensatedPh,
                temperature: temperature,
                targetRange: optimalRange,
                actions: actions,
                timestamp: new Date(timestamp)
            };
            
        } catch (error) {
            console.error('❌ Error handling pH data:', error);
            return { action: 'error', reason: error.message };
        }
    }
    
    /**
     * Handle NPK levels automation
     */
    async handleNpkLevels(deviceId, npkData, timestamp) {
        try {
            // NPK data should be an object with n_ppm, p_ppm, k_ppm
            const n = npkData.n_ppm || 0;
            const p = npkData.p_ppm || 0;
            const k = npkData.k_ppm || 0;
            
            // Basic NPK thresholds (can be refined based on plant type)
            const thresholds = {
                nitrogen: { low: 20, optimal: 50, high: 100 },
                phosphorus: { low: 10, optimal: 30, high: 60 },
                potassium: { low: 30, optimal: 60, high: 120 }
            };
            
            let actions = [];
            
            // Check Nitrogen levels
            if (n < thresholds.nitrogen.low) {
                await this.createAlert(deviceId, 'npk-levels', 'warning', 
                    `Nitrogen too low (${n} ppm)`, timestamp);
                actions.push('nitrogen-supplement-recommended');
            }
            
            // Check Phosphorus levels  
            if (p < thresholds.phosphorus.low) {
                await this.createAlert(deviceId, 'npk-levels', 'warning', 
                    `Phosphorus too low (${p} ppm)`, timestamp);
                actions.push('phosphorus-supplement-recommended');
            }
            
            // Check Potassium levels
            if (k < thresholds.potassium.low) {
                await this.createAlert(deviceId, 'npk-levels', 'warning', 
                    `Potassium too low (${k} ppm)`, timestamp);
                actions.push('potassium-supplement-recommended');
            }
            
            return {
                action: actions.length > 0 ? 'nutrient-review-needed' : 'nutrients-adequate',
                reasons: actions,
                npkValues: { nitrogen: n, phosphorus: p, potassium: k }
            };
            
        } catch (error) {
            console.error('❌ Error handling NPK data:', error);
            return { action: 'error', reason: error.message };
        }
    }
    
    /**
     * Get automation status
     */
    getStatus() {
        return {
            activeDevices: this.devices.size,
            cachedAlerts: this.alertCache.size,
            pumpActivations: this.lastPumpActivation.size,
            sensorDataPoints: this.sensorData.size,
            thresholds: AUTOMATION_RULES.THRESHOLDS,
            rules: {
                pumpControl: AUTOMATION_RULES.PUMP_CONTROL,
                alerts: AUTOMATION_RULES.ALERTS
            }
        };
    }
    
    /**
     * Reset automation cache (useful for testing)
     */
    reset() {
        this.devices.clear();
        this.alertCache.clear();
        this.commandHistory.clear();
        this.lastPumpActivation.clear();
        this.sensorData.clear();
        console.log('🔄 Automation cache reset');
    }
}

module.exports = SmartGardenAutomation;
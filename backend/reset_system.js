const { MongoClient } = require('mongodb');
const fs = require('fs');
const path = require('path');

// Configuration
const MONGODB_URI = 'mongodb+srv://iot_smartgarden:IoT_Smart_Garden@cluster0.17bgl.mongodb.net/?appName=Cluster0';
const DB_NAME = 'iot_smartgarden';

/**
 * Reset và thiết lập lại database cho Smart Garden System
 */
class SystemReset {
    constructor() {
        this.client = null;
        this.db = null;
    }

    async connect() {
        try {
            console.log('🔌 Kết nối MongoDB Atlas...');
            this.client = new MongoClient(MONGODB_URI, {
                useNewUrlParser: true,
                useUnifiedTopology: true,
            });
            await this.client.connect();
            this.db = this.client.db(DB_NAME);
            console.log('✅ Kết nối MongoDB thành công!');
            return this.db;
        } catch (error) {
            console.error('❌ Lỗi kết nối MongoDB:', error);
            throw error;
        }
    }

    /**
     * Xóa tất cả collections cũ
     */
    async dropExistingCollections() {
        try {
            console.log('🗑️ Xóa collections cũ...');
            const collections = await this.db.listCollections().toArray();
            
            for (const collection of collections) {
                await this.db.collection(collection.name).drop();
                console.log(`   ✅ Đã xóa collection: ${collection.name}`);
            }
        } catch (error) {
            console.error('❌ Lỗi xóa collections:', error);
        }
    }

    /**
     * Tạo và setup tất cả collections
     */
    async createCollections() {
        try {
            console.log('📊 Tạo collections mới...');

            // 1. Devices collection
            await this.createDevicesCollection();
            
            // 2. Sensor readings collection
            await this.createSensorReadingsCollection();
            
            // 3. Device status collection
            await this.createDeviceStatusCollection();
            
            // 4. Control commands collection
            await this.createControlCommandsCollection();
            
            // 5. Alerts collection
            await this.createAlertsCollection();
            
            // 6. System config collection
            await this.createSystemConfigCollection();
            
            // 7. System logs collection
            await this.createSystemLogsCollection();
            
            // 8. Automation rules collection
            await this.createAutomationRulesCollection();
            
            console.log('✅ Tạo tất cả collections thành công!');
        } catch (error) {
            console.error('❌ Lỗi tạo collections:', error);
        }
    }

    /**
     * Tạo devices collection
     */
    async createDevicesCollection() {
        const collection = this.db.collection('devices');
        await collection.createIndex({ device_id: 1 }, { unique: true });
        await collection.createIndex({ board_number: 1 });
        await collection.createIndex({ last_seen: -1 });
        await collection.createIndex({ online: 1 });
        
        // Insert default devices
        const devices = [
            {
                device_id: 'ESP_BOARD1_LIGHT_01',
                board_number: 1,
                type: 'light_sensor',
                name: 'Light Sensor Board',
                description: 'Cảm biến ánh sáng và điều khiển đèn LED',
                features: 'light_detection,led_control,mqtt_communication',
                status: 'offline',
                last_seen: null,
                online: false,
                created_at: new Date(),
                updated_at: new Date()
            },
            {
                device_id: 'ESP_BOARD2_SOIL_01',
                board_number: 2,
                type: 'soil_moisture',
                name: 'Soil Moisture Board',
                description: 'Cảm biến độ ẩm đất và điều khiển valve nước',
                features: 'soil_monitoring,water_valve_control,auto_watering',
                status: 'offline',
                last_seen: null,
                online: false,
                created_at: new Date(),
                updated_at: new Date()
            },
            {
                device_id: 'ESP_BOARD3_DHT22_01',
                board_number: 3,
                type: 'dht22',
                name: 'Temperature & Humidity Board',
                description: 'Cảm biến nhiệt độ và độ ẩm không khí',
                features: 'temperature_monitoring,humidity_monitoring,m2m_communication',
                status: 'offline',
                last_seen: null,
                online: false,
                created_at: new Date(),
                updated_at: new Date()
            },
            {
                device_id: 'ESP_BOARD4_NPK_01',
                board_number: 4,
                type: 'npk_sensor',
                name: 'NPK Sensor Board',
                description: 'Cảm biến NPK (Nitrogen, Phosphorus, Potassium)',
                features: 'npk_monitoring,soil_fertility,agricultural_sensors',
                status: 'offline',
                last_seen: null,
                online: false,
                created_at: new Date(),
                updated_at: new Date()
            },
            {
                device_id: 'ESP_BOARD5_PH_01',
                board_number: 5,
                type: 'ph_sensor',
                name: 'pH Sensor Board',
                description: 'Cảm biến pH và điều khiển dosing acid/alkaline',
                features: 'ph_monitoring,dual_servo_dosing,automatic_ph_control',
                status: 'offline',
                last_seen: null,
                online: false,
                created_at: new Date(),
                updated_at: new Date()
            },
            {
                device_id: 'ESP_BOARD6_ENV_01',
                board_number: 6,
                type: 'environment_monitor',
                name: 'Environmental Monitor Board',
                description: 'Giám sát môi trường tổng hợp với multiple sensors',
                features: 'multi_sensor,data_aggregation,i2c_spi_communication,multiple_modes',
                status: 'offline',
                last_seen: null,
                online: false,
                created_at: new Date(),
                updated_at: new Date()
            },
            {
                device_id: 'ESP_BOARD7_PH_01',
                board_number: 7,
                type: 'ph_sensor_control',
                name: 'pH Sensor Control Board',
                description: 'Cảm biến pH với LCD và calibration system',
                features: 'ph_monitoring,lcd_display,calibration_system,dual_servo',
                status: 'offline',
                last_seen: null,
                online: false,
                created_at: new Date(),
                updated_at: new Date()
            }
        ];
        
        await collection.insertMany(devices);
        console.log('   ✅ Đã tạo 7 thiết bị trong bảng devices');
    }

    /**
     * Tạo sensor_readings collection
     */
    async createSensorReadingsCollection() {
        const collection = this.db.collection('sensor_readings');
        await collection.createIndex({ sensor_type: 1 });
        await collection.createIndex({ device_id: 1 });
        await collection.createIndex({ timestamp: -1 });
        await collection.createIndex({ 'data.t': 1 });
        await collection.createIndex({ 'data.h': 1 });
        await collection.createIndex({ 'data.l': 1 });
        await collection.createIndex({ 'data.m': 1 });
        await collection.createIndex({ 'data.n_ppm': 1 });
        await collection.createIndex({ 'data.p_ppm': 1 });
        await collection.createIndex({ 'data.k_ppm': 1 });
        await collection.createIndex({ 'data.ph_value': 1 });
        
        console.log('   ✅ Đã tạo sensor_readings collection với indexes');
    }

    /**
     * Tạo device_status collection
     */
    async createDeviceStatusCollection() {
        const collection = this.db.collection('device_status');
        await collection.createIndex({ device_type: 1 }, { unique: true });
        await collection.createIndex({ last_update: -1 });
        await collection.createIndex({ online: 1 });
        await collection.createIndex({ status: 1 });
        
        // Insert initial status
        const deviceStatuses = [
            { device_type: 'light', status: 'OFFLINE', online: false, last_update: new Date(), created_at: new Date() },
            { device_type: 'soil_moisture', status: 'OFFLINE', online: false, last_update: new Date(), created_at: new Date() },
            { device_type: 'dht22', status: 'OFFLINE', online: false, last_update: new Date(), created_at: new Date() },
            { device_type: 'npk', status: 'OFFLINE', online: false, last_update: new Date(), created_at: new Date() },
            { device_type: 'ph', status: 'OFFLINE', online: false, last_update: new Date(), created_at: new Date() },
            { device_type: 'environment', status: 'OFFLINE', online: false, last_update: new Date(), created_at: new Date() }
        ];
        
        await collection.insertMany(deviceStatuses);
        console.log('   ✅ Đã tạo device_status collection');
    }

    /**
     * Tạo control_commands collection
     */
    async createControlCommandsCollection() {
        const collection = this.db.collection('control_commands');
        await collection.createIndex({ device_id: 1, timestamp: -1 });
        await collection.createIndex({ command_id: 1 }, { unique: true });
        await collection.createIndex({ status: 1 });
        await collection.createIndex({ timestamp: -1 });
        
        console.log('   ✅ Đã tạo control_commands collection');
    }

    /**
     * Tạo alerts collection
     */
    async createAlertsCollection() {
        const collection = this.db.collection('alerts');
        await collection.createIndex({ device_id: 1, timestamp: -1 });
        await collection.createIndex({ alert_type: 1 });
        await collection.createIndex({ severity: 1 });
        await collection.createIndex({ acknowledged: 1 });
        await collection.createIndex({ timestamp: -1 });
        
        console.log('   ✅ Đã tạo alerts collection');
    }

    /**
     * Tạo system_config collection
     */
    async createSystemConfigCollection() {
        const collection = this.db.collection('system_config');
        await collection.createIndex({ type: 1 }, { unique: true });
        await collection.createIndex({ updated_at: -1 });
        
        // Insert default configurations
        const configs = [
            {
                type: 'thresholds',
                data: {
                    light: { min: 100, max: 3000 },
                    soil_moisture: { min: 30, max: 80 },
                    temperature: { min: 18, max: 30 },
                    humidity: { min: 40, max: 80 },
                    ph: { min: 6.0, max: 8.0 },
                    npk: {
                        nitrogen: { min: 50, max: 200 },
                        phosphorus: { min: 20, max: 100 },
                        potassium: { min: 100, max: 300 }
                    }
                },
                created_at: new Date(),
                updated_at: new Date()
            },
            {
                type: 'automation',
                data: {
                    auto_watering: true,
                    auto_ph_control: true,
                    alerting_enabled: true,
                    data_retention_days: 30
                },
                created_at: new Date(),
                updated_at: new Date()
            },
            {
                type: 'system_settings',
                data: {
                    mqtt_broker: 'broker.hivemq.com',
                    mqtt_port: 1883,
                    api_port: 3001,
                    database_name: 'iot_smartgarden',
                    update_intervals: {
                        sensors: 15000,
                        status: 300000,
                        alerts: 5000
                    }
                },
                created_at: new Date(),
                updated_at: new Date()
            }
        ];
        
        await collection.insertMany(configs);
        console.log('   ✅ Đã tạo system_config collection với default configs');
    }

    /**
     * Tạo system_logs collection
     */
    async createSystemLogsCollection() {
        const collection = this.db.collection('system_logs');
        await collection.createIndex({ timestamp: -1 });
        await collection.createIndex({ level: 1 });
        await collection.createIndex({ source: 1 });
        await collection.createIndex({ message: 'text' });
        
        console.log('   ✅ Đã tạo system_logs collection');
    }

    /**
     * Tạo automation_rules collection
     */
    async createAutomationRulesCollection() {
        const collection = this.db.collection('automation_rules');
        await collection.createIndex({ rule_type: 1 });
        await collection.createIndex({ active: 1 });
        await collection.createIndex({ device_id: 1 });
        
        // Insert default automation rules
        const rules = [
            {
                rule_id: 'auto_watering_rule',
                rule_type: 'auto_watering',
                name: 'Tự động tưới nước',
                description: 'Tự động mở valve khi độ ẩm đất thấp',
                device_id: 'ESP_BOARD2_SOIL_01',
                condition: 'soil_moisture < 30',
                action: 'open_water_valve',
                duration: 5000,
                active: true,
                created_at: new Date(),
                updated_at: new Date()
            },
            {
                rule_id: 'ph_control_rule',
                rule_type: 'ph_control',
                name: 'Tự động điều chỉnh pH',
                description: 'Tự động dosing acid/alkaline khi pH không tối ưu',
                device_id: 'ESP_BOARD5_PH_01',
                condition: 'ph_value < 6.5 || ph_value > 7.5',
                action: 'dose_ph_adjuster',
                duration: 5000,
                active: true,
                created_at: new Date(),
                updated_at: new Date()
            }
        ];
        
        await collection.insertMany(rules);
        console.log('   ✅ Đã tạo automation_rules collection');
    }

    /**
     * Tạo sample data để test
     */
    async createSampleData() {
        try {
            console.log('📊 Tạo sample data...');

            // Sample sensor readings
            const sampleReadings = [
                {
                    topic: 'garden/sensor/light',
                    sensor_type: 'light',
                    timestamp: new Date(),
                    data: {
                        device_id: 'ESP_BOARD1_LIGHT_01',
                        light_level: 1250,
                        led_status: false,
                        rssi: -45
                    },
                    device_id: 'ESP_BOARD1_LIGHT_01',
                    processed: false
                },
                {
                    topic: 'garden/sensor/soil_moisture',
                    sensor_type: 'soil_moisture',
                    timestamp: new Date(),
                    data: {
                        device_id: 'ESP_BOARD2_SOIL_01',
                        moisture_percentage: 45.2,
                        valve_status: 'closed',
                        analog_raw: 1850
                    },
                    device_id: 'ESP_BOARD2_SOIL_01',
                    processed: false
                },
                {
                    topic: 'garden/sensor/dht22',
                    sensor_type: 'dht22',
                    timestamp: new Date(),
                    data: {
                        device_id: 'ESP_BOARD3_DHT22_01',
                        temperature: 24.5,
                        humidity: 65.8,
                        heat_index: 25.1
                    },
                    device_id: 'ESP_BOARD3_DHT22_01',
                    processed: false
                },
                {
                    topic: 'garden/sensor/npk',
                    sensor_type: 'npk',
                    timestamp: new Date(),
                    data: {
                        device_id: 'ESP_BOARD4_NPK_01',
                        n_ppm: 125.5,
                        p_ppm: 45.2,
                        k_ppm: 185.7,
                        quality: 'good'
                    },
                    device_id: 'ESP_BOARD4_NPK_01',
                    processed: false
                },
                {
                    topic: 'garden/sensor/ph',
                    sensor_type: 'ph',
                    timestamp: new Date(),
                    data: {
                        device_id: 'ESP_BOARD5_PH_01',
                        ph_value: 7.2,
                        status: 'optimal',
                        dosing_active: false
                    },
                    device_id: 'ESP_BOARD5_PH_01',
                    processed: false
                },
                {
                    topic: 'garden/sensor/environment',
                    sensor_type: 'environment',
                    timestamp: new Date(),
                    data: {
                        device_id: 'ESP_BOARD6_ENV_01',
                        temperature: 23.8,
                        humidity: 68.5,
                        light_level: 1150,
                        pressure: 1013.2,
                        soil_moisture: 42.1,
                        air_quality: 85.3
                    },
                    device_id: 'ESP_BOARD6_ENV_01',
                    processed: false
                }
            ];

            await this.db.collection('sensor_readings').insertMany(sampleReadings);
            console.log('   ✅ Đã tạo sample sensor readings');

            // Sample system log
            const sampleLog = {
                timestamp: new Date(),
                level: 'INFO',
                source: 'system_reset',
                message: 'Smart Garden System reset completed successfully',
                metadata: {
                    action: 'database_reset',
                    timestamp: new Date(),
                    affected_collections: ['devices', 'sensor_readings', 'device_status', 'control_commands', 'alerts', 'system_config', 'system_logs', 'automation_rules']
                }
            };

            await this.db.collection('system_logs').insertOne(sampleLog);
            console.log('   ✅ Đã tạo sample system log');

        } catch (error) {
            console.error('❌ Lỗi tạo sample data:', error);
        }
    }

    /**
     * Kiểm tra database status
     */
    async checkDatabaseStatus() {
        try {
            console.log('🔍 Kiểm tra trạng thái database...');
            
            const collections = await this.db.listCollections().toArray();
            console.log(`📊 Tổng số collections: ${collections.length}`);
            
            for (const collection of collections) {
                const count = await this.db.collection(collection.name).countDocuments();
                console.log(`   📁 ${collection.name}: ${count} documents`);
            }
            
            console.log('✅ Kiểm tra database status hoàn tất');
        } catch (error) {
            console.error('❌ Lỗi kiểm tra database:', error);
        }
    }

    /**
     * Disconnect từ database
     */
    async disconnect() {
        if (this.client) {
            await this.client.close();
            console.log('🔌 Đã ngắt kết nối MongoDB');
        }
    }
}

/**
 * Main function
 */
async function main() {
    const reset = new SystemReset();
    
    try {
        console.log('🔄 Bắt đầu reset Smart Garden System...');
        console.log('⏰ Thời gian:', new Date().toISOString());
        console.log('='.repeat(60));
        
        // Connect to database
        await reset.connect();
        
        // Drop existing collections
        await reset.dropExistingCollections();
        
        // Create new collections
        await reset.createCollections();
        
        // Create sample data
        await reset.createSampleData();
        
        // Check final status
        await reset.checkDatabaseStatus();
        
        console.log('='.repeat(60));
        console.log('🎉 Smart Garden System reset hoàn tất!');
        console.log('');
        console.log('📋 Tiếp theo bạn cần:');
        console.log('   1. Restart backend servers');
        console.log('   2. Test MQTT connections');
        console.log('   3. Kiểm tra WebSocket real-time');
        console.log('   4. Verify frontend data display');
        console.log('');
        console.log('🔧 Các lệnh để khởi động:');
        console.log('   cd backend && node api/server.js');
        console.log('   cd backend && node broker/server.js');
        console.log('   cd frontend/web && npm run dev');
        
    } catch (error) {
        console.error('❌ Lỗi trong quá trình reset:', error);
    } finally {
        await reset.disconnect();
    }
}

// Run if called directly
if (require.main === module) {
    main().catch(console.error);
}

module.exports = SystemReset;
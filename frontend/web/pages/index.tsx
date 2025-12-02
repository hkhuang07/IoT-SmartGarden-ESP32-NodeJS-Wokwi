import React, { useState, useEffect } from 'react';
import io from 'socket.io-client';
import axios from 'axios';
import { format } from 'date-fns';
import toast, { Toaster } from 'react-hot-toast';
import {
  SensorCard,
  DeviceStatusCard,
  AlertCard,
  ControlPanel,
  DashboardHeader,
  LoadingSpinner,
  EmptyState,
  StatCard,
  ModernProgressBar
} from '../components';

interface SensorData {
  light?: any;
  temp_humidity?: any;
  soil_moisture?: any;
  npk?: any;
  ph?: any;
}

interface DeviceStatus {
  device_type: string;
  status: string;
  last_update: string;
  online: boolean;
}

interface Alert {
  _id: string;
  topic: string;
  message: string;
  timestamp: string;
  severity: string;
  acknowledged: boolean;
}

const API_BASE_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:3001/api';

export default function SmartGardenDashboard() {
  const [sensorData, setSensorData] = useState<SensorData>({});
  const [deviceStatus, setDeviceStatus] = useState<DeviceStatus[]>([]);
  const [alerts, setAlerts] = useState<Alert[]>([]);
  const [loading, setLoading] = useState(true);
  const [selectedDevice, setSelectedDevice] = useState('');
  const [controlValue, setControlValue] = useState('');
  const [connectionStatus, setConnectionStatus] = useState('connecting');
  const [stats, setStats] = useState({
    totalDevices: 0,
    onlineDevices: 0,
    activeAlerts: 0,
    lastUpdate: null
  });

  // Initialize socket connection
  useEffect(() => {
    const socket = io('http://localhost:3001');
    
    socket.on('connect', () => {
      console.log('Connected to real-time data server');
      setConnectionStatus('connected');
      toast.success('🔌 Real-time connection established');
    });
    
    socket.on('disconnect', () => {
      setConnectionStatus('disconnected');
      toast.error('🔌 Connection lost');
    });

    socket.on('data', (data) => {
      console.log('Real-time data received:', data);
      if (data.topic?.startsWith('garden/sensor/')) {
        const sensorType = data.topic.split('/')[2];
        setSensorData(prev => ({
          ...prev,
          [sensorType]: JSON.parse(data.message)
        }));
        
        // Update stats
        setStats(prev => ({
          ...prev,
          lastUpdate: new Date()
        }));
      }
    });
    
    return () => socket.disconnect();
  }, []);

  // Fetch initial data
  useEffect(() => {
    fetchData();
    const interval = setInterval(fetchData, 30000); // Refresh every 30 seconds
    return () => clearInterval(interval);
  }, []);

  const fetchData = async () => {
    try {
      setLoading(true);
      const [sensorResponse, deviceResponse, alertResponse] = await Promise.all([
        axios.get(`${API_BASE_URL}/sensors/current`),
        axios.get(`${API_BASE_URL}/devices/status`),
        axios.get(`${API_BASE_URL}/alerts?limit=10`)
      ]);

      if (sensorResponse.data.success) {
        setSensorData(sensorResponse.data.data);
      }
      if (deviceResponse.data.success) {
        setDeviceStatus(deviceResponse.data.data);
        const onlineCount = deviceResponse.data.data.filter(d => d.online).length;
        setStats(prev => ({
          ...prev,
          totalDevices: deviceResponse.data.data.length,
          onlineDevices: onlineCount
        }));
      }
      if (alertResponse.data.success) {
        setAlerts(alertResponse.data.data);
        const activeCount = alertResponse.data.data.filter(a => !a.acknowledged).length;
        setStats(prev => ({
          ...prev,
          activeAlerts: activeCount
        }));
      }
    } catch (error) {
      console.error('Error fetching data:', error);
      toast.error('Failed to fetch data');
      setConnectionStatus('error');
    } finally {
      setLoading(false);
    }
  };

  const sendControlCommand = async (device: string, command: string, value?: string) => {
    try {
      const payload = value ? { value } : { command };
      const response = await axios.post(`${API_BASE_URL}/control/${device}`, payload);
      
      if (response.data.success) {
        toast.success(`🎯 Command sent to ${device.replace('_', ' ')}: ${command}`);
      } else {
        toast.error('❌ Failed to send command');
      }
    } catch (error) {
      console.error('Error sending command:', error);
      toast.error('❌ Error sending command');
    }
  };

  const acknowledgeAlert = async (alertId: string) => {
    try {
      const response = await axios.post(`${API_BASE_URL}/alerts/${alertId}/acknowledge`);
      if (response.data.success) {
        setAlerts(prev => prev.map(alert => 
          alert._id === alertId 
            ? { ...alert, acknowledged: true }
            : alert
        ));
        toast.success('✅ Alert acknowledged');
      }
    } catch (error) {
      console.error('Error acknowledging alert:', error);
      toast.error('❌ Failed to acknowledge alert');
    }
  };

  if (loading) {
    return <LoadingSpinner message="Loading Smart Garden Dashboard..." />;
  }

  return (
    <div className="min-h-screen bg-gradient-to-br from-green-50 via-emerald-50 to-teal-50">
      <Toaster 
        position="top-right"
        toastOptions={{
          duration: 3000,
          style: {
            background: '#fff',
            color: '#374151',
            borderRadius: '12px',
            boxShadow: '0 10px 25px -5px rgba(0, 0, 0, 0.1)',
            border: '1px solid rgba(34, 197, 94, 0.2)'
          },
          success: {
            iconTheme: {
              primary: '#22c55e',
              secondary: '#fff'
            }
          },
          error: {
            iconTheme: {
              primary: '#ef4444',
              secondary: '#fff'
            }
          }
        }}
      />
      
      {/* Header */}
      <DashboardHeader 
        deviceCount={stats.onlineDevices}
        onRefresh={fetchData}
        loading={loading}
      />

      {/* Main Content */}
      <main className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 pb-8">
        
        {/* Statistics Overview */}
        <div className="grid grid-cols-1 md:grid-cols-4 gap-6 mb-8">
          <StatCard
            title="Online Devices"
            value={`${stats.onlineDevices}/${stats.totalDevices}`}
            icon="📡"
            change={`${((stats.onlineDevices / stats.totalDevices) * 100).toFixed(0)}% uptime`}
            changeType="positive"
          />
          <StatCard
            title="Active Alerts"
            value={stats.activeAlerts}
            icon="🚨"
            change={stats.activeAlerts === 0 ? 'All good!' : 'Needs attention'}
            changeType={stats.activeAlerts === 0 ? 'positive' : 'negative'}
          />
          <StatCard
            title="Soil Moisture"
            value={`${sensorData.soil_moisture?.percentage?.toFixed(0) || 0}%`}
            icon="💧"
            change={sensorData.soil_moisture?.percentage > 60 ? 'Optimal' : 'Needs water'}
            changeType={sensorData.soil_moisture?.percentage > 60 ? 'positive' : 'negative'}
          />
          <StatCard
            title="pH Level"
            value={`${sensorData.ph?.value?.toFixed(1) || 0}`}
            icon="🔬"
            change={sensorData.ph?.suitability === 'optimal' ? 'Plant-friendly' : 
                   sensorData.ph?.suitability === 'moderate' ? 'Adjustable' : 'Critical'}
            changeType={sensorData.ph?.suitability === 'optimal' ? 'positive' : 
                       sensorData.ph?.suitability === 'moderate' ? 'neutral' : 'negative'}
          />
        </div>

        {/* Sensor Data Grid */}
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
          {/* Light Sensor */}
          <SensorCard
            title="Light Level"
            value={sensorData.light?.l || 0}
            unit=" lux"
            percentage={sensorData.light?.percentage || 0}
            type="light"
            icon="☀️"
          >
            <ModernProgressBar 
              value={sensorData.light?.percentage || 0}
              color="yellow"
              showPercentage={false}
            />
          </SensorCard>

          {/* Temperature & Humidity */}
          <SensorCard
            title="Temperature & Humidity"
            value={sensorData.temp_humidity?.t?.toFixed(1) || 0}
            unit="°C"
            type="temp"
            icon="🌡️"
          >
            <div className="space-y-3">
              <div>
                <div className="flex justify-between text-xs text-gray-500 mb-1">
                  <span>Temperature</span>
                  <span>{sensorData.temp_humidity?.t?.toFixed(1) || 0}°C</span>
                </div>
                <ModernProgressBar 
                  value={Math.min((sensorData.temp_humidity?.t || 0) / 40 * 100, 100)}
                  color="red"
                  showPercentage={false}
                />
              </div>
              <div>
                <div className="flex justify-between text-xs text-gray-500 mb-1">
                  <span>Humidity</span>
                  <span>{sensorData.temp_humidity?.h?.toFixed(1) || 0}%</span>
                </div>
                <ModernProgressBar 
                  value={sensorData.temp_humidity?.h || 0}
                  color="blue"
                  showPercentage={false}
                />
              </div>
            </div>
          </SensorCard>

          {/* Soil Moisture */}
          <SensorCard
            title="Soil Moisture"
            value={sensorData.soil_moisture?.m || 0}
            unit="%"
            percentage={sensorData.soil_moisture?.percentage || 0}
            type="soil"
            icon="💧"
          >
            <div className="flex items-center justify-between text-xs text-gray-500">
              <span>Status:</span>
              <span className={`font-medium ${
                (sensorData.soil_moisture?.percentage || 0) > 70 ? 'text-blue-600' :
                (sensorData.soil_moisture?.percentage || 0) > 30 ? 'text-green-600' : 'text-red-600'
              }`}>
                {(sensorData.soil_moisture?.percentage || 0) > 70 ? 'Wet' :
                 (sensorData.soil_moisture?.percentage || 0) > 30 ? 'Good' : 'Dry'}
              </span>
            </div>
          </SensorCard>

          {/* NPK Nutrients */}
          <SensorCard
            title="NPK Nutrients"
            value={`${sensorData.npk?.n_ppm?.toFixed(0) || 0}:${sensorData.npk?.p_ppm?.toFixed(0) || 0}:${sensorData.npk?.k_ppm?.toFixed(0) || 0}`}
            unit=" ppm"
            type="npk"
            icon="🧪"
          >
            <div className="space-y-2">
              <div className="flex justify-between text-xs">
                <span className="text-blue-600 font-medium">N (Nitrogen)</span>
                <span className="text-gray-600">{sensorData.npk?.n_ppm?.toFixed(1) || 0} ppm</span>
              </div>
              <div className="flex justify-between text-xs">
                <span className="text-green-600 font-medium">P (Phosphorus)</span>
                <span className="text-gray-600">{sensorData.npk?.p_ppm?.toFixed(1) || 0} ppm</span>
              </div>
              <div className="flex justify-between text-xs">
                <span className="text-yellow-600 font-medium">K (Potassium)</span>
                <span className="text-gray-600">{sensorData.npk?.k_ppm?.toFixed(1) || 0} ppm</span>
              </div>
            </div>
          </SensorCard>

          {/* pH Level */}
          <SensorCard
            title="pH Level"
            value={sensorData.ph?.value?.toFixed(1) || 0}
            unit=""
            percentage={sensorData.ph?.percent || 0}
            type="ph"
            icon="🔬"
          >
            <div className="space-y-2">
              <div className="flex justify-between text-xs">
                <span className="text-purple-600 font-medium">Status</span>
                <span className={`font-medium ${
                  sensorData.ph?.suitability === 'optimal' ? 'text-green-600' :
                  sensorData.ph?.suitability === 'moderate' ? 'text-yellow-600' : 'text-red-600'
                }`}>
                  {sensorData.ph?.suitability === 'optimal' ? 'Optimal' :
                   sensorData.ph?.suitability === 'moderate' ? 'Moderate' : 'Poor'}
                </span>
              </div>
              <div className="flex justify-between text-xs">
                <span className="text-purple-600 font-medium">Voltage</span>
                <span className="text-gray-600">{sensorData.ph?.voltage?.toFixed(2) || 0}V</span>
              </div>
              <div className="flex justify-between text-xs">
                <span className="text-purple-600 font-medium">Control</span>
                <span className="text-gray-600">
                  {sensorData.ph?.acidServo ? '🔴 Acid' : ''} 
                  {sensorData.ph?.alkalineServo ? ' 🔵 Alkaline' : ''}
                  {(!sensorData.ph?.acidServo && !sensorData.ph?.alkalineServo) ? '✅ Optimal' : ''}
                </span>
              </div>
            </div>
          </SensorCard>
        </div>

        {/* Control Panel */}
        <ControlPanel
          selectedDevice={selectedDevice}
          setSelectedDevice={setSelectedDevice}
          controlValue={controlValue}
          setControlValue={setControlValue}
          onSendCommand={sendControlCommand}
          className="mb-8"
        />

        {/* Device Status & Alerts Grid */}
        <div className="grid grid-cols-1 xl:grid-cols-2 gap-8">
          <DeviceStatusCard 
            devices={deviceStatus}
          />
          
          <AlertCard
            alerts={alerts}
            onAcknowledge={acknowledgeAlert}
          />
        </div>

        {/* Connection Status Indicator */}
        <div className="fixed bottom-6 right-6">
          <div className={`glass rounded-full p-4 shadow-lg ${
            connectionStatus === 'connected' ? 'bg-green-100 border-green-300' :
            connectionStatus === 'connecting' ? 'bg-yellow-100 border-yellow-300' :
            'bg-red-100 border-red-300'
          } border`}>
            <div className="flex items-center space-x-2">
              <div className={`w-3 h-3 rounded-full ${
                connectionStatus === 'connected' ? 'bg-green-500 animate-pulse' :
                connectionStatus === 'connecting' ? 'bg-yellow-500 animate-pulse' :
                'bg-red-500'
              }`}></div>
              <span className="text-sm font-medium text-gray-700">
                {connectionStatus === 'connected' ? 'Connected' :
                 connectionStatus === 'connecting' ? 'Connecting...' :
                 'Disconnected'}
              </span>
            </div>
          </div>
        </div>

        {/* Last Update Time */}
        {stats.lastUpdate && (
          <div className="text-center mt-8 text-sm text-gray-500">
            <p>Last updated: {format(stats.lastUpdate, 'PPpp')}</p>
            <p className="text-xs">Auto-refresh every 30 seconds</p>
          </div>
        )}
      </main>
    </div>
  );
}
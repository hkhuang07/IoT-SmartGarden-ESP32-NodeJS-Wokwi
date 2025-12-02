// Modern React Components for Smart Garden Dashboard
// Agricultural theme with beautiful UI components

import React from 'react';

// ===== SENSOR CARD COMPONENT =====
export const SensorCard = ({ 
  title, 
  value, 
  unit = '', 
  percentage = 0, 
  type = 'soil',
  icon,
  className = '',
  children 
}) => {
  const getGradientClass = (type) => {
    switch (type) {
      case 'light': return 'bg-gradient-to-br from-yellow-400 to-orange-500';
      case 'temp': return 'bg-gradient-to-br from-red-400 to-red-600';
      case 'soil': return 'bg-gradient-to-br from-green-400 to-green-600';
      case 'npk': return 'bg-gradient-to-br from-lime-400 to-green-500';
      case 'ph': return 'bg-gradient-to-br from-purple-400 to-pink-500';
      default: return 'bg-gradient-to-br from-green-400 to-green-600';
    }
  };

  const getValueColor = (type) => {
    switch (type) {
      case 'light': return 'text-yellow-600';
      case 'temp': return 'text-red-600';
      case 'soil': return 'text-green-600';
      case 'npk': return 'text-lime-600';
      case 'ph': return 'text-purple-600';
      default: return 'text-green-600';
    }
  };

  const getProgressColor = (type) => {
    switch (type) {
      case 'light': return 'bg-gradient-to-r from-yellow-400 to-orange-500';
      case 'temp': return 'bg-gradient-to-r from-blue-400 to-red-500';
      case 'soil': return 'bg-gradient-to-r from-green-400 to-green-600';
      case 'npk': return 'bg-gradient-to-r from-lime-400 to-green-500';
      case 'ph': return 'bg-gradient-to-r from-purple-400 to-pink-500';
      default: return 'bg-gradient-to-r from-green-400 to-green-600';
    }
  };

  return (
    <div className={`card-modern sensor-card ${type} ${className} fade-in-up`}>
      {/* Header with icon */}
      <div className="flex items-center justify-between mb-4">
        <div className="flex-1">
          <h3 className="sensor-title">{title}</h3>
          <div className={`text-3xl font-bold ${getValueColor(type)} mb-1`}>
            {value}{unit}
          </div>
          {percentage !== undefined && (
            <p className="text-sm text-gray-500 font-medium">
              {percentage.toFixed(1)}%
            </p>
          )}
        </div>
        <div className={`sensor-icon ${type} ${getGradientClass(type)}`}>
          {icon}
        </div>
      </div>

      {/* Progress bar */}
      {percentage !== undefined && (
        <div className="progress-bar">
          <div 
            className={`progress-fill ${getProgressColor(type)}`}
            style={{ width: `${Math.min(Math.max(percentage, 0), 100)}%` }}
          />
        </div>
      )}

      {/* Additional content */}
      {children && (
        <div className="mt-4">
          {children}
        </div>
      )}
    </div>
  );
};

// ===== DEVICE STATUS COMPONENT =====
export const DeviceStatusCard = ({ devices, className = '' }) => {
  const onlineCount = devices.filter(d => d.online).length;
  const totalCount = devices.length;

  return (
    <div className={`card-modern control-panel ${className}`}>
      <h2 className="control-title">
        📡 Device Status
      </h2>
      
      {/* Status Summary */}
      <div className="flex items-center justify-between p-4 bg-gradient-to-r from-green-50 to-emerald-50 rounded-xl mb-6 border border-green-200">
        <div className="flex items-center">
          <div className="device-status-indicator online"></div>
          <div>
            <p className="text-lg font-bold text-green-700">
              {onlineCount} / {totalCount} Devices Online
            </p>
            <p className="text-sm text-green-600">
              System Status: {onlineCount === totalCount ? 'Excellent' : onlineCount > totalCount * 0.7 ? 'Good' : 'Needs Attention'}
            </p>
          </div>
        </div>
        <div className="text-3xl">
          {onlineCount === totalCount ? '🌱' : onlineCount > totalCount * 0.7 ? '🌿' : '⚠️'}
        </div>
      </div>

      {/* Device List */}
      <div className="space-y-3 max-h-80 overflow-y-auto pr-2">
        {devices.map((device, index) => (
          <div key={index} className="device-item slide-in-right" style={{ animationDelay: `${index * 0.1}s` }}>
            <div className="flex items-center justify-between">
              <div className="flex items-center">
                <div className={`device-status-indicator ${device.online ? 'online' : 'offline'}`}></div>
                <div>
                  <p className="font-semibold text-gray-900 capitalize">
                    {device.device_type.replace('_', ' ')}
                  </p>
                  <p className="text-sm text-gray-500">
                    Last update: {new Date(device.last_update).toLocaleTimeString()}
                  </p>
                </div>
              </div>
              <div className="text-right">
                <span className={`inline-flex px-3 py-1 rounded-full text-xs font-medium ${
                  device.status === 'ONLINE' 
                    ? 'bg-green-100 text-green-800' 
                    : 'bg-red-100 text-red-800'
                }`}>
                  {device.status}
                </span>
              </div>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};

// ===== ALERT COMPONENT =====
export const AlertCard = ({ alerts, onAcknowledge, className = '' }) => {
  const getSeverityConfig = (severity) => {
    switch (severity) {
      case 'CRITICAL':
        return {
          color: 'red',
          bgColor: 'bg-red-50',
          borderColor: 'border-red-200',
          textColor: 'text-red-800',
          dotColor: 'bg-red-500'
        };
      case 'WARNING':
        return {
          color: 'yellow',
          bgColor: 'bg-yellow-50',
          borderColor: 'border-yellow-200',
          textColor: 'text-yellow-800',
          dotColor: 'bg-yellow-500'
        };
      default:
        return {
          color: 'blue',
          bgColor: 'bg-blue-50',
          borderColor: 'border-blue-200',
          textColor: 'text-blue-800',
          dotColor: 'bg-blue-500'
        };
    }
  };

  const unacknowledgedAlerts = alerts.filter(alert => !alert.acknowledged);
  const acknowledgedAlerts = alerts.filter(alert => alert.acknowledged);

  return (
    <div className={`card-modern control-panel ${className}`}>
      <h2 className="control-title">
        🚨 Alerts & Notifications
      </h2>

      {/* Alert Summary */}
      <div className="flex items-center justify-between p-4 bg-gradient-to-r from-orange-50 to-red-50 rounded-xl mb-6 border border-orange-200">
        <div className="flex items-center">
          <div className={`w-4 h-4 rounded-full ${unacknowledgedAlerts.length > 0 ? 'bg-red-500 animate-pulse' : 'bg-green-500'} mr-3`}></div>
          <div>
            <p className="text-lg font-bold text-orange-700">
              {unacknowledgedAlerts.length} Active Alerts
            </p>
            <p className="text-sm text-orange-600">
              Total: {alerts.length} alerts
            </p>
          </div>
        </div>
        <div className="text-3xl">
          {unacknowledgedAlerts.length > 0 ? '⚠️' : '✅'}
        </div>
      </div>

      {/* Alerts List */}
      <div className="space-y-3 max-h-80 overflow-y-auto pr-2">
        {alerts.length === 0 ? (
          <div className="text-center py-8 text-gray-500">
            <div className="text-4xl mb-2">🌱</div>
            <p className="font-medium">No alerts at the moment</p>
            <p className="text-sm">Your garden is running smoothly!</p>
          </div>
        ) : (
          <>
            {/* Unacknowledged Alerts */}
            {unacknowledgedAlerts.length > 0 && (
              <div>
                <h4 className="text-sm font-semibold text-gray-700 mb-2 uppercase tracking-wide">
                  Active Alerts
                </h4>
                {unacknowledgedAlerts.map((alert, index) => {
                  const config = getSeverityConfig(alert.severity);
                  return (
                    <div key={alert._id || index} className={`alert-item ${config.color.toLowerCase()}`}>
                      <div className="flex items-start justify-between">
                        <div className="flex-1">
                          <div className="flex items-center mb-2">
                            <span className={`alert-severity ${config.dotColor.replace('bg-', '')}`}></span>
                            <p className="font-medium text-gray-900 text-sm">
                              {alert.topic}
                            </p>
                          </div>
                          <p className="text-gray-600 text-sm mb-2">
                            {alert.message}
                          </p>
                          <p className="text-xs text-gray-500">
                            {new Date(alert.timestamp).toLocaleString()}
                          </p>
                        </div>
                        <button
                          onClick={() => onAcknowledge(alert._id)}
                          className="ml-4 btn-modern text-xs px-3 py-1"
                        >
                          Acknowledge
                        </button>
                      </div>
                    </div>
                  );
                })}
              </div>
            )}

            {/* Acknowledged Alerts */}
            {acknowledgedAlerts.length > 0 && (
              <div className="mt-6">
                <h4 className="text-sm font-semibold text-gray-700 mb-2 uppercase tracking-wide">
                  Resolved Alerts
                </h4>
                {acknowledgedAlerts.slice(0, 3).map((alert, index) => (
                  <div key={alert._id || `ack-${index}`} className="alert-item info opacity-60">
                    <div className="flex items-center">
                      <div className="alert-severity bg-gray-400"></div>
                      <div className="flex-1">
                        <p className="text-sm text-gray-700">{alert.topic}</p>
                        <p className="text-xs text-gray-500">
                          {new Date(alert.timestamp).toLocaleString()}
                        </p>
                      </div>
                      <span className="text-xs text-gray-500">✓ Resolved</span>
                    </div>
                  </div>
                ))}
              </div>
            )}
          </>
        )}
      </div>
    </div>
  );
};

// ===== CONTROL PANEL COMPONENT =====
export const ControlPanel = ({ 
  selectedDevice, 
  setSelectedDevice, 
  controlValue, 
  setControlValue, 
  onSendCommand,
  className = '' 
}) => {
  const devices = [
    { value: 'light', label: '💡 Light Control', description: 'Control grow lights' },
    { value: 'water_valve', label: '🚿 Water Valve', description: 'Control irrigation valve' },
    { value: 'roof_servo', label: '🏠 Roof Servo', description: 'Control greenhouse roof' },
    { value: 'pump', label: '💧 Pump Control', description: 'Control water pump' },
    { value: 'ventilation', label: '🌬️ Ventilation', description: 'Control ventilation system' }
  ];

  const quickCommands = {
    light: ['ON', 'OFF', '50%', '75%', '100%'],
    water_valve: ['OPEN', 'CLOSE', 'PARTIAL'],
    roof_servo: ['OPEN', 'CLOSE', 'HALF'],
    pump: ['START', 'STOP', 'PULSE'],
    ventilation: ['ON', 'OFF', 'HIGH', 'LOW']
  };

  return (
    <div className={`card-modern control-panel ${className}`}>
      <h2 className="control-title">
        🎛️ Control Panel
      </h2>
      
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Device Selection */}
        <div className="form-group">
          <label className="form-label">Select Device</label>
          <select
            value={selectedDevice}
            onChange={(e) => setSelectedDevice(e.target.value)}
            className="form-control"
          >
            <option value="">Choose device...</option>
            {devices.map(device => (
              <option key={device.value} value={device.value}>
                {device.label}
              </option>
            ))}
          </select>
          {selectedDevice && (
            <p className="text-xs text-gray-500 mt-1">
              {devices.find(d => d.value === selectedDevice)?.description}
            </p>
          )}
        </div>

        {/* Command Input */}
        <div className="form-group">
          <label className="form-label">Command / Value</label>
          <input
            type="text"
            value={controlValue}
            onChange={(e) => setControlValue(e.target.value)}
            placeholder="Enter command..."
            className="form-control"
            disabled={!selectedDevice}
          />
          
          {/* Quick Commands */}
          {selectedDevice && quickCommands[selectedDevice] && (
            <div className="flex flex-wrap gap-1 mt-2">
              {quickCommands[selectedDevice].map(cmd => (
                <button
                  key={cmd}
                  type="button"
                  onClick={() => setControlValue(cmd)}
                  className="px-2 py-1 text-xs bg-gray-100 hover:bg-gray-200 rounded-md transition-colors"
                >
                  {cmd}
                </button>
              ))}
            </div>
          )}
        </div>

        {/* Send Button */}
        <div className="flex items-end">
          <button
            onClick={() => selectedDevice && onSendCommand(selectedDevice, controlValue || 'TOGGLE', controlValue)}
            disabled={!selectedDevice}
            className="btn-primary w-full disabled:opacity-50 disabled:cursor-not-allowed"
          >
            Send Command
          </button>
        </div>
      </div>
    </div>
  );
};

// ===== DASHBOARD HEADER COMPONENT =====
export const DashboardHeader = ({ deviceCount, onRefresh, loading = false, className = '' }) => {
  return (
    <div className={`dashboard-header ${className}`}>
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
        <div className="flex flex-col lg:flex-row lg:items-center lg:justify-between">
          <div className="mb-4 lg:mb-0">
            <h1 className="dashboard-title">
              🌱 Smart Garden IoT Dashboard
            </h1>
            <p className="dashboard-subtitle">
              Monitor and control your intelligent garden system
            </p>
          </div>
          
          <div className="flex flex-col sm:flex-row items-start sm:items-center gap-4">
            {/* Device Status */}
            <div className="flex items-center">
              <div className="device-status-indicator online"></div>
              <span className="text-white font-semibold">
                {deviceCount} Devices Online
              </span>
            </div>
            
            {/* Refresh Button */}
            <button
              onClick={onRefresh}
              disabled={loading}
              className="btn-secondary disabled:opacity-50"
            >
              {loading ? (
                <div className="loading-spinner w-5 h-5 mr-2"></div>
              ) : (
                <span className="mr-2">🔄</span>
              )}
              {loading ? 'Refreshing...' : 'Refresh'}
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

// ===== LOADING COMPONENT =====
export const LoadingSpinner = ({ message = "Loading Smart Garden Dashboard...", className = '' }) => {
  return (
    <div className={`min-h-screen bg-gradient-to-br from-green-50 via-emerald-50 to-teal-50 flex items-center justify-center ${className}`}>
      <div className="text-center">
        <div className="loading-spinner w-12 h-12 mx-auto mb-4"></div>
        <p className="text-xl text-gray-700 font-semibold">{message}</p>
        <p className="text-sm text-gray-500 mt-2">Setting up your garden dashboard...</p>
      </div>
    </div>
  );
};

// ===== EMPTY STATE COMPONENT =====
export const EmptyState = ({ 
  icon = "🌱", 
  title = "No Data Available", 
  description = "Waiting for sensor data...",
  action = null,
  className = '' 
}) => {
  return (
    <div className={`text-center py-12 ${className}`}>
      <div className="text-6xl mb-4 floating">{icon}</div>
      <h3 className="text-xl font-semibold text-gray-900 mb-2">{title}</h3>
      <p className="text-gray-600 mb-4">{description}</p>
      {action && <div>{action}</div>}
    </div>
  );
};

// ===== STATISTICS COMPONENT =====
export const StatCard = ({ 
  title, 
  value, 
  icon, 
  change, 
  changeType = 'positive', 
  className = '' 
}) => {
  return (
    <div className={`card-modern ${className}`}>
      <div className="flex items-center justify-between">
        <div>
          <p className="text-sm font-medium text-gray-600">{title}</p>
          <p className="text-2xl font-bold text-gray-900">{value}</p>
          {change && (
            <p className={`text-xs ${changeType === 'positive' ? 'text-green-600' : 'text-red-600'}`}>
              {changeType === 'positive' ? '↗️' : '↘️'} {change}
            </p>
          )}
        </div>
        <div className="text-2xl opacity-80">{icon}</div>
      </div>
    </div>
  );
};

// ===== MODERN PROGRESS BAR COMPONENT =====
export const ModernProgressBar = ({ 
  value, 
  max = 100, 
  color = 'green', 
  showPercentage = true, 
  animated = true,
  className = '' 
}) => {
  const percentage = Math.min(Math.max((value / max) * 100, 0), 100);
  
  const getColorClass = (color) => {
    switch (color) {
      case 'yellow': return 'bg-gradient-to-r from-yellow-400 to-orange-500';
      case 'red': return 'bg-gradient-to-r from-red-400 to-red-600';
      case 'blue': return 'bg-gradient-to-r from-blue-400 to-blue-600';
      case 'green': default: return 'bg-gradient-to-r from-green-400 to-green-600';
    }
  };

  return (
    <div className={`progress-bar ${className}`}>
      <div 
        className={`progress-fill ${getColorClass(color)} ${animated ? 'animate-pulse' : ''}`}
        style={{ width: `${percentage}%` }}
      />
      {showPercentage && (
        <div className="text-right text-xs text-gray-500 mt-1">
          {percentage.toFixed(1)}%
        </div>
      )}
    </div>
  );
};
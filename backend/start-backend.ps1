# 🚀 Smart Garden Backend - Quick Start Script
# Script này sẽ khởi động cả MQTT Broker và API Server

Write-Host "🌱 Smart Garden Backend - Quick Start" -ForegroundColor Green
Write-Host "====================================" -ForegroundColor Green

# Check if we're in the backend directory
$currentDir = Get-Location
if ($currentDir -notlike "*backend") {
    Write-Host "⚠️  Please run this script from the backend directory" -ForegroundColor Yellow
    Write-Host "   Run: cd backend" -ForegroundColor Yellow
    exit 1
}

Write-Host "📍 Current directory: $currentDir" -ForegroundColor Cyan

# Check if node_modules exists
if (!(Test-Path "node_modules")) {
    Write-Host "📦 Installing backend dependencies..." -ForegroundColor Yellow
    npm install
    if ($LASTEXITCODE -ne 0) {
        Write-Host "❌ Failed to install dependencies" -ForegroundColor Red
        exit 1
    }
    Write-Host "✅ Backend dependencies installed" -ForegroundColor Green
} else {
    Write-Host "✅ Backend dependencies already installed" -ForegroundColor Green
}

# Check environment file
if (!(Test-Path ".env")) {
    Write-Host "⚠️  Creating .env file..." -ForegroundColor Yellow
    @"
# MongoDB Configuration
MONGODB_URI=mongodb+srv://iot_smartgarden:IoT_Smart_Garden@cluster0.17bgl.mongodb.net/?appName=Cluster0
DATABASE_NAME=iot_smartgarden

# MQTT Configuration
MQTT_BROKER=mqtt://broker.hivemq.com
MQTT_PORT=1883
MQTT_USERNAME=
MQTT_PASSWORD=

# API Configuration
API_PORT=3001
CORS_ORIGIN=http://localhost:3000

# Automation Thresholds
SOIL_MOISTURE_CRITICAL=30
SOIL_MOISTURE_LOW=40
SOIL_MOISTURE_OPTIMAL=60
TEMPERATURE_MIN=18
TEMPERATURE_MAX=35
HUMIDITY_MIN=40
HUMIDITY_MAX=80
LIGHT_MIN=300
LIGHT_MAX=900

# Server Configuration
LOG_LEVEL=info
MAX_CONNECTIONS=100
"@ | Out-File -FilePath ".env" -Encoding UTF8
    Write-Host "✅ .env file created" -ForegroundColor Green
} else {
    Write-Host "✅ .env file found" -ForegroundColor Green
}

# Check if required files exist
$requiredFiles = @(
    "broker\server.js",
    "broker\db_connection.js", 
    "broker\automation.js",
    "api\server.js"
)

$missingFiles = @()
foreach ($file in $requiredFiles) {
    if (!(Test-Path $file)) {
        $missingFiles += $file
    }
}

if ($missingFiles.Count -gt 0) {
    Write-Host "❌ Missing required files:" -ForegroundColor Red
    foreach ($file in $missingFiles) {
        Write-Host "   - $file" -ForegroundColor Red
    }
    exit 1
} else {
    Write-Host "✅ All required backend files found" -ForegroundColor Green
}

Write-Host "`n🚀 Starting Smart Garden Backend Services..." -ForegroundColor Green
Write-Host "===========================================" -ForegroundColor Green

# Function to check if port is available
function Test-Port {
    param([int]$PortNumber)
    try {
        $connection = New-Object System.Net.Sockets.TcpClient
        $connection.Connect("127.0.0.1", $PortNumber)
        $connection.Close()
        return $false  # Port is in use
    } catch {
        return $true   # Port is available
    }
}

# Check ports
$brokerPort = 3002
$apiPort = 3001

Write-Host "`n🔍 Checking port availability..." -ForegroundColor Cyan
if (!(Test-Port $apiPort)) {
    Write-Host "⚠️  Port $apiPort is in use (API Server)" -ForegroundColor Yellow
    Write-Host "   Please stop the existing process or use a different port" -ForegroundColor Yellow
} else {
    Write-Host "✅ Port $apiPort is available (API Server)" -ForegroundColor Green
}

if (!(Test-Port $brokerPort)) {
    Write-Host "⚠️  Port $brokerPort is in use (MQTT Broker)" -ForegroundColor Yellow
    Write-Host "   Please stop the existing process or use a different port" -ForegroundColor Yellow
} else {
    Write-Host "✅ Port $brokerPort is available (MQTT Broker)" -ForegroundColor Green
}

Write-Host "`n📊 Backend Services Status:" -ForegroundColor Cyan
Write-Host "   🔌 MQTT Broker:  http://localhost:$brokerPort" -ForegroundColor White
Write-Host "   🌐 API Server:   http://localhost:$apiPort/api" -ForegroundColor White
Write-Host "   📡 WebSocket:    ws://localhost:$apiPort" -ForegroundColor White

Write-Host "`n⌨️  Starting services..." -ForegroundColor Green
Write-Host "   Terminal 1: MQTT Broker (npm run start:broker)" -ForegroundColor Yellow
Write-Host "   Terminal 2: API Server (npm run start:api)" -ForegroundColor Yellow
Write-Host "   Terminal 3: Frontend (npm run dev)" -ForegroundColor Yellow

Write-Host "`n🎯 Test URLs:" -ForegroundColor Cyan
Write-Host "   Health Check:    http://localhost:$apiPort/api/health" -ForegroundColor White
Write-Host "   Sensor Data:     http://localhost:$apiPort/api/sensors/current" -ForegroundColor White
Write-Host "   Device Status:   http://localhost:$apiPort/api/devices/status" -ForegroundColor White

Write-Host "`n📋 Manual Startup Instructions:" -ForegroundColor Yellow
Write-Host "Open 3 separate terminals and run:" -ForegroundColor White
Write-Host "" -ForegroundColor White
Write-Host "Terminal 1 (MQTT Broker):" -ForegroundColor Cyan
Write-Host "  npm run start:broker" -ForegroundColor White
Write-Host "" -ForegroundColor White
Write-Host "Terminal 2 (API Server):" -ForegroundColor Cyan
Write-Host "  npm run start:api" -ForegroundColor White
Write-Host "" -ForegroundColor White
Write-Host "Terminal 3 (Frontend):" -ForegroundColor Cyan
Write-Host "  cd ..\frontend\web" -ForegroundColor White
Write-Host "  npm run dev" -ForegroundColor White
Write-Host "" -ForegroundColor White
Write-Host "===========================================" -ForegroundColor Green

# Ask if user wants to start services
Write-Host "`n❓ Would you like to auto-start backend services? (y/n)" -ForegroundColor Yellow
$response = Read-Host
if ($response -eq 'y' -or $response -eq 'Y') {
    Write-Host "`n🚀 Starting backend services in background..." -ForegroundColor Green
    
    try {
        # Start MQTT Broker
        Write-Host "📡 Starting MQTT Broker..." -ForegroundColor Cyan
        $brokerProcess = Start-Process -FilePath "npm" -ArgumentList "run", "start:broker" -WindowStyle Normal -PassThru
        Start-Sleep 3
        
        # Start API Server
        Write-Host "🌐 Starting API Server..." -ForegroundColor Cyan  
        $apiProcess = Start-Process -FilePath "npm" -ArgumentList "run", "start:api" -WindowStyle Normal -PassThru
        Start-Sleep 3
        
        Write-Host "`n✅ Backend services started!" -ForegroundColor Green
        Write-Host "📊 MQTT Broker PID: $($brokerProcess.Id)" -ForegroundColor White
        Write-Host "🌐 API Server PID: $($apiProcess.Id)" -ForegroundColor White
        
        Write-Host "`n📱 Next steps:" -ForegroundColor Yellow
        Write-Host "   1. Start frontend: cd ..\frontend\web; npm run dev" -ForegroundColor White
        Write-Host "   2. Open browser: http://localhost:3000" -ForegroundColor White
        
    } catch {
        Write-Host "❌ Error starting services: $($_.Exception.Message)" -ForegroundColor Red
    }
} else {
    Write-Host "`n👋 Backend setup complete. Please follow manual startup instructions above." -ForegroundColor Yellow
}

Write-Host "`n🌱 Happy gardening! 🪴" -ForegroundColor Green
# 🚀 Smart Garden Frontend - Quick Start Script
# Script này sẽ clear cache và restart frontend

Write-Host "🌱 Smart Garden Frontend - Quick Start" -ForegroundColor Green
Write-Host "=====================================" -ForegroundColor Green

# Check if we're in the frontend/web directory
$currentDir = Get-Location
if ($currentDir -notlike "*frontend\web") {
    Write-Host "⚠️  Please run this script from the frontend/web directory" -ForegroundColor Yellow
    Write-Host "   Run: cd frontend\web" -ForegroundColor Yellow
    exit 1
}

Write-Host "📍 Current directory: $currentDir" -ForegroundColor Cyan

# Check if node_modules exists
if (!(Test-Path "node_modules")) {
    Write-Host "📦 Installing dependencies..." -ForegroundColor Yellow
    npm install
    if ($LASTEXITCODE -ne 0) {
        Write-Host "❌ Failed to install dependencies" -ForegroundColor Red
        exit 1
    }
    Write-Host "✅ Dependencies installed" -ForegroundColor Green
} else {
    Write-Host "✅ Dependencies already installed" -ForegroundColor Green
}

# Check environment files
if (!(Test-Path ".env.local")) {
    Write-Host "⚠️  Creating .env.local..." -ForegroundColor Yellow
    @"
NEXT_PUBLIC_API_URL=http://localhost:3001/api
NEXT_PUBLIC_WS_URL=http://localhost:3001
"@ | Out-File -FilePath ".env.local" -Encoding UTF8
    Write-Host "✅ .env.local created" -ForegroundColor Green
} else {
    Write-Host "✅ .env.local found" -ForegroundColor Green
}

# Clear Next.js cache
Write-Host "`n🧹 Clearing Next.js cache..." -ForegroundColor Cyan
if (Test-Path ".next") {
    Remove-Item -Recurse -Force .\.next
    Write-Host "✅ Next.js cache cleared" -ForegroundColor Green
} else {
    Write-Host "✅ No cache to clear" -ForegroundColor Green
}

# Clear webpack cache if exists
$webpackCacheDir = ".next\cache\webpack"
if (Test-Path $webpackCacheDir) {
    Write-Host "🧹 Clearing webpack cache..." -ForegroundColor Cyan
    Remove-Item -Recurse -Force $webpackCacheDir
    Write-Host "✅ Webpack cache cleared" -ForegroundColor Green
}

Write-Host "`n🚀 Starting Next.js development server..." -ForegroundColor Green
Write-Host "🌐 Dashboard will be available at: http://localhost:3000" -ForegroundColor Yellow
Write-Host "📊 Make sure backend servers are running:" -ForegroundColor Yellow
Write-Host "   - MQTT Broker: npm run start:broker" -ForegroundColor White
Write-Host "   - API Server: npm run start:api" -ForegroundColor White

Write-Host "`n⌨️  Press Ctrl+C to stop the server" -ForegroundColor Cyan
Write-Host "=====================================" -ForegroundColor Green

# Start Next.js
npm run dev
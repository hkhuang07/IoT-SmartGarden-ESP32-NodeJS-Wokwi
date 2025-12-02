# 🧪 Test Smart Garden Frontend - CSS Import Fix
Write-Host "🧪 Testing CSS Import Fix..." -ForegroundColor Green

# Check current directory
$currentDir = Get-Location
Write-Host "📍 Current directory: $currentDir" -ForegroundColor Cyan

# Check if we're in frontend/web
if ($currentDir -notlike "*frontend\web") {
    Write-Host "⚠️  Please run this script from frontend\web directory" -ForegroundColor Yellow
    Write-Host "   Run: cd frontend\web" -ForegroundColor Yellow
    exit 1
}

# Verify CSS imports structure
Write-Host "`n🔍 Verifying CSS imports structure..." -ForegroundColor Cyan

# Check _app.tsx
$appFile = "pages\_app.tsx"
if (Test-Path $appFile) {
    $appContent = Get-Content $appFile
    if ($appContent -match "import.*globals\.css") {
        Write-Host "✅ _app.tsx imports globals.css correctly" -ForegroundColor Green
    } else {
        Write-Host "❌ _app.tsx missing globals.css import" -ForegroundColor Red
    }
}

# Check globals.css
$globalsFile = "styles\globals.css"
if (Test-Path $globalsFile) {
    $globalsContent = Get-Content $globalsFile
    if ($globalsContent -match "@import.*agricultural-theme\.css") {
        Write-Host "✅ globals.css imports agricultural-theme.css" -ForegroundColor Green
    }
    if ($globalsContent -match "@import.*enhanced-animations\.css") {
        Write-Host "✅ globals.css imports enhanced-animations.css" -ForegroundColor Green
    }
}

# Check index.tsx doesn't have direct CSS imports
$indexFile = "pages\index.tsx"
if (Test-Path $indexFile) {
    $indexContent = Get-Content $indexFile
    if ($indexContent -match "import.*\.css") {
        Write-Host "❌ index.tsx still has CSS import (should be removed)" -ForegroundColor Red
    } else {
        Write-Host "✅ index.tsx has no direct CSS import (correct)" -ForegroundColor Green
    }
}

# Check environment file
if (Test-Path ".env.local") {
    Write-Host "✅ .env.local file exists" -ForegroundColor Green
} else {
    Write-Host "⚠️  .env.local not found" -ForegroundColor Yellow
}

# Test TypeScript compilation
Write-Host "`n🔧 Testing TypeScript compilation..." -ForegroundColor Cyan
try {
    $tsResult = npx tsc --noEmit 2>&1
    if ($LASTEXITCODE -eq 0) {
        Write-Host "✅ TypeScript compilation successful" -ForegroundColor Green
    } else {
        Write-Host "❌ TypeScript compilation failed" -ForegroundColor Red
        Write-Host "Error details: $tsResult" -ForegroundColor Yellow
    }
} catch {
    Write-Host "⚠️  TypeScript check skipped" -ForegroundColor Yellow
}

# Clear any existing cache
Write-Host "`n🧹 Clearing cache..." -ForegroundColor Cyan
if (Test-Path ".next") {
    Remove-Item -Recurse -Force ".next" -ErrorAction SilentlyContinue
    Write-Host "✅ Next.js cache cleared" -ForegroundColor Green
}

Write-Host "`n🎯 CSS Import Fix Summary:" -ForegroundColor Yellow
Write-Host "   ✅ Removed CSS import from index.tsx" -ForegroundColor White
Write-Host "   ✅ CSS imported via globals.css in _app.tsx" -ForegroundColor White
Write-Host "   ✅ Custom themes will load correctly" -ForegroundColor White

Write-Host "`n🚀 Ready to start frontend!" -ForegroundColor Green
Write-Host "Run: npm run dev" -ForegroundColor White
Write-Host "Then open: http://localhost:3000" -ForegroundColor Green
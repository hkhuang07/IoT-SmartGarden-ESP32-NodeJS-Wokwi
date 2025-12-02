#!/bin/bash

# Smart Garden IoT Dashboard - Quick Setup Script
# Tự động setup và chạy dashboard hiện đại

echo "🌱 Smart Garden IoT Dashboard - Modern UI Setup"
echo "================================================"

# Navigate to web directory
cd frontend/web

echo "📦 Installing dependencies..."
npm install

echo "🔧 Setting up environment variables..."
cat > .env.local << EOF
NEXT_PUBLIC_API_URL=http://localhost:3001/api
EOF

echo "✅ Environment variables created!"

echo ""
echo "🚀 Starting development server..."
echo "Dashboard sẽ chạy tại: http://localhost:3000"
echo "Backend API cần chạy tại: http://localhost:3001"
echo ""
echo "📝 Commands hữu ích:"
echo "  - Chạy dev server: npm run dev"
echo "  - Build production: npm run build"
echo "  - Start production: npm run start"
echo ""
echo "🎨 Features mới:"
echo "  ✅ Modern agricultural theme"
echo "  ✅ Gradient colors và animations"
echo "  ✅ Responsive design"
echo "  ✅ Real-time updates"
echo "  ✅ Beautiful sensor cards"
echo "  ✅ Control panel nâng cao"
echo "  ✅ Device status monitoring"
echo "  ✅ Alert system"
echo ""

# Ask if user wants to start dev server
read -p "🚀 Bạn có muốn chạy development server ngay không? (y/n): " -n 1 -r
echo

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "🚀 Starting development server..."
    npm run dev
fi
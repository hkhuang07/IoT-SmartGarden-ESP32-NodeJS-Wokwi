/*
 * Wokwi Custom Chip: Soil Moisture Sensor
 * Mô phỏng cảm biến độ ẩm đất cho Smart Garden
 * Đầu ra digital và analog
 */

class SoilMoistureSensor {
  constructor() {
    this.paused = false;
    this.time = 0;
    this.moistureValue = 1500; // Giá trị ADC
    this.moisturePercent = 50; // Phần trăm độ ẩm
    this.threshold = 2000; // Ngưỡng kích hoạt digital output
    
    this.outputDigital = 1; // 0 = khô, 1 = ướt
    this.outputAnalog = 0; // Giá trị analog 0-4095
    
    this.DRY_SOIL_VALUE = 3000;  // Giá trị đất khô
    this.WET_SOIL_VALUE = 1000;  // Giá trị đất ướt
    this.ADC_RESOLUTION = 4096;
    this.ADC_VOLTAGE = 3.3;
  }

  init() {
    this.resetOutputs();
  }

  resetOutputs() {
    this.setMoisture(1500);
  }

  setMoisture(value) {
    this.moistureValue = Math.max(0, Math.min(4095, value));
    
    // Chuyển đổi sang phần trăm
    this.moisturePercent = map(this.moistureValue, this.DRY_SOIL_VALUE, this.WET_SOIL_VALUE, 0, 100);
    this.moisturePercent = Math.max(0, Math.min(100, this.moisturePercent));
    
    this.updateOutputs();
  }

  setMoisturePercent(percent) {
    this.moisturePercent = Math.max(0, Math.min(100, percent));
    
    // Chuyển đổi phần trăm về giá trị ADC
    this.moistureValue = map(this.moisturePercent, 0, 100, this.DRY_SOIL_VALUE, this.WET_SOIL_VALUE);
    
    this.updateOutputs();
  }

  setThreshold(value) {
    this.threshold = Math.max(0, Math.min(4095, value));
    this.updateOutputs();
  }

  updateOutputs() {
    // Digital output dựa trên threshold
    this.outputDigital = (this.moistureValue > this.threshold) ? 0 : 1;
    
    // Analog output là giá trị ADC
    this.outputAnalog = this.moistureValue;
  }

  // Mô phỏng dao động thực tế của cảm biến
  simulateNaturalVariation() {
    if (!this.paused) {
      this.time += 0.05;
      
      // Mô phỏng thay đổi độ ẩm theo thời gian
      const variation = Math.sin(this.time * 0.1) * 50 + (Math.random() - 0.5) * 20;
      const baseMoisture = this.moistureValue + variation;
      
      this.moistureValue = Math.max(0, Math.min(4095, baseMoisture));
      
      // Cập nhật phần trăm
      this.moisturePercent = map(this.moistureValue, this.DRY_SOIL_VALUE, this.WET_SOIL_VALUE, 0, 100);
      this.moisturePercent = Math.max(0, Math.min(100, this.moisturePercent));
      
      this.updateOutputs();
    }
  }

  update() {
    this.simulateNaturalVariation();
  }

  // Phương thức tiện ích
  isDry() {
    return this.moistureValue > this.threshold;
  }

  isWet() {
    return this.moistureValue <= this.threshold;
  }

  getMoistureLevel() {
    if (this.moisturePercent > 80) return "Very Wet";
    if (this.moisturePercent > 60) return "Wet";
    if (this.moisturePercent > 40) return "Moderate";
    if (this.moisturePercent > 20) return "Dry";
    return "Very Dry";
  }
}

// Hàm map tương tự Arduino
function map(x, in_min, in_max, out_min, out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

const SoilMoistureSensorPackage = {
  meta: {
    name: "Soil Moisture Sensor",
    version: "1.0.0", 
    author: "MiniMax Agent",
    description: "Cảm biến độ ẩm đất cho Smart Garden",
    pins: [
      { name: "GND", type: "power" },
      { name: "VCC", type: "power" },
      { name: "DO", type: "digital" },  // Digital Output
      { name: "AO", type: "analog" }   // Analog Output
    ]
  },
  defaultAttrs: {
    moisture: "1500"
  },
  symbol: `
    <defs>
      <pattern id="soil" x="0" y="0" width="8" height="8" patternUnits="userSpaceOnUse">
        <rect x="0" y="0" width="8" height="8" fill="#8B4513"/>
        <circle cx="2" cy="2" r="0.5" fill="#654321"/>
        <circle cx="6" cy="4" r="0.3" fill="#654321"/>
        <circle cx="4" cy="6" r="0.4" fill="#654321"/>
      </pattern>
    </defs>
    <rect x="-40" y="-30" width="80" height="60" rx="6" fill="url(#soil)" stroke="currentColor" stroke-width="2"/>
    <rect x="-35" y="-25" width="70" height="20" fill="none" stroke="currentColor" stroke-width="1" opacity="0.3"/>
    <text x="0" y="-15" text-anchor="middle" font-family="monospace" font-size="10">SOIL</text>
    <text x="0" y="0" text-anchor="middle" font-family="monospace" font-size="8">Moisture</text>
    <text x="0" y="12" text-anchor="middle" font-family="monospace" font-size="8">${Math.round(map(parseInt(attrs.moisture), 3000, 1000, 0, 100))}%</text>
    <circle cx="-20" cy="20" r="2" fill="red"/>
    <text x="-15" y="23" font-family="monospace" font-size="6">DO</text>
    <circle cx="20" cy="20" r="2" fill="blue"/>
    <text x="25" y="23" font-family="monospace" font-size="6">AO</text>
  `,
  pins: {
    GND: { direction: 'in', },
    VCC: { direction: 'in', },
    DO: { direction: 'out', voltage: 0 },
    AO: { direction: 'out', voltage: 0 }
  },
  init(sensor) {
    sensor.data = new SoilMoistureSensor();
    sensor.data.setMoisture(parseInt(sensor.attrs.moisture));
  },
  update(sensor) {
    sensor.data.update();
    
    // Digital output (0-3.3V)
    sensor.pins.DO.voltage = sensor.data.outputDigital * 3.3;
    
    // Analog output (0-3.3V dựa trên giá trị ADC)
    sensor.pins.AO.voltage = sensor.data.outputAnalog * (3.3 / 4095);
  }
};

window.SoilMoistureSensor = SoilMoistureSensorPackage;

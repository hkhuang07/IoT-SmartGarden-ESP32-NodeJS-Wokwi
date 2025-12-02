/*
 * Wokwi Custom Chip: pH Sensor
 * Mô phỏng cảm biến đo độ pH cho Smart Garden
 * Đầu ra digital và analog
 * pH range: 0-14 (phổ biến cho soil testing)
 */

class PHSensor {
  constructor() {
    this.paused = false;
    this.time = 0;
    this.phValue = 7.0; // Giá trị pH hiện tại
    this.adcValue = 2048; // Giá trị ADC tương ứng (0-4095)
    this.phStatus = "Neutral"; // Acidic, Neutral, Alkaline
    
    this.outputDigital = 1; // 0 = Acidic (< 7), 1 = Alkaline (>= 7)
    this.outputAnalog = 0; // Giá trị analog 0-4095
    
    // pH scale configuration
    this.MIN_PH = 0.0;    // Strongly acidic
    this.MAX_PH = 14.0;   // Strongly alkaline  
    this.NEUTRAL_PH = 7.0; // Neutral pH
    
    // ADC mapping cho pH
    this.ACIDIC_ADC_VALUE = 1200;    // pH ~4.0
    this.NEUTRAL_ADC_VALUE = 2048;   // pH 7.0
    this.ALKALINE_ADC_VALUE = 2800;  // pH ~10.0
    
    this.ADC_RESOLUTION = 4096;
    this.ADC_VOLTAGE = 3.3;
    this.phThreshold = 7.0; // Ngưỡng phân biệt acid/alkaline
  }

  init() {
    this.resetOutputs();
  }

  resetOutputs() {
    this.setPH(7.0);
  }

  setPH(ph) {
    this.phValue = Math.max(this.MIN_PH, Math.min(this.MAX_PH, ph));
    
    // Chuyển đổi pH sang giá trị ADC
    this.adcValue = this.phToADC(this.phValue);
    
    // Xác định trạng thái pH
    this.updatePHStatus();
    
    this.updateOutputs();
  }

  setPHPercent(percent) {
    // Chuyển đổi percent sang pH (0% = very acidic, 100% = very alkaline)
    this.phValue = map(percent, 0, 100, this.MIN_PH, this.MAX_PH);
    this.setPH(this.phValue);
  }

  setThreshold(ph) {
    this.phThreshold = Math.max(this.MIN_PH, Math.min(this.MAX_PH, ph));
    this.updateOutputs();
  }

  phToADC(ph) {
    if (ph <= this.NEUTRAL_PH) {
      // Acidic range: pH 0-7
      return map(ph, this.MIN_PH, this.NEUTRAL_PH, this.ACIDIC_ADC_VALUE * 0.3, this.NEUTRAL_ADC_VALUE);
    } else {
      // Alkaline range: pH 7-14
      return map(ph, this.NEUTRAL_PH, this.MAX_PH, this.NEUTRAL_ADC_VALUE, this.ALKALINE_ADC_VALUE * 1.2);
    }
  }

  adcToPH(adc) {
    if (adc <= this.NEUTRAL_ADC_VALUE) {
      // Acidic range
      return map(adc, this.ACIDIC_ADC_VALUE * 0.3, this.NEUTRAL_ADC_VALUE, this.MIN_PH, this.NEUTRAL_PH);
    } else {
      // Alkaline range  
      return map(adc, this.NEUTRAL_ADC_VALUE, this.ALKALINE_ADC_VALUE * 1.2, this.NEUTRAL_PH, this.MAX_PH);
    }
  }

  updatePHStatus() {
    if (this.phValue < 6.0) {
      this.phStatus = "Strong Acidic";
    } else if (this.phValue < 6.8) {
      this.phStatus = "Acidic";
    } else if (this.phValue < 7.2) {
      this.phStatus = "Neutral";
    } else if (this.phValue < 8.0) {
      this.phStatus = "Alkaline";
    } else {
      this.phStatus = "Strong Alkaline";
    }
  }

  updateOutputs() {
    // Digital output dựa trên threshold (pH >= 7 = alkaline)
    this.outputDigital = (this.phValue >= this.phThreshold) ? 1 : 0;
    
    // Analog output là giá trị ADC tương ứng với pH
    this.outputAnalog = Math.round(this.phToADC(this.phValue));
  }

  // Mô phỏng dao động thực tế của cảm biến pH
  simulateNaturalVariation() {
    if (!this.paused) {
      this.time += 0.05;
      
      // Mô phỏng thay đổi pH theo thời gian (tự nhiên chậm hơn)
      const variation = Math.sin(this.time * 0.05) * 0.2 + (Math.random() - 0.5) * 0.1;
      const basePH = this.phValue + variation;
      
      this.phValue = Math.max(this.MIN_PH, Math.min(this.MAX_PH, basePH));
      
      // Cập nhật ADC value và status
      this.adcValue = this.phToADC(this.phValue);
      this.updatePHStatus();
      
      this.updateOutputs();
    }
  }

  update() {
    this.simulateNaturalVariation();
  }

  // Phương thức tiện ích
  isAcidic() {
    return this.phValue < this.phThreshold;
  }

  isAlkaline() {
    return this.phValue >= this.phThreshold;
  }

  isOptimal() {
    // pH tối ưu cho hầu hết cây trồng: 6.0-7.5
    return this.phValue >= 6.0 && this.phValue <= 7.5;
  }

  getPHLevel() {
    if (this.phValue < 4.0) return "Very Acidic";
    if (this.phValue < 5.5) return "Acidic";
    if (this.phValue < 6.5) return "Slightly Acidic";
    if (this.phValue < 7.5) return "Optimal";
    if (this.phValue < 8.5) return "Slightly Alkaline";
    if (this.phValue < 10.0) return "Alkaline";
    return "Very Alkaline";
  }

  getPlantSuitability() {
    // Đánh giá phù hợp cho cây trồng
    if (this.phValue >= 6.0 && this.phValue <= 7.5) {
      return "Excellent for most plants";
    } else if (this.phValue >= 5.5 && this.phValue <= 8.0) {
      return "Good for many plants";
    } else if (this.phValue >= 5.0 && this.phValue <= 8.5) {
      return "Tolerable for hardy plants";
    } else {
      return "Requires pH adjustment";
    }
  }
}

// Hàm map tương tự Arduino
function map(x, in_min, in_max, out_min, out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

const PHSensorPackage = {
  meta: {
    name: "pH Sensor",
    version: "1.0.0", 
    author: "MiniMax Agent",
    description: "Cảm biến đo độ pH cho Smart Garden",
    pins: [
      { name: "GND", type: "power" },
      { name: "VCC", type: "power" },
      { name: "DO", type: "digital" },  // Digital Output (pH level indicator)
      { name: "AO", type: "analog" }   // Analog Output (ADC value)
    ]
  },
  defaultAttrs: {
    ph: "7.0"
  },
  symbol: `
    <defs>
      <pattern id="phpattern" x="0" y="0" width="8" height="8" patternUnits="userSpaceOnUse">
        <rect x="0" y="0" width="8" height="8" fill="#87CEEB"/>
        <circle cx="2" cy="2" r="1" fill="#4169E1" opacity="0.7"/>
        <circle cx="6" cy="4" r="0.5" fill="#4169E1" opacity="0.7"/>
        <circle cx="4" cy="6" r="0.7" fill="#4169E1" opacity="0.7"/>
      </pattern>
    </defs>
    <rect x="-40" y="-30" width="80" height="60" rx="6" fill="url(#phpattern)" stroke="currentColor" stroke-width="2"/>
    <rect x="-35" y="-25" width="70" height="20" fill="none" stroke="currentColor" stroke-width="1" opacity="0.3"/>
    <text x="0" y="-15" text-anchor="middle" font-family="monospace" font-size="10">pH</text>
    <text x="0" y="0" text-anchor="middle" font-family="monospace" font-size="8">Sensor</text>
    <text x="0" y="12" text-anchor="middle" font-family="monospace" font-size="8">${attrs.ph || 7.0}</text>
    <text x="0" y="22" text-anchor="middle" font-family="monospace" font-size="6">
      ${(() => {
        const ph = parseFloat(attrs.ph || 7.0);
        if (ph < 6.0) return "Acid";
        if (ph > 8.0) return "Alk";
        return "Neut";
      })()}
    </text>
    <circle cx="-20" cy="35" r="2" fill="red"/>
    <text x="-15" y="38" font-family="monospace" font-size="6">DO</text>
    <circle cx="20" cy="35" r="2" fill="blue"/>
    <text x="25" y="38" font-family="monospace" font-size="6">AO</text>
  `,
  pins: {
    GND: { direction: 'in', },
    VCC: { direction: 'in', },
    DO: { direction: 'out', voltage: 0 },
    AO: { direction: 'out', voltage: 0 }
  },
  init(sensor) {
    sensor.data = new PHSensor();
    sensor.data.setPH(parseFloat(sensor.attrs.ph));
  },
  update(sensor) {
    sensor.data.update();
    
    // Digital output (0 = Acidic, 1 = Alkaline)
    sensor.pins.DO.voltage = sensor.data.outputDigital * 3.3;
    
    // Analog output (0-3.3V dựa trên giá trị ADC và pH)
    sensor.pins.AO.voltage = sensor.data.outputAnalog * (3.3 / 4095);
  }
};

window.PHSensor = PHSensorPackage;
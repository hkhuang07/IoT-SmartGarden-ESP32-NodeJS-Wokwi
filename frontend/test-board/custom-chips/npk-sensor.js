/*
 * Wokwi Custom Chip: NPK Nutrient Sensor
 * Mô phỏng cảm biến đo dinh dưỡng NPK (Nitrogen, Phosphorus, Potassium)
 * Đầu ra analog cho mỗi chất dinh dưỡng
 */

class NPKSensor {
  constructor() {
    this.paused = false;
    this.time = 0;
    this.nitrogen = 20; // ppm
    this.phosphorus = 15; // ppm
    this.potassium = 25; // ppm
    
    this.outputN = 0;
    this.outputP = 0;
    this.outputK = 0;
    
    this.CALIBRATION_N = 0.015;
    this.CALIBRATION_P = 0.012;
    this.CALIBRATION_K = 0.016;
    this.ADC_RESOLUTION = 4096;
    this.ADC_VOLTAGE = 3.3;
  }

  init() {
    this.resetOutputs();
  }

  resetOutputs() {
    this.setOutputs(0, 0, 0);
  }

  setOutputs(n, p, k) {
    this.nitrogen = n;
    this.phosphorus = p;
    this.potassium = k;
    
    // Chuyển đổi ppm thành giá trị ADC
    this.outputN = Math.round((this.nitrogen * this.CALIBRATION_N) * this.ADC_RESOLUTION / this.ADC_VOLTAGE);
    this.outputP = Math.round((this.phosphorus * this.CALIBRATION_P) * this.ADC_RESOLUTION / this.ADC_VOLTAGE);
    this.outputK = Math.round((this.potassium * this.CALIBRATION_K) * this.ADC_RESOLUTION / this.ADC_VOLTAGE);
    
    // Đảm bảo trong khoảng 0-4095
    this.outputN = Math.max(0, Math.min(4095, this.outputN));
    this.outputP = Math.max(0, Math.min(4095, this.outputP));
    this.outputK = Math.max(0, Math.min(4095, this.outputK));
  }

  setNitrogen(ppm) {
    this.nitrogen = Math.max(0, ppm);
    this.setOutputs(this.nitrogen, this.phosphorus, this.potassium);
  }

  setPhosphorus(ppm) {
    this.phosphorus = Math.max(0, ppm);
    this.setOutputs(this.nitrogen, this.phosphorus, this.potassium);
  }

  setPotassium(ppm) {
    this.potassium = Math.max(0, ppm);
    this.setOutputs(this.nitrogen, this.phosphorus, this.potassium);
  }

  setAll(n, p, k) {
    this.setOutputs(n, p, k);
  }

  // Mô phỏng dao động thực tế của cảm biến
  simulateFluctuation() {
    if (!this.paused) {
      this.time += 0.1;
      const noiseN = Math.sin(this.time) * 0.5 + (Math.random() - 0.5) * 0.3;
      const noiseP = Math.sin(this.time * 1.3) * 0.4 + (Math.random() - 0.5) * 0.2;
      const noiseK = Math.sin(this.time * 0.7) * 0.3 + (Math.random() - 0.5) * 0.4;
      
      this.outputN += noiseN;
      this.outputP += noiseP;
      this.outputK += noiseK;
      
      this.outputN = Math.max(0, Math.min(4095, this.outputN));
      this.outputP = Math.max(0, Math.min(4095, this.outputP));
      this.outputK = Math.max(0, Math.min(4095, this.outputK));
    }
  }

  update() {
    this.simulateFluctuation();
  }
}

const NPKSensorPackage = {
  meta: {
    name: "NPK Sensor",
    version: "1.0.0",
    author: "MiniMax Agent",
    description: "Cảm biến đo dinh dưỡng NPK cho Smart Garden",
    pins: [
      { name: "GND", type: "power" },
      { name: "VCC", type: "power" },
      { name: "N", type: "analog" },   // Nitrogen output
      { name: "P", type: "analog" },   // Phosphorus output
      { name: "K", type: "analog" },   // Potassium output
    ]
  },
  defaultAttrs: {
    nitrogen: "20",
    phosphorus: "15", 
    potassium: "25"
  },
  symbol: `
    <defs>
      <marker id="arrow" viewBox="0 0 10 10" refX="5" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
        <path d="M 0 0 L 10 5 L 0 10 z" fill="currentColor" />
      </marker>
    </defs>
    <rect x="-40" y="-30" width="80" height="60" rx="6" fill="none" stroke="currentColor" stroke-width="2"/>
    <text x="0" y="-15" text-anchor="middle" font-family="monospace" font-size="10">NPK</text>
    <text x="0" y="0" text-anchor="middle" font-family="monospace" font-size="8">N:P:K</text>
    <text x="0" y="12" text-anchor="middle" font-family="monospace" font-size="8">${attrs.nitrogen}:${attrs.phosphorus}:${attrs.potassium}</text>
  `,
  pins: {
    GND: { direction: 'in', },
    VCC: { direction: 'in', },
    N: { direction: 'out', voltage: 0 },
    P: { direction: 'out', voltage: 0 },
    K: { direction: 'out', voltage: 0 }
  },
  init(sensor) {
    sensor.data = new NPKSensor();
    sensor.data.setAll(
      parseFloat(sensor.attrs.nitrogen),
      parseFloat(sensor.attrs.phosphorus),
      parseFloat(sensor.attrs.potassium)
    );
  },
  update(sensor) {
    sensor.data.update();
    
    // Chuyển đổi giá trị ADC thành điện áp
    const voltageN = sensor.data.outputN * (3.3 / 4095);
    const voltageP = sensor.data.outputP * (3.3 / 4095);
    const voltageK = sensor.data.outputK * (3.3 / 4095);
    
    sensor.pins.N.voltage = voltageN;
    sensor.pins.P.voltage = voltageP;
    sensor.pins.K.voltage = voltageK;
  }
};

window.NPKSensor = NPKSensorPackage;

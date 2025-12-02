# MicroPython Servo Library for Smart Garden
# Servo Motor Controller with PWM

from machine import Pin, PWM
import time

class Servo:
    def __init__(self, pin, min_pulse=544, max_pulse=2400, angle_range=180):
        """
        Initialize servo motor
        :param pin: GPIO pin number
        :param min_pulse: Minimum pulse width in microseconds (default: 544)
        :param max_pulse: Maximum pulse width in microseconds (default: 2400)
        :param angle_range: Servo angle range in degrees (default: 180)
        """
        self.pin = Pin(pin)
        self.pwm = PWM(self.pin)
        self.pwm.freq(50)  # 50Hz frequency for servos
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.angle_range = angle_range
        self.current_angle = 0
        
        print(f"✅ Servo initialized on pin {pin}")
    
    def angle_to_duty(self, angle):
        """Convert angle (0-180) to duty cycle value"""
        # Map angle to pulse width
        pulse_width = self.min_pulse + (self.max_pulse - self.min_pulse) * angle / self.angle_range
        
        # Convert to duty cycle (0-1023 for 10-bit resolution)
        duty_cycle = int(pulse_width * 1023 / 20000)  # 20000us = 20ms period
        
        return max(0, min(1023, duty_cycle))
    
    def write_angle(self, angle):
        """Set servo angle (0-180 degrees)"""
        try:
            angle = max(0, min(self.angle_range, int(angle)))
            duty_cycle = self.angle_to_duty(angle)
            
            self.pwm.duty(duty_cycle)
            self.current_angle = angle
            
            print(f"[SERVO] Angle set to {angle}° (duty: {duty_cycle})")
            
        except Exception as e:
            print(f"❌ Servo angle error: {e}")
    
    def get_angle(self):
        """Get current servo angle"""
        return self.current_angle
    
    def center(self):
        """Center the servo (90 degrees)"""
        self.write_angle(self.angle_range // 2)
    
    def sweep(self, start_angle=0, end_angle=180, delay=0.5):
        """Sweep servo from start_angle to end_angle"""
        try:
            direction = 1 if end_angle > start_angle else -1
            current = start_angle
            
            while (direction > 0 and current <= end_angle) or (direction < 0 and current >= end_angle):
                self.write_angle(current)
                time.sleep(delay)
                current += direction
                
        except Exception as e:
            print(f"❌ Servo sweep error: {e}")
    
    def attach(self, pin):
        """Re-attach servo to different pin"""
        try:
            self.pin = Pin(pin)
            self.pwm = PWM(self.pin)
            self.pwm.freq(50)
            print(f"✅ Servo re-attached to pin {pin}")
        except Exception as e:
            print(f"❌ Servo attach error: {e}")
    
    def detach(self):
        """Detach servo (stop PWM)"""
        try:
            self.pwm.deinit()
            print("✅ Servo detached")
        except Exception as e:
            print(f"❌ Servo detach error: {e}")
    
    def calibrate(self, min_pulse=None, max_pulse=None):
        """Calibrate servo pulse range"""
        try:
            if min_pulse is not None:
                self.min_pulse = min_pulse
            if max_pulse is not None:
                self.max_pulse = max_pulse
            
            print(f"✅ Servo calibrated: min={self.min_pulse}, max={self.max_pulse}")
            
        except Exception as e:
            print(f"❌ Servo calibration error: {e}")
    
    def test_sweep(self, duration=5):
        """Test servo with full sweep"""
        print(f"[SERVO] Starting test sweep for {duration} seconds...")
        start_time = time.time()
        
        try:
            while time.time() - start_time < duration:
                # Sweep from 0 to 180
                for angle in range(0, 181, 10):
                    self.write_angle(angle)
                    time.sleep(0.2)
                
                # Sweep back from 180 to 0
                for angle in range(180, -1, -10):
                    self.write_angle(angle)
                    time.sleep(0.2)
                
        except Exception as e:
            print(f"❌ Servo test error: {e}")
        
        finally:
            self.center()
            print("[SERVO] Test sweep completed")
import pigpio
import time
import signal
import sys

class HardwareClockGenerator:
    def __init__(self, gpio_pin=17, frequency=30000000):  # 30 MHz default
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Failed to connect to pigpio daemon")
            
        self.gpio_pin = gpio_pin
        self.frequency = frequency
        
    def start_clock(self):
        """Start the hardware clock"""
        try:
            # Set GPIO pin to hardware clock mode
            success = self.pi.hardware_clock(self.gpio_pin, self.frequency)
            
            if success == 0:
                print(f"Started hardware clock on GPIO {self.gpio_pin}")
                print(f"Frequency: {self.frequency/1000000:.1f} MHz")
                print("Use Ctrl+C to stop")
                
                try:
                    while True:
                        time.sleep(1)
                except KeyboardInterrupt:
                    print("\nStopping clock")
            else:
                print("Failed to start hardware clock")
                
        except Exception as e:
            print(f"Error: {e}")
            
    def cleanup(self):
        """Stop the clock and cleanup"""
        self.pi.hardware_clock(self.gpio_pin, 0)  # Stop clock
        self.pi.stop()

def main():
    # Test different frequencies
    frequencies = [
        30000000,  # 30 MHz
        25000000,  # 25 MHz
        20000000,  # 20 MHz
        15000000,  # 15 MHz
        10000000,  # 10 MHz
        5000000    # 5 MHz
    ]
    
    generator = None
    
    try:
        for freq in frequencies:
            print(f"\nTesting {freq/1000000:.1f} MHz")
            generator = HardwareClockGenerator(frequency=freq)
            generator.start_clock()
            
            # Run for 5 seconds then try next frequency
            time.sleep(5)
            
            if generator:
                generator.cleanup()
                
    except KeyboardInterrupt:
        print("\nTest interrupted")
    finally:
        if generator:
            generator.cleanup()

if __name__ == "__main__":
    main()
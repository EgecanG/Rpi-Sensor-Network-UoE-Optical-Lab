import pigpio
import time
import signal
import sys

class SquareWaveGenerator:
    def __init__(self, gpio_pin=17):
        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Failed to connect to pigpio daemon")
        
        self.gpio_pin = gpio_pin
        self.pi.set_mode(gpio_pin, pigpio.OUTPUT)
        
        # Create shortest possible wave pulses
        # Each pulse is 1 microsecond (minimum possible)
        self.wave = [
            pigpio.pulse(1 << gpio_pin, 0, 1),  # GPIO High for 1µs
            pigpio.pulse(0, 1 << gpio_pin, 1)   # GPIO Low for 1µs
        ]
        
    def start_wave(self):
        """Start generating the square wave"""
        try:
            # Clear any existing waveforms
            self.pi.wave_clear()
            
            # Add our wave pulses to the DMA channel
            self.pi.wave_add_generic(self.wave)
            
            # Create and send the wave
            wid = self.pi.wave_create()
            
            if wid >= 0:
                print(f"Starting square wave on GPIO {self.gpio_pin}")
                print("Theoretical frequency: 500 kHz (1µs high, 1µs low)")
                print("Use Ctrl+C to stop")
                
                # Send wave repeatedly
                self.pi.wave_send_repeat(wid)
                
                try:
                    # Keep the program running
                    while True:
                        time.sleep(1)
                except KeyboardInterrupt:
                    print("\nStopping square wave")
                    
                # Clean up the waveform
                self.pi.wave_tx_stop()
                self.pi.wave_delete(wid)
                
        except Exception as e:
            print(f"Error generating wave: {e}")
            
    def cleanup(self):
        """Clean up GPIO and pigpio resources"""
        self.pi.wave_clear()
        self.pi.stop()

def main():
    generator = None
    try:
        # Create and start the square wave generator
        generator = SquareWaveGenerator()
        generator.start_wave()
    finally:
        # Ensure cleanup happens even if there's an error
        if generator:
            generator.cleanup()

if __name__ == "__main__":
    main()
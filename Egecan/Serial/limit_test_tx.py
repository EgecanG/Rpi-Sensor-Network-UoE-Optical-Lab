# sender.py (Run this on first Pi)
import serial
import time

def setup_uart_sender():
    # Configure serial port
    uart = serial.Serial(
        port='/dev/ttyAMA0',  # Serial port name: Can also use Serial0 which is connected to ttyAMA0
        baudrate=10000000,      # Must match on both devices
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.1
    )
    return uart

def send_message(uart):
    # Create exactly 5KB test string
    pattern = "ABCDEFGHIJ"
    test_string = pattern * 512  # 512 * 10 = 5120 bytes (5KB)
    
    counter = 0
    try:
        while True:
            uart.write(test_string.encode())
            print(f"Sent message {counter} of size: {len(test_string.encode())} bytes")
            
            response = uart.readline().decode().strip()
            if response:
                print(f"Received: {response}")
            
            counter += 1
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        print("\nSending stopped by user")
        uart.close()

if __name__ == '__main__':
    uart = setup_uart_sender()
    send_message(uart)
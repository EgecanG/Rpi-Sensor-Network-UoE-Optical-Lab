# sender.py (Run this on first Pi)
import serial
import time

def setup_uart_sender():
    uart = serial.Serial(
        port='/dev/ttyAMA0',
        baudrate=10000000,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.1
    )
    return uart

def send_message(uart):
    # Create test message - "Hello" repeated 200 times
    test_message = "Hello" * 200 + "\n"
    counter = 0
    
    try:
        while True:
            uart.write(test_message.encode())
            print(f"Sent message {counter}, size: {len(test_message)} bytes")
            
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
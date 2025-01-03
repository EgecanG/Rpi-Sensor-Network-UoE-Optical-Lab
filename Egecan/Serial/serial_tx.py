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
    counter = 0
    try:
        while True:
            # Create message with counter
            message = f"Hello from Pi 1! Count: {counter}\n"
            uart.write(message.encode())
            print(f"Sent: {message.strip()}")
            
            # Wait for response
            response = uart.readline().decode().strip()
            if response:
                print(f"Received: {response}")
            
            counter += 1
            time.sleep(0.001)  # Wait 1 second between messages
            
    except KeyboardInterrupt:
        print("\nSending stopped by user")
        uart.close()

if __name__ == '__main__':
    uart = setup_uart_sender()
    send_message(uart)
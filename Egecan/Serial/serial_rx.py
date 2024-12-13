# receiver.py (Run this on second Pi)
import serial
import time

def setup_uart_receiver():
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

def receive_message(uart):
    counter = 0
    try:
        while True:
            # Read incoming message
            if uart.in_waiting > 0:
                message = uart.readline().decode().strip()
                print(f"Received: {message}")
                
                # Send response
                response = f"Hello from Pi 2! Count: {counter}\n"
                uart.write(response.encode())
                print(f"Sent: {response.strip()}")
                counter += 1
                
    except KeyboardInterrupt:
        print("\nReceiving stopped by user")
        uart.close()

if __name__ == '__main__':
    uart = setup_uart_receiver()
    receive_message(uart)
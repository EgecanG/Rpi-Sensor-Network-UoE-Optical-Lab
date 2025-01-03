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
    def receive_message(uart):
    counter = 0
    error_count = 0
    expected_size = 2560  # 5KB
    try:
        while True:
            if uart.in_waiting > 0:
                message = uart.read(expected_size).decode()
                received_size = len(message.encode())
                
                # Verify the pattern
                is_valid = all(message[i:i+10] == "ABCDEFGHIJ" for i in range(0, len(message), 10))
                if not is_valid:
                    error_count += 1
                
                print(f"Received message {counter}")
                print(f"Size: {received_size} bytes")
                print(f"Valid: {'OK' if is_valid else 'ERROR'}")
                print(f"Total errors so far: {error_count}")
                print(f"Error rate: {(error_count/(counter+1)*100):.2f}%\n")
                
                # Send response
                response = f"Message {counter} received, size: {received_size}, valid: {is_valid}, total errors: {error_count}\n"
                uart.write(response.encode())
                counter += 1
                
    except KeyboardInterrupt:
        print("\nReceiving stopped by user")
        print(f"\nFinal Statistics:")
        print(f"Total messages received: {counter}")
        print(f"Total errors: {error_count}")
        print(f"Final error rate: {(error_count/counter*100):.2f}%")
        uart.close()

if __name__ == '__main__':
    uart = setup_uart_receiver()
    receive_message(uart)
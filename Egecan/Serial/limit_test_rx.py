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
    total_bytes_sent = 0
    total_byte_errors = 0
    expected_size = 1000  # 5KB
    expected_pattern = "ABCDEFGHIJ"
    
    try:
        while True:
            if uart.in_waiting > 0:
                message = uart.read(expected_size).decode()
                received_size = len(message.encode())
                total_bytes_sent += received_size
                
                # Count byte errors by comparing each character
                byte_errors = sum(1 for i in range(len(message)) 
                                if message[i] != expected_pattern[i % 10])
                total_byte_errors += byte_errors
                
                print(f"Message {counter}:")
                print(f"Bytes with errors in this message: {byte_errors}/{received_size}")
                print(f"Total bytes with errors: {total_byte_errors}/{total_bytes_sent}")
                print(f"Bit error rate: {(total_byte_errors/total_bytes_sent*100):.6f}%\n")
                
                # Send response
                response = f"Message {counter}, byte errors: {byte_errors}/{received_size}\n"
                uart.write(response.encode())
                counter += 1
                
    except KeyboardInterrupt:
        print("\nReceiving stopped by user")
        print(f"\nFinal Statistics:")
        print(f"Total messages: {counter}")
        print(f"Total bytes transmitted: {total_bytes_sent}")
        print(f"Total bytes with errors: {total_byte_errors}")
        print(f"Final bit error rate: {(total_byte_errors/total_bytes_sent*100):.6f}%")
        uart.close()

if __name__ == '__main__':
    uart = setup_uart_receiver()
    receive_message(uart)
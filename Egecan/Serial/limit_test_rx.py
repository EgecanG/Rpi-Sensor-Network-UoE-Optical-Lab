import serial
import time
from zlib import crc32
import json

def setup_uart_receiver():
    uart = serial.Serial(
        port='/dev/ttyAMA0',
        baudrate=10000000,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.1
    )
    return uart

def verify_message(data, expected_checksum):
    actual_checksum = crc32(data.encode())
    return actual_checksum == expected_checksum

def receive_messages(uart):
    print("Receiver started. Waiting for messages...")
    
    try:
        while True:
            if uart.in_waiting > 0:
                try:
                    # Read and parse incoming message
                    message = uart.readline().decode().strip()
                    packet = json.loads(message)
                    
                    # Verify message integrity
                    if verify_message(packet["data"], packet["checksum"]):
                        response = {"status": "success"}
                    else:
                        response = {
                            "status": "error",
                            "error": "Checksum mismatch"
                        }
                    
                    # Send acknowledgment
                    uart.write(json.dumps(response).encode() + b'\n')
                    
                    # Print status based on test type
                    if packet["type"] == "size_test":
                        print(f"Size test: {packet['size']} bytes - {'OK' if response['status'] == 'success' else 'FAILED'}")
                    elif packet["type"] == "speed_test":
                        if packet["iteration"] % 10 == 0:  # Print every 10th iteration
                            print(f"Speed test iteration {packet['iteration']} - {'OK' if response['status'] == 'success' else 'FAILED'}")
                    
                except json.JSONDecodeError:
                    print("Error: Invalid JSON received")
                except Exception as e:
                    print(f"Error processing message: {str(e)}")
                    
    except KeyboardInterrupt:
        print("\nReceiver stopped by user")
        uart.close()

if __name__ == '__main__':
    uart = setup_uart_receiver()
    receive_messages(uart)
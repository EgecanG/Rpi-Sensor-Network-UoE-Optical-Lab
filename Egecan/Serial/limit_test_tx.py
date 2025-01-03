import serial
import time
from zlib import crc32
import json

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

def create_test_message(size):
    # Create a repeating pattern of known data
    pattern = "abcdefghijklmnopqrstuvwxyz0123456789"
    message = (pattern * (size // len(pattern) + 1))[:size]
    return message

def test_message_size(uart, start_size=1000, max_size=100000, step=1000):
    print("\nTesting maximum message size...")
    print("Size (bytes) | Status | Time (s)")
    print("-" * 40)
    
    for size in range(start_size, max_size + 1, step):
        # Create and send test message
        message = create_test_message(size)
        checksum = crc32(message.encode())
        
        # Package message with checksum
        packet = {
            "type": "size_test",
            "size": size,
            "checksum": checksum,
            "data": message
        }
        
        # Measure transmission time
        start_time = time.time()
        try:
            uart.write(json.dumps(packet).encode() + b'\n')
            
            # Wait for acknowledgment
            response = uart.readline().decode().strip()
            end_time = time.time()
            
            if not response:
                print(f"{size:11d} | FAILED | Timeout")
                break
                
            response_data = json.loads(response)
            if response_data["status"] == "success":
                print(f"{size:11d} | OK     | {end_time - start_time:.4f}")
            else:
                print(f"{size:11d} | FAILED | {response_data['error']}")
                break
                
        except Exception as e:
            print(f"{size:11d} | ERROR  | {str(e)}")
            break
            
        time.sleep(0.1)  # Short delay between tests

def test_transmission_speed(uart, message_size=1000, start_delay=0.1, min_delay=0.0001, iterations=100):
    print("\nTesting minimum transmission delay...")
    print("Delay (s) | Success Rate | Avg Time (s)")
    print("-" * 45)
    
    current_delay = start_delay
    while current_delay >= min_delay:
        successes = 0
        total_time = 0
        
        message = create_test_message(message_size)
        checksum = crc32(message.encode())
        
        for i in range(iterations):
            packet = {
                "type": "speed_test",
                "iteration": i,
                "checksum": checksum,
                "data": message
            }
            
            start_time = time.time()
            try:
                uart.write(json.dumps(packet).encode() + b'\n')
                response = uart.readline().decode().strip()
                
                if response:
                    response_data = json.loads(response)
                    if response_data["status"] == "success":
                        successes += 1
                        total_time += time.time() - start_time
                
            except Exception:
                pass
                
            time.sleep(current_delay)
        
        success_rate = (successes / iterations) * 100
        avg_time = total_time / iterations if successes > 0 else 0
        print(f"{current_delay:.6f} | {success_rate:10.1f}% | {avg_time:.6f}")
        
        if success_rate < 90:  # Stop if reliability drops below 90%
            break
            
        current_delay /= 2  # Try a shorter delay

if __name__ == '__main__':
    uart = setup_uart_sender()
    
    while True:
        print("\nUART Testing Menu:")
        print("1. Test maximum message size")
        print("2. Test minimum transmission delay")
        print("3. Exit")
        
        choice = input("Select test (1-3): ")
        
        if choice == '1':
            test_message_size(uart)
        elif choice == '2':
            test_transmission_speed(uart)
        elif choice == '3':
            uart.close()
            break
        else:
            print("Invalid choice. Please try again.")
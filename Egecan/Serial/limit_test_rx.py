# receiver.py (Run this on second Pi)
import serial
import time

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

def receive_message(uart):
    expected_message = "Hello" * 200 + "\n"
    counter = 0
    error_count = 0
    
    try:
        while True:
            if uart.in_waiting > 0:
                received = uart.readline().decode()
                
                # Verify if message is correct
                is_correct = (received == expected_message)
                if not is_correct:
                    error_count += 1
                
                print(f"\nMessage {counter}:")
                print(f"Correct: {'Yes' if is_correct else 'No'}")
                print(f"Total errors so far: {error_count}/{counter + 1}")
                print(f"Error rate: {(error_count/(counter + 1)*100):.2f}%")
                
                # Send response
                response = f"Message {counter}: {'OK' if is_correct else 'ERROR'}, Total errors: {error_count}\n"
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
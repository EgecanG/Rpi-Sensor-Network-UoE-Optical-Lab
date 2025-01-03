import serial
import time
import cv2
import numpy as np
import zlib  # For checksum verification

def setup_uart_receiver():
    uart = serial.Serial(
        port='/dev/ttyAMA0',
        baudrate=10000000,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )
    return uart

def receive_frame(uart):
    try:
        # Read header (8 bytes: 4 for size + 4 for checksum)
        header = uart.read(8)
        if not header or len(header) < 8:
            print("Failed to receive header")
            return None
            
        frame_size = int.from_bytes(header[:4], byteorder='big')
        expected_checksum = int.from_bytes(header[4:], byteorder='big')
        
        # Read frame data
        frame_data = bytearray()
        while len(frame_data) < frame_size:
            remaining = frame_size - len(frame_data)
            chunk_size = min(1024, remaining)
            chunk = uart.read(chunk_size)
            if not chunk:
                print("Failed to receive complete frame")
                return None
            frame_data.extend(chunk)
        
        # Verify checksum
        actual_checksum = zlib.crc32(frame_data)
        if actual_checksum != expected_checksum:
            print("Checksum mismatch")
            return None
            
        # Send acknowledgment
        uart.write(b'A')
        
        # Decode frame
        frame_arr = np.frombuffer(frame_data, dtype=np.uint8)
        frame = cv2.imdecode(frame_arr, cv2.IMREAD_COLOR)
        
        return frame
        
    except Exception as e:
        print(f"Error receiving frame: {e}")
        return None

def main():
    uart = setup_uart_receiver()
    cv2.namedWindow('Received Frame', cv2.WINDOW_NORMAL)
    
    try:
        frame_count = 0
        while True:
            frame = receive_frame(uart)
            if frame is not None:
                cv2.imshow('Received Frame', frame)
                frame_count += 1
                print(f"Received frame {frame_count}")
                
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                    
    except KeyboardInterrupt:
        print("\nReceiving stopped by user")
    finally:
        cv2.destroyAllWindows()
        uart.close()

if __name__ == '__main__':
    main()
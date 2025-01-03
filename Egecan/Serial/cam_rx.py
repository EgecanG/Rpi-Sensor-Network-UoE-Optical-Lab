import serial
import time
import cv2
import numpy as np
import zlib

def setup_uart_receiver():
    uart = serial.Serial(
        port='/dev/ttyAMA0',
        baudrate=10000000,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=2
    )
    return uart

def receive_frame(uart):
    try:
        start_time = time.time()
        
        # Read header
        header = uart.read(8)
        if not header or len(header) < 8:
            print("Failed to receive header")
            return None
            
        frame_size = int.from_bytes(header[:4], byteorder='big')
        expected_checksum = int.from_bytes(header[4:], byteorder='big')
        
        print(f"Expecting frame of size: {frame_size / 1024:.2f} KB")
        
        # Read frame data in smaller chunks
        frame_data = bytearray()
        chunk_size = 256  # Reduced chunk size
        chunks_received = 0
        total_chunks = frame_size // chunk_size + (1 if frame_size % chunk_size else 0)
        
        while len(frame_data) < frame_size:
            remaining = frame_size - len(frame_data)
            current_chunk_size = min(chunk_size, remaining)
            chunk = uart.read(current_chunk_size)
            
            if not chunk:
                print(f"Failed to receive chunk. Got {len(frame_data)} of {frame_size} bytes")
                return None
                
            frame_data.extend(chunk)
            chunks_received += 1
            
            if chunks_received % 20 == 0:  # Progress update every 20 chunks
                print(f"Progress: {chunks_received}/{total_chunks} chunks")
        
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
        
        receive_time = time.time() - start_time
        print(f"Frame reception took {receive_time:.2f} seconds")
        
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
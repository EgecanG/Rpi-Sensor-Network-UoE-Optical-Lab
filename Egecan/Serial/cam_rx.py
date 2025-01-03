import serial
import time
import cv2
import numpy as np
import zlib

# Must match transmitter settings
FIXED_FRAME_SIZE = 100 * 1024  # 100KB fixed size
CHUNK_SIZE = 256

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
        
        if frame_size != FIXED_FRAME_SIZE:
            print(f"Unexpected frame size: {frame_size}, expected {FIXED_FRAME_SIZE}")
            return None
        
        # Read frame data in chunks
        frame_data = bytearray()
        chunks_received = 0
        total_chunks = FIXED_FRAME_SIZE // CHUNK_SIZE
        
        while len(frame_data) < FIXED_FRAME_SIZE:
            chunk = uart.read(CHUNK_SIZE)
            
            if not chunk:
                print(f"Failed to receive chunk. Got {len(frame_data)} of {FIXED_FRAME_SIZE} bytes")
                return None
                
            frame_data.extend(chunk)
            chunks_received += 1
            
            if chunks_received % 20 == 0:
                print(f"Progress: {chunks_received}/{total_chunks} chunks")
        
        # Verify checksum
        actual_checksum = zlib.crc32(frame_data)
        if actual_checksum != expected_checksum:
            print("Checksum mismatch")
            return None
            
        # Send acknowledgment
        uart.write(b'A')
        
        # Find the end of actual JPEG data (before padding)
        # Look for the JPEG EOF marker (0xFF 0xD9)
        for i in range(len(frame_data)-2, 0, -1):
            if frame_data[i] == 0xFF and frame_data[i+1] == 0xD9:
                frame_data = frame_data[:i+2]
                break
        
        # Decode frame
        frame_arr = np.frombuffer(frame_data, dtype=np.uint8)
        frame = cv2.imdecode(frame_arr, cv2.IMREAD_COLOR)
        
        if frame is None:
            print("Failed to decode image")
            return None
        
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
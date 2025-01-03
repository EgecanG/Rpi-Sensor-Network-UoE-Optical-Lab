import serial
import time
import cv2
import numpy as np
from picamera2 import Picamera2
import zlib

# Define a fixed frame size (slightly larger than typical compressed size)
FIXED_FRAME_SIZE = 100 * 1024  # 100KB fixed size
CHUNK_SIZE = 256

def setup_uart_sender():
    uart = serial.Serial(
        port='/dev/ttyAMA0',
        baudrate=10000000,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=2
    )
    return uart

def setup_camera():
    camera = Picamera2()
    config = camera.create_still_configuration(
        main={"size": (1280, 720), "format": "RGB888"},
    )
    camera.configure(config)
    camera.start()
    time.sleep(2)
    return camera

def compress_frame(frame, target_size):
    quality = 85
    min_quality = 5
    max_quality = 95
    
    while True:
        _, encoded_frame = cv2.imencode('.jpg', frame, [
            cv2.IMWRITE_JPEG_QUALITY, quality,
            cv2.IMWRITE_PNG_COMPRESSION, 0
        ])
        size = len(encoded_frame.tobytes())
        
        if size <= target_size - 1024:  # Leave some room for padding
            # Pad the frame data to reach exact size
            frame_data = encoded_frame.tobytes()
            padding = bytes([0] * (target_size - len(frame_data)))
            return frame_data + padding
            
        if size > target_size:
            max_quality = quality - 1
        else:
            min_quality = quality + 1
            
        if max_quality <= min_quality:
            # If we can't achieve target size, force it by truncation/padding
            frame_data = encoded_frame.tobytes()[:target_size]
            if len(frame_data) < target_size:
                padding = bytes([0] * (target_size - len(frame_data)))
                frame_data += padding
            return frame_data
            
        quality = (min_quality + max_quality) // 2

def send_frame(uart, frame):
    try:
        start_time = time.time()
        
        # Compress and pad frame to fixed size
        frame_data = compress_frame(frame, FIXED_FRAME_SIZE)
        checksum = zlib.crc32(frame_data)
        
        # Send header with fixed size and checksum
        header = FIXED_FRAME_SIZE.to_bytes(4, byteorder='big')
        header += checksum.to_bytes(4, byteorder='big')
        uart.write(header)
        uart.flush()
        time.sleep(0.005)
        
        # Send frame data in chunks
        chunks_sent = 0
        total_chunks = FIXED_FRAME_SIZE // CHUNK_SIZE
        
        for i in range(0, FIXED_FRAME_SIZE, CHUNK_SIZE):
            chunk = frame_data[i:i + CHUNK_SIZE]
            uart.write(chunk)
            uart.flush()
            chunks_sent += 1
            if chunks_sent % 20 == 0:
                print(f"Progress: {chunks_sent}/{total_chunks} chunks")
            time.sleep(0.003)
            
        # Wait for acknowledgment
        ack = uart.read(1)
        if not ack or ack != b'A':
            print("No acknowledgment received")
        
        transmission_time = time.time() - start_time
        print(f"Frame transmission took {transmission_time:.2f} seconds")
            
    except Exception as e:
        print(f"Error sending frame: {e}")

def main():
    uart = setup_uart_sender()
    camera = setup_camera()
    
    try:
        frame_count = 0
        while True:
            start_time = time.time()
            
            frame = camera.capture_array()
            send_frame(uart, frame)
            frame_count += 1
            print(f"Sent frame {frame_count}")
            
            elapsed = time.time() - start_time
            sleep_time = max(0, 1.0 - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        print("\nSending stopped by user")
    finally:
        camera.stop()
        uart.close()

if __name__ == '__main__':
    main()
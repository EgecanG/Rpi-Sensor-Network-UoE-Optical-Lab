import serial
import time
import cv2
import numpy as np
from picamera2 import Picamera2
import zlib

def setup_uart_sender():
    uart = serial.Serial(
        port='/dev/ttyAMA0',
        baudrate=10000000,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=2  # Increased timeout for larger frames
    )
    return uart

def setup_camera():
    camera = Picamera2()
    # Configure camera for HD resolution
    config = camera.create_still_configuration(
        main={"size": (1280, 720), "format": "RGB888"},
    )
    camera.configure(config)
    camera.start()
    # Allow camera to warm up
    time.sleep(2)
    return camera

def send_frame(uart, frame):
    try:
        start_time = time.time()
        
        # Compress frame with higher quality
        _, encoded_frame = cv2.imencode('.jpg', frame, [
            cv2.IMWRITE_JPEG_QUALITY, 85,
            cv2.IMWRITE_PNG_COMPRESSION, 0
        ])
        frame_data = encoded_frame.tobytes()
        
        # Calculate checksum
        checksum = zlib.crc32(frame_data)
        
        # Print frame size for monitoring
        print(f"Frame size: {len(frame_data) / 1024:.2f} KB")
        
        # Send header: [size(4 bytes) + checksum(4 bytes)]
        header = len(frame_data).to_bytes(4, byteorder='big')
        header += checksum.to_bytes(4, byteorder='big')
        uart.write(header)
        
        # Send frame data in larger chunks
        chunk_size = 4096  # Increased chunk size
        for i in range(0, len(frame_data), chunk_size):
            chunk = frame_data[i:i + chunk_size]
            uart.write(chunk)
            uart.flush()
            time.sleep(0.002)  # Slightly increased delay
            
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
            
            # Calculate sleep time to maintain 1 FPS
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
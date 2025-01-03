import serial
import time
import cv2
import numpy as np
from picamera2 import Picamera2
import zlib  # For checksum

def setup_uart_sender():
    uart = serial.Serial(
        port='/dev/ttyAMA0',
        baudrate=10000000,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )
    return uart

def setup_camera():
    camera = Picamera2()
    # Configure camera for a smaller resolution to reduce data
    config = camera.create_still_configuration(
        main={"size": (320, 240), "format": "RGB888"},
    )
    camera.configure(config)
    camera.start()
    # Allow camera to warm up
    time.sleep(2)
    return camera

def send_frame(uart, frame):
    try:
        # No need to convert color space, just compress directly
        _, encoded_frame = cv2.imencode('.jpg', frame, [
            cv2.IMWRITE_JPEG_QUALITY, 80,
            cv2.IMWRITE_PNG_COMPRESSION, 0  # Disable PNG compression to avoid warnings
        ])
        frame_data = encoded_frame.tobytes()
        
        # Calculate checksum
        checksum = zlib.crc32(frame_data)
        
        # Send header: [size(4 bytes) + checksum(4 bytes)]
        header = len(frame_data).to_bytes(4, byteorder='big')
        header += checksum.to_bytes(4, byteorder='big')
        uart.write(header)
        
        # Send frame data in chunks
        chunk_size = 1024
        for i in range(0, len(frame_data), chunk_size):
            chunk = frame_data[i:i + chunk_size]
            uart.write(chunk)
            uart.flush()
            time.sleep(0.001)  # Small delay to prevent buffer overflow
            
        # Wait for acknowledgment
        ack = uart.read(1)
        if not ack or ack != b'A':
            print("No acknowledgment received")
            
    except Exception as e:
        print(f"Error sending frame: {e}")

def main():
    uart = setup_uart_sender()
    camera = setup_camera()
    
    try:
        frame_count = 0
        while True:
            frame = camera.capture_array()
            send_frame(uart, frame)
            frame_count += 1
            print(f"Sent frame {frame_count}")
            time.sleep(0.1)  # 10 FPS
                
    except KeyboardInterrupt:
        print("\nSending stopped by user")
        camera.stop()
        uart.close()

if __name__ == '__main__':
    main()
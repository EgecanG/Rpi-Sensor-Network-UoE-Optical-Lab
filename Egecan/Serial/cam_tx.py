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

def send_frame(uart, frame):
    try:
        start_time = time.time()
        
        _, encoded_frame = cv2.imencode('.jpg', frame, [
            cv2.IMWRITE_JPEG_QUALITY, 85,
            cv2.IMWRITE_PNG_COMPRESSION, 0
        ])
        frame_data = encoded_frame.tobytes()
        
        checksum = zlib.crc32(frame_data)
        print(f"Frame size: {len(frame_data) / 1024:.2f} KB")
        
        # Send header
        header = len(frame_data).to_bytes(4, byteorder='big')
        header += checksum.to_bytes(4, byteorder='big')
        uart.write(header)
        uart.flush()
        time.sleep(0.005)  # Wait after header
        
        # Send frame data in smaller chunks
        chunk_size = 256  # Reduced chunk size
        chunks_sent = 0
        total_chunks = len(frame_data) // chunk_size + (1 if len(frame_data) % chunk_size else 0)
        
        for i in range(0, len(frame_data), chunk_size):
            chunk = frame_data[i:i + chunk_size]
            uart.write(chunk)
            uart.flush()
            chunks_sent += 1
            if chunks_sent % 20 == 0:  # Progress update every 20 chunks
                print(f"Progress: {chunks_sent}/{total_chunks} chunks")
            time.sleep(0.003)  # Increased delay between chunks
            
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
            
            # Wait for next second
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
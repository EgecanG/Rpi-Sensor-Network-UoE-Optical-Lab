import serial
import time
import cv2
import numpy as np
from picamera2 import Picamera2

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

def setup_camera():
    camera = Picamera2()
    # Configure camera for 640x480 resolution
    config = camera.create_still_configuration(
        main={"size": (640, 480)},
        raw={"size": camera.sensor_resolution}
    )
    camera.configure(config)
    camera.start()
    return camera

def send_frame(uart, frame):
    # Convert frame from BGR to RGB (picamera2 captures in RGB format)
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    
    # Compress frame to reduce size
    _, encoded_frame = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 50])
    frame_data = encoded_frame.tobytes()
    
    # Add header with frame size
    header = len(frame_data).to_bytes(4, byteorder='big')
    
    # Send header first
    uart.write(header)
    # Send frame data in chunks
    chunk_size = 1024
    for i in range(0, len(frame_data), chunk_size):
        chunk = frame_data[i:i + chunk_size]
        uart.write(chunk)
        uart.flush()
        time.sleep(0.001)  # Small delay to prevent buffer overflow

def main():
    uart = setup_uart_sender()
    camera = setup_camera()
    
    try:
        while True:
            # Capture frame using picamera2
            frame = camera.capture_array()
            send_frame(uart, frame)
            time.sleep(0.1)  # Adjust frame rate as needed
                
    except KeyboardInterrupt:
        print("\nSending stopped by user")
        camera.stop()
        uart.close()

if __name__ == '__main__':
    main()
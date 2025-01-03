import serial
import time
import cv2
import numpy as np
import pickle

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
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return camera

def send_frame(uart, frame):
    # Compress frame to reduce size
    _, encoded_frame = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
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
            ret, frame = camera.read()
            if ret:
                send_frame(uart, frame)
                time.sleep(0.1)  # Adjust frame rate as needed
                
    except KeyboardInterrupt:
        print("\nSending stopped by user")
        camera.release()
        uart.close()

if __name__ == '__main__':
    main()
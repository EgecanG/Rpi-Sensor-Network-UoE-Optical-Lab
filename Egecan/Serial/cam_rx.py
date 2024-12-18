# receiver.py (Run this on second Pi)
import serial
import time
import numpy as np
from PIL import Image
import io
import cv2

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

def receive_image(uart):
    # First receive the image size
    size_str = uart.readline().decode().strip()
    try:
        size = int(size_str)
    except ValueError:
        return None
    
    # Send ready signal
    uart.write("READY\n".encode())
    
    # Receive the image data in chunks
    received_data = bytearray()
    chunk_size = 1024
    
    while len(received_data) < size:
        remaining = size - len(received_data)
        chunk = uart.read(min(chunk_size, remaining))
        
        if not chunk:
            return None
            
        received_data.extend(chunk)
        uart.write("OK\n".encode())
    
    return bytes(received_data)

def display_image(image_data):
    try:
        # Convert received bytes to image
        image = Image.open(io.BytesIO(image_data))
        # Convert PIL image to OpenCV format
        cv_image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        
        # Display image
        cv2.imshow('Received Image', cv_image)
        cv2.waitKey(1)  # Update window
        
        return True
    except Exception as e:
        print(f"Error displaying image: {e}")
        return False

def main():
    uart = setup_uart_receiver()
    
    try:
        cv2.namedWindow('Received Image', cv2.WINDOW_NORMAL)
        
        while True:
            print("Waiting for image...")
            image_data = receive_image(uart)
            
            if image_data:
                print(f"Received image ({len(image_data)} bytes)")
                if display_image(image_data):
                    print("Image displayed successfully")
                else:
                    print("Failed to display image")
            else:
                print("Failed to receive image")
            
    except KeyboardInterrupt:
        print("\nReceiving stopped by user")
        uart.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
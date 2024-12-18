# sender.py (Run this on first Pi)
import serial
import time
from picamera2 import Picamera2
import io
import numpy as np
from PIL import Image

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
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(config)
    picam2.start()
    return picam2

def capture_and_compress_image(picam2):
    # Capture image
    array = picam2.capture_array()
    
    # Convert to PIL Image and compress
    img = Image.fromarray(array)
    
    # Save to bytes with compression
    img_byte_arr = io.BytesIO()
    img.save(img_byte_arr, format='JPEG', quality=50)
    return img_byte_arr.getvalue()

def send_image(uart, image_data):
    # First send the size of the image
    size = len(image_data)
    uart.write(f"{size}\n".encode())
    
    # Wait for receiver to acknowledge
    response = uart.readline().decode().strip()
    if response != "READY":
        return False
    
    # Send the image data in chunks
    chunk_size = 1024
    for i in range(0, len(image_data), chunk_size):
        chunk = image_data[i:i + chunk_size]
        uart.write(chunk)
        
        # Wait for acknowledgment after each chunk
        response = uart.readline().decode().strip()
        if response != "OK":
            return False
    
    return True

def main():
    uart = setup_uart_sender()
    picam2 = setup_camera()
    
    try:
        while True:
            print("Capturing new image...")
            image_data = capture_and_compress_image(picam2)
            
            print(f"Sending image ({len(image_data)} bytes)...")
            if send_image(uart, image_data):
                print("Image sent successfully")
            else:
                print("Failed to send image")
            
            time.sleep(1)  # Wait before sending next image
            
    except KeyboardInterrupt:
        print("\nSending stopped by user")
        uart.close()
        picam2.close()

if __name__ == '__main__':
    main()
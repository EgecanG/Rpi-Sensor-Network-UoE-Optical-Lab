import serial
import time
import cv2
import numpy as np
import pickle

def setup_uart_receiver():
    uart = serial.Serial(
        port='/dev/ttyAMA0',
        baudrate=10000000,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.1
    )
    return uart

def receive_frame(uart):
    # Read header with frame size
    header = uart.read(4)
    if not header:
        return None
        
    frame_size = int.from_bytes(header, byteorder='big')
    
    # Read frame data
    frame_data = bytearray()
    while len(frame_data) < frame_size:
        remaining = frame_size - len(frame_data)
        chunk_size = min(1024, remaining)
        chunk = uart.read(chunk_size)
        if not chunk:
            return None
        frame_data.extend(chunk)
    
    # Decode frame
    frame_arr = np.frombuffer(frame_data, dtype=np.uint8)
    frame = cv2.imdecode(frame_arr, cv2.IMREAD_COLOR)
    return frame

def main():
    uart = setup_uart_receiver()
    
    try:
        while True:
            frame = receive_frame(uart)
            if frame is not None:
                cv2.imshow('Received Frame', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
    except KeyboardInterrupt:
        print("\nReceiving stopped by user")
        cv2.destroyAllWindows()
        uart.close()

if __name__ == '__main__':
    main()
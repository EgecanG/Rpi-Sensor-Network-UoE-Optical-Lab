from picamera2 import Picamera2
import numpy as np
import subprocess
import cv2
import time
import sys

def capture_frame():
    # Initialize camera
    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (160, 120)})  # Small size for testing
    picam2.configure(config)
    picam2.start()
    time.sleep(0.5)
    
    # Capture frame
    frame = picam2.capture_array()
    picam2.stop()
    
    # Convert to BGR for OpenCV
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return frame.astype(np.uint8)

def send_frame(frame):
    # Start the C program to send bits
    process = subprocess.Popen(['sudo', './gpio_transfer', 'send'], 
                             stdin=subprocess.PIPE)
    
    # Send frame data byte by byte
    process.stdin.write(frame.tobytes())
    process.stdin.close()
    process.wait()

def receive_frame(shape):
    # Calculate total bytes needed
    total_bytes = shape[0] * shape[1] * shape[2]
    
    # Start the C program to receive bits
    process = subprocess.Popen(['sudo', './gpio_transfer', 'receive'],
                             stdout=subprocess.PIPE)
    
    # Read exact number of bytes needed
    received_data = process.stdout.read(total_bytes)
    process.terminate()
    
    # Convert back to numpy array
    return np.frombuffer(received_data, dtype=np.uint8).reshape(shape)

def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} [send|receive]")
        return

    if sys.argv[1] == "send":
        # Capture and send frame
        frame = capture_frame()
        print(f"Captured frame shape: {frame.shape}")
        
        # Display original frame
        cv2.imshow('Sending Frame', frame)
        cv2.waitKey(1000)
        
        # Send frame
        send_frame(frame)
        
        # Save frame for verification
        cv2.imwrite('sent_frame.jpg', frame)
        
    elif sys.argv[1] == "receive":
        # Known frame shape (must match sender)
        shape = (120, 160, 3)
        
        # Receive frame
        frame = receive_frame(shape)
        
        # Display received frame
        cv2.imshow('Received Frame', frame)
        cv2.waitKey(0)
        
        # Save received frame
        cv2.imwrite('received_frame.jpg', frame)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
from picamera2 import Picamera2
import numpy as np
import subprocess
import cv2
import time

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

def transfer_frame(frame):
    # Save frame to binary file
    frame.tofile('image_data.bin')
    
    # Run the C program for transfer
    try:
        subprocess.run(['sudo', './gpio_transfer'], check=True)
        
        # Read the received data
        received_data = np.fromfile('received_data.bin', dtype=np.uint8)
        return received_data.reshape(frame.shape)
        
    except subprocess.CalledProcessError as e:
        print(f"Transfer error: {e}")
        return None

def main():
    try:
        # Capture frame
        print("Capturing frame...")
        frame = capture_frame()
        print(f"Frame captured: shape={frame.shape}")
        
        # Display original frame
        cv2.imshow('Original Frame', frame)
        cv2.waitKey(1)
        
        # Transfer frame
        print("\nTransferring frame...")
        received_frame = transfer_frame(frame)
        
        if received_frame is not None:
            # Display both frames
            cv2.imshow('Original Frame', frame)
            cv2.imshow('Received Frame', received_frame)
            
            # Calculate accuracy
            accuracy = np.mean(frame == received_frame) * 100
            print(f"\nTransfer accuracy: {accuracy:.1f}%")
            
            # Wait for key press
            cv2.waitKey(0)
        
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
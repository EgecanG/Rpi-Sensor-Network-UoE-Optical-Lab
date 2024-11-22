from picamera2 import Picamera2
import numpy as np
import subprocess
import matplotlib.pyplot as plt

def capture_frame():
    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (640, 480)})
    picam2.configure(config)
    picam2.start()
    time.sleep(2)
    
    frame = picam2.capture_array()
    picam2.stop()
    
    # Resize for transfer
    frame = frame[::4, ::4, :3]  # Larger image since we have faster transfer
    return np.clip(frame, 0, 255).astype(np.uint8)

def transfer_image(frame):
    # Save image data to file
    frame.tofile('image_data.bin')
    
    # Call C program to transfer data
    subprocess.run(['sudo', './gpio_transfer'], check=True)
    
    # Read back the transferred data
    received_data = np.fromfile('received_data.bin', dtype=np.uint8)
    return received_data.reshape(frame.shape)

def display_images(original, received):
    plt.figure(figsize=(12, 6))
    
    plt.subplot(121)
    plt.imshow(original[:, :, ::-1])
    plt.title('Original Frame')
    plt.axis('off')
    
    plt.subplot(122)
    plt.imshow(received[:, :, ::-1])
    plt.title('Received Frame')
    plt.axis('off')
    
    plt.suptitle('High-Speed Image Transfer')
    plt.tight_layout()
    plt.show()

def main():
    try:
        print("Capturing frame...")
        frame = capture_frame()
        print(f"Frame captured! Shape: {frame.shape}")
        
        print("\nTransferring image...")
        received_frame = transfer_image(frame)
        
        print("\nDisplaying images...")
        display_images(frame, received_frame)
        
        accuracy = np.mean(frame == received_frame) * 100
        print(f"\nTransfer accuracy: {accuracy:.1f}%")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()

from picamera2 import Picamera2
import numpy as np
import subprocess
import matplotlib.pyplot as plt
import time
import os

def capture_frame():
    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (320, 240)})  # Smaller size for faster transfer
    picam2.configure(config)
    picam2.start()
    time.sleep(0.1)  # Reduced warm-up time
    
    frame = picam2.capture_array()
    picam2.stop()
    
    frame = frame[::2, ::2, :3]  # Smaller resize factor
    return np.clip(frame, 0, 255).astype(np.uint8)

def transfer_image(frame):
    frame.tofile('image_data.bin')
    subprocess.run(['sudo', './gpio_transfer'], check=True)
    
    if os.path.exists('received_data.bin'):
        received_data = np.fromfile('received_data.bin', dtype=np.uint8)
        if received_data.size == frame.size:
            return received_data.reshape(frame.shape)
    return None

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
        start_time = time.time()
        total_bytes = 0
        frames_transferred = 0
        
        while True:
            frame = capture_frame()
            total_bytes += frame.size
            
            received_frame = transfer_image(frame)
            frames_transferred += 1
            
            elapsed_time = time.time() - start_time
            current_rate = total_bytes / (1024 * 1024 * elapsed_time)
            
            print(f"\rTime: {elapsed_time:.2f}s, Rate: {current_rate:.2f} MB/s, Frames: {frames_transferred}", end='')
            
            if elapsed_time >= 10.0:
                print("\n\nFinal Results:")
                print(f"Total data transferred: {total_bytes / (1024*1024):.2f} MB")
                print(f"Average rate: {current_rate:.2f} MB/s")
                print(f"Frames transferred: {frames_transferred}")
                
                if received_frame is not None:
                    display_images(frame, received_frame)
                break
                
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        for file in ['image_data.bin', 'received_data.bin']:
            if os.path.exists(file):
                try:
                    os.remove(file)
                except Exception as e:
                    print(f"Warning: Could not remove {file}: {e}")

if __name__ == "__main__":
    main()
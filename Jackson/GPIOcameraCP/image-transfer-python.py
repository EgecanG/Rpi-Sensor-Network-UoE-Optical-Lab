from picamera2 import Picamera2
import numpy as np
import subprocess
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import os

class ImageTransfer:
    def __init__(self):
        # Initialize camera
        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(main={"size": (320, 240)})
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(0.1)
        
        # Setup display
        plt.ion()  # Enable interactive mode
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 6))
        self.img1 = self.ax1.imshow(np.zeros((120, 160, 3)))  # Placeholder
        self.img2 = self.ax2.imshow(np.zeros((120, 160, 3)))
        self.ax1.set_title('Original Frame')
        self.ax2.set_title('Received Frame')
        self.ax1.axis('off')
        self.ax2.axis('off')
        self.fig.suptitle('High-Speed Image Transfer')
        plt.tight_layout()
        
        self.start_time = time.time()
        self.total_bytes = 0
        self.frames_transferred = 0

    def capture_frame(self):
        frame = self.picam2.capture_array()
        frame = frame[::2, ::2, :3]  # Smaller resize factor
        return np.clip(frame, 0, 255).astype(np.uint8)

    def transfer_image(self, frame):
        frame.tofile('image_data.bin')
        subprocess.run(['sudo', './gpio_transfer'], check=True)
        
        if os.path.exists('received_data.bin'):
            received_data = np.fromfile('received_data.bin', dtype=np.uint8)
            if received_data.size == frame.size:
                return received_data.reshape(frame.shape)
        return None

    def update_display(self, frame, received_frame):
        # Update images
        self.img1.set_array(frame[:, :, ::-1])  # BGR to RGB
        self.img2.set_array(received_frame[:, :, ::-1])
        self.fig.canvas.flush_events()
        plt.pause(0.001)  # Small pause to allow display update

    def run(self):
        try:
            while True:
                # Capture and transfer frame
                frame = self.capture_frame()
                self.total_bytes += frame.size
                
                received_frame = self.transfer_image(frame)
                if received_frame is not None:
                    self.frames_transferred += 1
                    self.update_display(frame, received_frame)
                
                # Calculate and display stats
                elapsed_time = time.time() - self.start_time
                current_rate = self.total_bytes / (1024 * 1024 * elapsed_time)
                print(f"\rTime: {elapsed_time:.2f}s, Rate: {current_rate:.2f} MB/s, "
                      f"Frames: {self.frames_transferred}", end='')
                
                # Check if 10 seconds have elapsed
                if elapsed_time >= 10.0:
                    print("\n\nFinal Results:")
                    print(f"Total data transferred: {self.total_bytes / (1024*1024):.2f} MB")
                    print(f"Average rate: {current_rate:.2f} MB/s")
                    print(f"Frames transferred: {self.frames_transferred}")
                    break
                    
        except Exception as e:
            print(f"\nError: {e}")
        finally:
            # Cleanup
            self.picam2.stop()
            for file in ['image_data.bin', 'received_data.bin']:
                if os.path.exists(file):
                    try:
                        os.remove(file)
                    except Exception as e:
                        print(f"Warning: Could not remove {file}: {e}")
            
            # Keep display open for a few seconds
            plt.pause(5)

if __name__ == "__main__":
    transfer = ImageTransfer()
    transfer.run()
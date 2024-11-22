from picamera2 import Picamera2
import numpy as np
import subprocess
import cv2
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
        
        # Initialize stats
        self.start_time = time.time()
        self.total_bytes = 0
        self.frames_transferred = 0
        
        # Create windows
        cv2.namedWindow('Original Frame', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Received Frame', cv2.WINDOW_NORMAL)

    def capture_frame(self):
        frame = self.picam2.capture_array()
        frame = frame[::2, ::2, :3]  # Resize
        return np.clip(frame, 0, 255).astype(np.uint8)

    def transfer_image(self, frame):
        frame.tofile('image_data.bin')
        try:
            subprocess.run(['sudo', './gpio_transfer'], check=True)
            if os.path.exists('received_data.bin'):
                received_data = np.fromfile('received_data.bin', dtype=np.uint8)
                if received_data.size == frame.size:
                    return received_data.reshape(frame.shape)
        except subprocess.CalledProcessError:
            pass
        return None

    def run(self):
        try:
            while True:
                # Capture and transfer frame
                frame = self.capture_frame()
                self.total_bytes += frame.size
                
                received_frame = self.transfer_image(frame)
                if received_frame is not None:
                    self.frames_transferred += 1
                    
                    # Display frames
                    cv2.imshow('Original Frame', frame)
                    cv2.imshow('Received Frame', received_frame)
                    
                    # Process key events (press 'q' to quit)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                
                # Calculate and display stats
                elapsed_time = time.time() - self.start_time
                current_rate = self.total_bytes / (1024 * 1024 * elapsed_time)
                print(f"\rTime: {elapsed_time:.2f}s, Rate: {current_rate:.2f} MB/s, "
                      f"Frames: {self.frames_transferred}", end='')
                
                # Stop after 10 seconds
                if elapsed_time >= 10.0:
                    print("\n\nFinal Results:")
                    print(f"Total data transferred: {self.total_bytes / (1024*1024):.2f} MB")
                    print(f"Average rate: {current_rate:.2f} MB/s")
                    print(f"Frames transferred: {self.frames_transferred}")
                    # Keep the last frame displayed for a moment
                    cv2.waitKey(3000)
                    break

        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            self.cleanup()

    def cleanup(self):
        self.picam2.stop()
        cv2.destroyAllWindows()
        for file in ['image_data.bin', 'received_data.bin']:
            if os.path.exists(file):
                try:
                    os.remove(file)
                except Exception as e:
                    print(f"Warning: Could not remove {file}: {e}")

def main():
    transfer = ImageTransfer()
    transfer.run()

if __name__ == "__main__":
    main()
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
        cv2.resizeWindow('Original Frame', 320, 240)
        cv2.resizeWindow('Received Frame', 320, 240)

    def capture_frame(self):
        frame = self.picam2.capture_array()
        print(f"Captured frame shape: {frame.shape}, dtype: {frame.dtype}")
        print(f"Frame min/max values: {np.min(frame)}/{np.max(frame)}")
        
        # Ensure BGR format and proper size
        if len(frame.shape) == 3 and frame.shape[2] > 3:
            frame = frame[:, :, :3]
        
        # Resize for transfer
        frame = cv2.resize(frame, (160, 120))
        return frame.astype(np.uint8)

    def transfer_image(self, frame):
        print(f"Transferring frame shape: {frame.shape}, size: {frame.size} bytes")
        frame.tofile('image_data.bin')
        try:
            subprocess.run(['sudo', './gpio_transfer'], check=True)
            if os.path.exists('received_data.bin'):
                received_data = np.fromfile('received_data.bin', dtype=np.uint8)
                print(f"Received data size: {received_data.size} bytes")
                if received_data.size == frame.size:
                    received_frame = received_data.reshape(frame.shape)
                    print(f"Received frame min/max values: {np.min(received_frame)}/{np.max(received_frame)}")
                    return received_frame
        except subprocess.CalledProcessError as e:
            print(f"Transfer error: {e}")
        return None

    def run(self):
        try:
            while True:
                # Capture and transfer frame
                frame = self.capture_frame()
                if frame is None:
                    print("Failed to capture frame")
                    continue
                
                self.total_bytes += frame.size
                
                # Save original frame for debug
                cv2.imwrite('debug_original.jpg', frame)
                
                received_frame = self.transfer_image(frame)
                if received_frame is not None:
                    self.frames_transferred += 1
                    
                    # Save received frame for debug
                    cv2.imwrite('debug_received.jpg', received_frame)
                    
                    # Display frames
                    cv2.imshow('Original Frame', frame)
                    cv2.imshow('Received Frame', received_frame)
                    
                    # Process key events
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                else:
                    print("Failed to receive frame")
                
                # Calculate and display stats
                elapsed_time = time.time() - self.start_time
                current_rate = self.total_bytes / (1024 * 1024 * elapsed_time)
                print(f"\rTime: {elapsed_time:.2f}s, Rate: {current_rate:.2f} MB/s, "
                      f"Frames: {self.frames_transferred}")
                
                if elapsed_time >= 10.0:
                    print("\n\nFinal Results:")
                    print(f"Total data transferred: {self.total_bytes / (1024*1024):.2f} MB")
                    print(f"Average rate: {current_rate:.2f} MB/s")
                    print(f"Frames transferred: {self.frames_transferred}")
                    cv2.waitKey(3000)
                    break

        except KeyboardInterrupt:
            print("\nStopped by user")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        self.picam2.stop()
        cv2.destroyAllWindows()
        for file in ['image_data.bin', 'received_data.bin', 'debug_original.jpg', 'debug_received.jpg']:
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
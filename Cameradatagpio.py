from picamera2 import Picamera2
import RPi.GPIO as GPIO
import numpy as np
import time
import matplotlib.pyplot as plt

# GPIO Setup
TX_PIN = 17
RX_PIN = 26
DELAY = 0.01  # 10ms delay

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TX_PIN, GPIO.OUT)
    GPIO.setup(RX_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.output(TX_PIN, GPIO.LOW)
    print(f"GPIO setup complete: TX on GPIO {TX_PIN}, RX on GPIO {RX_PIN}")

def capture_frame():
    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (640, 480)})
    picam2.configure(config)
    picam2.start()
    time.sleep(2)
    
    frame = picam2.capture_array()
    picam2.stop()
    
    if len(frame.shape) == 3:
        frame = np.mean(frame, axis=2).astype(np.uint8)
    
    # Make the frame even smaller for faster transmission
    frame = frame[::40, ::40]  # This will give us roughly a 16x12 image
    return frame

def send_bit(bit):
    GPIO.output(TX_PIN, bit)
    #time.sleep(DELAY)  # Wait before next bit

def read_bit():
    bit = GPIO.input(RX_PIN)
    #time.sleep(DELAY)  # Wait before next bit
    return bit

def transmit_and_receive_byte(byte_value):
    received_bits = []
    
    # Convert byte to binary and transmit/receive bit by bit
    binary = format(byte_value, '08b')
    for bit in binary:
        # Transmit
        send_bit(int(bit))
        # Receive
        received_bit = read_bit()
        received_bits.append(str(received_bit))
    
    # Convert received bits back to byte
    received_byte = int(''.join(received_bits), 2)
    return received_byte

def transmit_and_receive_frame(frame):
    height, width = frame.shape
    received_data = np.zeros_like(frame)
    
    # First transmit frame dimensions
    print(f"\nTransmitting frame dimensions: {height}x{width}")
    height_byte = transmit_and_receive_byte(height)
    width_byte = transmit_and_receive_byte(width)
    print(f"Received dimensions: {height_byte}x{width_byte}")
    
    total_pixels = height * width
    transmitted_pixels = 0
    
    # Transmit each pixel value
    for i in range(height):
        for j in range(width):
            pixel_value = frame[i, j]
            received_value = transmit_and_receive_byte(pixel_value)
            received_data[i, j] = received_value
            
            transmitted_pixels += 1
            if transmitted_pixels % 10 == 0:  # Update progress every 10 pixels
                progress = (transmitted_pixels / total_pixels) * 100
                print(f"Progress: {progress:.1f}% ({transmitted_pixels}/{total_pixels} pixels)")
    
    return received_data

def display_images(original, received):
    plt.figure(figsize=(10, 5))
    
    plt.subplot(121)
    plt.imshow(original, cmap='gray')
    plt.title('Original Frame')
    plt.axis('off')
    
    plt.subplot(122)
    plt.imshow(received, cmap='gray')
    plt.title('Received Frame')
    plt.axis('off')
    
    plt.suptitle('Image Transmission Comparison')
    plt.tight_layout()
    plt.show()

def main():
    try:
        print("Starting program...")
        setup_gpio()
        
        # Test GPIO connection
        print("\nTesting GPIO connection...")
        send_bit(1)
        test_bit = read_bit()
        print(f"Test bit received: {test_bit}")
        if test_bit != 1:
            print("WARNING: GPIO connection test failed!")
            return
        
        input("\nPress Enter to start frame capture and transmission...")
        
        print("\nCapturing frame...")
        frame = capture_frame()
        print(f"Frame captured! Shape: {frame.shape}")
        
        print("\nStarting transmission and reception...")
        received_frame = transmit_and_receive_frame(frame)
        print("Transmission complete!")
        
        print("\nDisplaying images...")
        display_images(frame, received_frame)
        
        # Save images
        np.save('original_frame.npy', frame)
        np.save('received_frame.npy', received_frame)
        print("\nImages saved as .npy files")
        
        # Print transmission accuracy
        accuracy = np.mean(frame == received_frame) * 100
        print(f"\nTransmission accuracy: {accuracy:.1f}%")
        
    except Exception as e:
        print(f"\nERROR: {str(e)}")
        print("Program terminated due to error")
    
    finally:
        GPIO.cleanup()
        print("\nGPIO cleanup complete")

if __name__ == "__main__":
    main()
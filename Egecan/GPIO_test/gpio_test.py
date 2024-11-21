import RPi.GPIO as GPIO

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)

try:
    while True:
        GPIO.output(17, GPIO.HIGH)
        GPIO.output(17, GPIO.LOW)

except KeyboardInterrupt:
    GPIO.cleanup()
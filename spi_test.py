import spidev
import time

# Configure SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI0 (bus 0, chip select 0)
spi.max_speed_hz = 500000  # Set SPI clock speed (500kHz)

# GPIO for chip select (SS)
import RPi.GPIO as GPIO
SS_PIN = 8  # GPIO pin for slave select
GPIO.setmode(GPIO.BCM)
GPIO.setup(SS_PIN, GPIO.OUT)
GPIO.output(SS_PIN, GPIO.HIGH)

# Function to send data
def send_data(data):
    GPIO.output(SS_PIN, GPIO.LOW)  # Select the slave
    response = spi.xfer2([data])  # Send data
    GPIO.output(SS_PIN, GPIO.HIGH)  # Deselect the slave
    return response

try:
    while True:
        data_to_send = 0x55  # Example data to send (hex 0x55)
        print(f"Sending: {data_to_send}")
        response = send_data(data_to_send)
        print(f"Response: {response}")
        time.sleep(1)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    spi.close()
    GPIO.cleanup()

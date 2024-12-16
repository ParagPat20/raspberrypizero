import spidev
import time

# Create an SPI object
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device (CS) 0
spi.max_speed_hz = 50000  # Set the speed (50 kHz)
spi.mode = 0b00  # Set SPI mode to 0

try:
    while True:
        # Data to send
        data = [0x31, 0x42, 0x43, 0x54]  # Example data
        print(data)
        response = spi.xfer2(data)  # Send data and receive response
        print(data)
        print("Sent:", data, "Received:", response)
        time.sleep(1)  # Wait for 1 second

except KeyboardInterrupt:
    print("Exiting...")
finally:
    spi.close()  # Close the SPI connection
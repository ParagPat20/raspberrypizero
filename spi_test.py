import spidev
import time

# Create an SPI object
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device (CS) 0
spi.max_speed_hz = 50000  # Set the speed (50 kHz)

try:
    while True:
        # Data to send
        data = [0x01, 0x02, 0x03, 0x04]  # Example data
        response = spi.xfer2(data)  # Send data and receive response
        print("Sent:", data, "Received:", response)
        time.sleep(1)  # Wait for 1 second

except KeyboardInterrupt:
    print("Exiting...")
finally:
    spi.close()  # Close the SPI connection
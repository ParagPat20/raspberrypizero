import spidev
import time

# Create SPI instance
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, chip select 0 (CE0)
spi.max_speed_hz = 50000  # Set SPI clock speed
spi.mode = 0b00  # SPI mode 0

def send_message(message):
    # Convert string to bytes and send over SPI
    byte_message = [ord(char) for char in message]
    spi.xfer(byte_message)
    print("Sent: ", message)

def read_message(length):
    # Read the response from the SPI slave
    response = spi.readbytes(length)
    print("Received: ", "".join([chr(byte) for byte in response]))

try:
    while True:
        # Send a message to the ESP32
        send_message("Hello ESP32")
        time.sleep(1)

        # Read response from the ESP32 (assuming 16-byte response)
        read_message(16)
        time.sleep(1)

except KeyboardInterrupt:
    spi.close()
    print("SPI communication terminated.")

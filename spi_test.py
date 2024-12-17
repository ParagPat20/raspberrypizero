import spidev
import time

# Create SPI instance
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0
spi.max_speed_hz = 50000  # Set SPI clock speed
spi.mode = 0b00  # SPI mode 0

BUFFER_SIZE = 8

def transfer_data(send_buffer):
    # Send data to the slave and receive the response
    response = spi.xfer2(send_buffer)
    return response

try:
    while True:
        # Prepare a message to send to the slave
        send_buffer = [10, 20, 30, 40, 50, 60, 70, 80]
        print("Sending: ", send_buffer)

        # Transfer data and print the response
        response = transfer_data(send_buffer)
        print("Received: ", response)

        time.sleep(1)  # Optional: delay between transactions

except KeyboardInterrupt:
    spi.close()
    print("SPI communication terminated.")

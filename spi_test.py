import spidev
import time

# SPI Configuration
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, device 0
spi.max_speed_hz = 10000  # Lower speed for debugging
spi.mode = 0b00  # SPI mode 0

BUFFER_SIZE = 8

def transfer_data(send_buffer):
    response = spi.xfer2(send_buffer)
    return response

try:
    while True:
        # Send test data
        send_buffer = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80]
        print("Sending: ", send_buffer)

        # Receive response from slave
        response = transfer_data(send_buffer)
        print("Received: ", response)

        # Pause before the next transaction
        time.sleep(1)

except KeyboardInterrupt:
    spi.close()
    print("SPI communication terminated.")

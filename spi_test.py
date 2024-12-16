import spidev
import time

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Use CE0 (GPIO 8)
spi.max_speed_hz = 1000000  # 1 MHz SPI speed
spi.mode = 0b00  # SPI Mode 0 (CPOL=0, CPHA=0)

# Function to send and receive data
def spi_communication():
    sent_data = 0x01  # Data to send
    print(f"Sent: {hex(sent_data)}")
    
    # Perform SPI transaction
    received_data = spi.xfer2([sent_data])  # Send and receive data
    print(f"Received: {hex(received_data[0])}")
    
    if received_data[0] != 0:
        print("Communication successful!")
    else:
        print("No response or unexpected response!")


# Main loop
try:
    while True:
        spi_communication()
        time.sleep(1)  # Wait 1 second between transactions

except KeyboardInterrupt:
    print("Exiting SPI communication.")
    spi.close()

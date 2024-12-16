import spidev
import time

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, device 0 (CE0 on Pi Zero 2 W)
spi.max_speed_hz = 1000000  # Set speed to 1 MHz
spi.mode = 0b00  # SPI Mode 0 (CPOL=0, CPHA=0)

# Function to send and receive data
def spi_communication():
    # Send data to ESP32 (e.g., 0x01)
    sent_data = 0x01
    print(f"Sent: {hex(sent_data)}")
    
    # Receive response from ESP32
    received_data = spi.xfer2([sent_data])  # Send and receive data in a single transfer
    print(f"Received: {hex(received_data[0])}")
    
    # Optional: check if response is correct (e.g., 0x42)
    if received_data[0] == 0x42:
        print("Communication successful!")
    else:
        print("Unexpected response!")

# Main loop
try:
    while True:
        spi_communication()
        time.sleep(1)  # Wait 1 second between transfers

except KeyboardInterrupt:
    print("Exiting SPI communication.")
    spi.close()

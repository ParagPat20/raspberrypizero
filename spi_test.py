import spidev
import time

# Initialize SPI
spi = spidev.SpiDev()  # Create an SPI instance
spi.open(0, 0)         # Open bus 0, device (CS) 0
spi.max_speed_hz = 1000000  # 1 MHz
spi.mode = 0  # SPI Mode 0 (CPOL=0, CPHA=0)

def read_spi_data():
    # Read data from the SPI bus
    received_data = spi.xfer2([0x00])  # Send dummy byte to receive data
    return received_data[0]  # Return received byte

def main():
    try:
        while True:
            data = read_spi_data()
            print(f"Received: {data:08b}")  # Print received data as binary
            time.sleep(0.1)  # Small delay
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        spi.close()  # Clean up and close SPI

if __name__ == "__main__":
    main()

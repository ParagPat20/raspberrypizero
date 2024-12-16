import spidev
import time

# Open SPI bus
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, CE0 (Chip Enable 0)
spi.max_speed_hz = 1000000  # 1 MHz SPI speed

def send_data(data):
    """Send data to ESP32 via SPI"""
    try:
        # Send data and receive response
        response = spi.xfer2([data])
        return response[0]
    except Exception as e:
        print(f"SPI transmission error: {e}")
        return None

def main():
    while True:
        # Example: Send a command or data
        command = 0x34  # Example command
        response = send_data(command)
        
        if response is not None:
            print(f"Sent: {command}, Received: {response}")
        
        time.sleep(1)  # Wait for 1 second between transmissions

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("SPI communication stopped")
        spi.close()
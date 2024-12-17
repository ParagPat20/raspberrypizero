import smbus
import time

I2C_ADDRESS = 0x08  # I2C address for the ESP32 slave

# Open I2C bus 1 (GPIO 2, 3 for I2C1 on Raspberry Pi)
bus = smbus.SMBus(1)

def write_data(data):
    bus.write_byte(I2C_ADDRESS, data)  # Write a byte to the ESP32 slave

def read_data():
    data = bus.read_byte(I2C_ADDRESS)  # Read a byte from the ESP32 slave
    print(f"Data received: {data}")

try:
    while True:
        # Send data (e.g., 0x42) to ESP32
        print("Sending data to ESP32")
        write_data(0x42)

        # Read response from ESP32
        read_data()

        time.sleep(1)

except KeyboardInterrupt:
    print("Program terminated.")

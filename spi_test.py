import smbus
import time

# I2C bus and address
I2C_BUS = 1  # Default I2C bus on Raspberry Pi
I2C_ADDRESS = 0x55  # ESP32 Slave I2C address

# Initialize the I2C bus
bus = smbus.SMBus(I2C_BUS)

def read_data():
    # Read data from ESP32 slave (Request data from slave)
    try:
        data = bus.read_i2c_block_data(I2C_ADDRESS, 0, 32)  # Read up to 32 bytes
        data_str = ''.join(chr(byte) for byte in data if byte != 0)  # Convert bytes to string and avoid nulls (0)
        print(f"Received data: {data_str}")
    except IOError:
        print("Failed to read from I2C device.")

def send_data():
    # Send data to ESP32 slave (Write data to slave)
    try:
        # Sending simple byte data, equivalent to a 'request' for ESP32 to send something back
        bus.write_byte(I2C_ADDRESS, 0x01)  # Writing a byte to trigger onRequest
        print("Sent request to ESP32")
    except IOError:
        print("Failed to write to I2C device.")

def main():
    while True:
        send_data()  # Send request to ESP32
        read_data()   # Read response from ESP32

if __name__ == "__main__":
    main()

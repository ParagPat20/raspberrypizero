import serial
import time

# Configure the serial port
port = '/dev/ttyUSB0'  # Change this to your serial port (e.g., '/dev/ttyUSB0' on Linux)
baudrate = 115200  # Set the baud rate (must match the device's baud rate)

# Create a serial connection
ser = serial.Serial(port, baudrate, timeout=1)

# Allow some time for the connection to establish
time.sleep(2)

print("Starting to receive data...")

try:
    while True:
        if ser.in_waiting > 0:  # Check if there is data waiting in the buffer
            data = ser.readline().decode('utf-8').rstrip()  # Read a line of data
            print(f"Received: {data}")  # Print the received data

except KeyboardInterrupt:
    print("Exiting...")

finally:
    ser.close()  # Close the serial connection
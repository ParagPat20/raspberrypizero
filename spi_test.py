import RPi.GPIO as GPIO
import time

SCL_PIN = 11  # SCK (SCL)
SDA_PIN = 9   # MISO (SDA)

GPIO.setmode(GPIO.BCM)
GPIO.setup(SCL_PIN, GPIO.OUT)  # Set SCL as output (master)
GPIO.setup(SDA_PIN, GPIO.OUT)  # Set SDA as output (master)

def start_condition():
    GPIO.output(SDA_PIN, GPIO.HIGH)
    GPIO.output(SCL_PIN, GPIO.HIGH)
    time.sleep(0.000001)
    GPIO.output(SDA_PIN, GPIO.LOW)  # Start condition
    time.sleep(0.000001)
    GPIO.output(SCL_PIN, GPIO.LOW)  # Clock low

def stop_condition():
    GPIO.output(SDA_PIN, GPIO.LOW)
    GPIO.output(SCL_PIN, GPIO.HIGH)
    time.sleep(0.000001)
    GPIO.output(SDA_PIN, GPIO.HIGH)  # Stop condition

def write_byte(data):
    for i in range(8):
        GPIO.output(SDA_PIN, (data & (0x80 >> i)) != 0)  # Write MSB first
        time.sleep(0.000001)
        GPIO.output(SCL_PIN, GPIO.HIGH)  # Clock high
        time.sleep(0.000001)
        GPIO.output(SCL_PIN, GPIO.LOW)   # Clock low

def main():
    start_condition()  # Send start condition

    data_to_send = 0x42  # Example data to send
    write_byte(data_to_send)

    stop_condition()  # Send stop condition

    print(f"Sent data: {hex(data_to_send)}")

if __name__ == "__main__":
    try:
        while True:
            main()
            time.sleep(1)  # Delay for next transaction
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Program terminated.")

import time

import board

import neopixel



# LED strip configuration:

LED_COUNT = 16  # Total number of LEDs (4x4 matrix)

LED_PIN = 12     # GPIO pin connected to the LED strip

LED_BRIGHTNESS = 0.5  # Brightness level



# Create NeoPixel object with appropriate configuration

pixels = neopixel.NeoPixel(LED_PIN, LED_COUNT, brightness=LED_BRIGHTNESS)



# Function to set a single pixel color

def set_pixel(x, y, color):

    pixel_index = x + y * 4  # Calculate pixel index based on 2D coordinates

    pixels[pixel_index] = color



# Example usage:
def animate_leds():
    while True:

        # Set a single pixel to red

        set_pixel(1, 2, (255, 0, 0)) 



        # Set a whole row to blue

        for i in range(4):

            set_pixel(i, 0, (0, 0, 255))



        # Wait a short time before updating again

        time.sleep(0.1)
        

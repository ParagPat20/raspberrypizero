import mouse
import keyboard
import time
while True:
    if keyboard.is_pressed('r'):
        mouse.click()
        time.sleep(0.1)

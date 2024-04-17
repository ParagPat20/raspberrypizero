import picamera

try:
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)
        camera.capture('/home/oxi/test_image.jpg')  # Capture an image and save it
        print("Image captured successfully. Camera is working.")
except picamera.exc.PiCameraError as e:
    print("Error:", e)
    print("Camera is not working.")

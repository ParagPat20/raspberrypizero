import picamera

def check_camera():
    try:
        with picamera.PiCamera() as camera:
            camera.resolution = (640, 480)
            camera.start_preview()
            input("Press Enter to capture an image...")
            camera.capture('test_image.jpg')
            print("Image captured successfully.")
            camera.stop_preview()
    except picamera.exc.PiCameraError as e:
        print(f"An error occurred: {e}")
        return False
    return True

if __name__ == "__main__":
    if check_camera():
        print("Camera is working.")
    else:
        print("Camera is not working.")

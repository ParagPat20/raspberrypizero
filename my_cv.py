import io
import time
import picamera
import cv2
import numpy as np

# Create the in-memory stream
stream = io.BytesIO()

with picamera.PiCamera() as camera:
    camera.resolution = (640, 480)  # Set the resolution of the video frames
    camera.framerate = 24  # Set the frame rate of the video stream
    camera.start_preview()  # Start the camera preview

    # Wait for the camera to warm up
    time.sleep(2)

    while True:
        camera.capture(stream, format='jpeg')
        # Construct a numpy array from the stream
        data = np.frombuffer(stream.getvalue(), dtype=np.uint8)
        # "Decode" the image from the array, preserving color
        image = cv2.imdecode(data, 1)
        if image is True:
            print('yay')
        # Check for the 'q' key to exit the live stream
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Clear the stream in preparation for the next frame
        stream.seek(0)
        stream.truncate()

    # Clean up
    cv2.destroyAllWindows()

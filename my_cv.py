import random
import cv2
import numpy as np
from ultralytics import YOLO

# Opening the file in read mode
my_file = open("coco.txt", "r")
# Reading the file
data = my_file.read()
# Replacing and splitting the text when a newline ('\n') is seen.
class_list = data.split("\n")
my_file.close()

# Generate random colors for the class list
detection_colors = []
for i in range(len(class_list)):
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    detection_colors.append((b, g, r))

# Load a pretrained YOLOv8n model
model = YOLO("best.pt", "v8")

# Vals to resize video frames | small frame optimizes the run
frame_wid = 640
frame_hyt = 480

# Initialize the Pi Camera
import picamera
import io
import socket
import struct
import time

# Create a socket server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 8000))
server_socket.listen(0)

# Accept a single connection from the client
connection = server_socket.accept()[0].makefile('wb')

try:
    with picamera.PiCamera() as camera:
        camera.resolution = (640, 480)  # Adjust resolution as needed
        camera.framerate = 30  # Adjust frame rate as needed

        # Start capturing and sending the video feed
        time.sleep(2)  # Give the camera some time to warm up
        stream = io.BytesIO()
        for _ in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
            stream.seek(0)
            image_data = stream.read()

            # Send the image size to the client
            connection.write(struct.pack('<L', len(image_data)))
            connection.flush()

            # Send the image data to the client
            connection.write(image_data)
            stream.seek(0)
            stream.truncate()

            # Convert the image data to a numpy array
            frame = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), 1)

            # Predict on the image
            detect_params = model.predict(source=[frame], conf=0.45, save=False)

            # Convert tensor array to numpy
            DP = detect_params[0].numpy()
            print(DP)

            if len(DP) != 0:
                for i in range(len(detect_params[0])):
                    print(i)

                    boxes = detect_params[0].boxes
                    box = boxes[i]  # returns one box
                    clsID = box.cls.numpy()[0]
                    conf = box.conf.numpy()[0]
                    bb = box.xyxy.numpy()[0]

                    cv2.rectangle(
                        frame,
                        (int(bb[0]), int(bb[1])),
                        (int(bb[2]), int(bb[3])),
                        detection_colors[int(clsID)],
                        3,
                    )

                    # Display class name and confidence
                    font = cv2.FONT_HERSHEY_COMPLEX
                    cv2.putText(
                        frame,
                        class_list[int(clsID)] + " " + str(round(conf, 3)) + "%",
                        (int(bb[0]), int(bb[1]) - 10),
                        font,
                        1,
                        (255, 255, 255),
                        2,
                    )

            # Display the resulting frame
            cv2.imshow("ObjectDetection", frame)

            # Terminate the run when "Q" is pressed
            if cv2.waitKey(1) == ord("q"):
                break

finally:
    connection.close()
    server_socket.close()

import zmq
import cv2
import numpy as np

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.connect("tcp://192.168.67.229:5555")

while True:
    frame = socket.recv()
    socket.send_string('OK')
    nparr = np.frombuffer(frame, np.uint8)
    image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    cv2.imshow('Frame', image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

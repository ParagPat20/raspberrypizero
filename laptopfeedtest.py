import zmq
import cv2
import numpy as np

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://192.168.67.229:5555")
socket.setsockopt_string(zmq.SUBSCRIBE, '')

while True:
    frame = socket.recv()
    flag = socket.recv_string()
    if flag == 'latest':
        nparr = np.frombuffer(frame, np.uint8)
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        cv2.imshow('Frame', image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

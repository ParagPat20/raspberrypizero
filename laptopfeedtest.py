import socket
import cv2
import numpy as np

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('0.0.0.0', 8000))

connection = client_socket.makefile('rb')

try:
    while True:
        image_len = int.from_bytes(connection.read(4), byteorder='big')
        if not image_len:
            break
        image_data = connection.read(image_len)
        nparr = np.frombuffer(image_data, np.uint8)
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        cv2.imshow('Frame', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    connection.close()
    client_socket.close()
    cv2.destroyAllWindows()

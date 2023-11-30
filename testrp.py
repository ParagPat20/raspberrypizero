import socket
from drone import *

log("Hello!")
server = socket.socket()
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('192.168.190.101', 12345))
server.listen(100)
conn ,addr = server.accept()
while True:
    data = conn.recv(1024).decode('utf-8')
    data = str(data)
    log(data)
import socket
from drone import *

log("Hello!")
server = socket.socket()
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('192.168.190.122', 12345))
server.listen(100)

while True:
    conn ,addr = server.accept()
    data = conn.recv(1024).decode('utf-8')
    data = str(data)
    log("TESTRP @ {} : {} ".format(int(round(time.time() * 1000)),data))
    conn.close()
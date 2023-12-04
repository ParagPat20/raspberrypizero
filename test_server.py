import socket
import threading
import time

class Logger:
    def __init__(self):
        self.log_text = None
        self.mcu_status = False
        self.cd1_status = False
        self.cd2_status = False

    def set_log_text(self, log_text):
        self.log_text = log_text

    def log(self, message,foreground = "#5E95FF"):
        if self.log_text:
            self.log_text.configure(state="normal", font=("Montserrat Bold", 20 * -1), foreground=foreground)
            message = str(message)
            self.log_text.insert("end", message + "\n")
            self.log_text.configure(state="disabled")
            self.log_text.see("end")

def handle_client(logger1, conn, addr):
    try:
        data = conn.recv(1024).decode()
        data = str(data)
        logger1.log("{} from {}".format(data, addr))
        print("{} from {}".format(data, addr))

    except Exception as e:
        logger1.log('Error handling client: {}'.format(e))
    finally:
        conn.close()


def log_server(logger1=None, pc_host='192.168.170.101', port=61234):
    try:
        with socket.socket() as msg_socket:
            msg_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            print('1')
            msg_socket.bind((pc_host, port))
            print('2')
            msg_socket.listen(5)

            try:
                logger1.log('PC: {} - log_server() is started!'.format(time.ctime()))
                
                while True:
                    print("waiting for connection...")
                    logger1.log("waiting for connection...")
                    conn, addr = msg_socket.accept()
                    print("new connection established {}".format(conn, addr))
                    with conn:
                        client_thread = threading.Thread(target=handle_client, args=(logger1, conn, addr))
                        client_thread.start()
            
            except Exception as e:
                logger1.log('PC: log Error: {}'.format(e))
    except Exception as e:
        logger1.log('PC: Socekt Log Error: {}'.format(e))
# Singleton logger instance
logger = Logger()

log_server(logger)
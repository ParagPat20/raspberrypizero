from http.server import BaseHTTPRequestHandler, HTTPServer
import threading
import time
from drone import Drone

cmd_port = 12345
status_port = 60001
local_host = '192.168.207.122'  # Change this to your local host

MCU = None
MCU_initialized = False
d1 = None

drone_list = []

class MyRequestHandler(BaseHTTPRequestHandler):
    def _send_response(self, message):
        self.send_response(200)
        self.send_header("Content-type", "text/plain")
        self.end_headers()
        self.wfile.write(message.encode())

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length).decode('utf-8')
        
        global immediate_command_str
        immediate_command_str = post_data

        self._send_response("Command received successfully")

def execute_command(immediate_command_str):
    try:
        print('{} - Immediate command is: {}'.format(time.ctime(), immediate_command_str))
        exec(immediate_command_str)
    except Exception as e:
        print(f"MCU_Host: Error in execute_command: {e}")

def initialize_MCU():
    global d1, MCU, MCU_initialized
    if not MCU and not MCU_initialized:
        MCU = Drone('/dev/serial0', 115200)
        d1 = MCU
        d1_str = 'MCU'
        print("MCU Connected")
        threading.Thread(target=MCU.send_status, args=(local_host, 60001)).start()
        MCU_initialized = True
    print("MCU getting ready for the params...")
    time.sleep(2)  # Getting ready for params
    MCU.get_vehicle_state()
    print('mcu_status')

def start_mcu_host():
    server_address = ('', cmd_port)
    httpd = HTTPServer(server_address, MyRequestHandler)
    print(f"Starting HTTP server on port {cmd_port}")
    httpd.serve_forever()

if __name__ == "__main__":

    # Initialize MCU
    # initialize_MCU()

    # Start the HTTP server in a separate thread
    http_server_thread = threading.Thread(target=start_mcu_host)
    http_server_thread.start()

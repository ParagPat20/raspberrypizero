from drone import Drone
from drone import *


cmd_port = 12345
ctrl_port = 54321
status_port = 60002
local_host = CD1_host

CD1 = None

##################################################### Initialization #####################################################

CD1_initialized = False
d1 = None

context = zmq.Context()
msg_socket = context.socket(zmq.REP)
msg_socket.bind("tcp://{}:{}".format(CD1_host, cmd_port))

print('{} - SERVER_receive_and_execute_immediate_command() is started!'.format(time.ctime()))

def drone_list_update(cmd):
    try:
        global drone_list
        drone_list = cmd
        print(drone_list)
    except Exception as e:
        print(f"CD1_Host: Error in drone_list_update: {e}")

def execute_command(immediate_command_str):
    try:
        print('{} - Immediate command is: {}'.format(time.ctime(), immediate_command_str))
        exec(immediate_command_str)
    except Exception as e:
        print(f"CD1_Host: Error in execute_command: {e}")

##########################################################################################################################

def initialize_CD1():
    try:
        global d1, CD1, CD1_initialized
        if not CD1 and not CD1_initialized:
            CD1 = Drone('/dev/serial0', 115200)
            d1 = CD1
            d1_str = 'CD1'
            print("CD1 Connected")
            threading.Thread(target=CD1.send_status, args=(CD1_host,60002,)).start()
            CD1_initialized=True
        print("CD1 getting ready for the params...")
        time.sleep(2) #getting ready for params
        CD1.get_vehicle_state()
        log('CD1_status')
    except Exception as e:
        print(f"CD1_Host: Error in initialize_CD1: {e}")

##########################################################################################################################
print("Server started, have fun!")
##########################################################################################################################

while True:
    try:
        # Use zmq to receive messages
        immediate_command_str = msg_socket.recv_string()

        print('\n{} - Received immediate command: {}'.format(time.ctime(), immediate_command_str))

        # Use threading to run command execution in the background
        command_thread = threading.Thread(target=execute_command, args=(immediate_command_str,))
        command_thread.start()

    except Exception as e:
        print(f"Error: {e}")

##########################################################################################################################

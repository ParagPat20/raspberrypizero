from drone import Drone
from drone import *


cmd_port = 12345
ctrl_port = 54321
status_port = 60003
local_host = CD2_host

CD2 = None

##################################################### Initialization #####################################################

CD2_initialized = False
d1 = None

context = zmq.Context()
msg_socket = context.socket(zmq.REP)
msg_socket.bind("tcp://{}:{}".format(CD2_host, cmd_port))

print('{} - SERVER_receive_and_execute_immediate_command() is started!'.format(time.ctime()))

def drone_list_update(cmd):
    try:
        global drone_list
        drone_list = cmd
        print(drone_list)
    except Exception as e:
        print(f"CD2_Host: Error in drone_list_update: {e}")

def execute_command(immediate_command_str):
    try:
        print('{} - Immediate command is: {}'.format(time.ctime(), immediate_command_str))
        exec(immediate_command_str)

    except Exception as e:
        print(f"CD2_Host: Error in execute_command: {e}")

##########################################################################################################################

def initialize_CD2():
    try:
        global d1, CD2, CD2_initialized
        if not CD2 and not CD2_initialized:
            CD2 = Drone('/dev/serial0', 115200)
            d1 = CD2
            d1_str = 'CD2'
            print("CD2 Connected")
            # threading.Thread(target=CD2.send_status, args=(CD2_host,60003,)).start()
            CD2_initialized=True
        print("CD2 getting ready for the params...")
        time.sleep(2) #getting ready for params
        CD2.get_vehicle_state()
        log('CD2_status')
    except Exception as e:
        print(f"CD2_Host: Error in initialize_CD2: {e}")

##########################################################################################################################
print("Server started, have fun!")
##########################################################################################################################

while True:
    try:
        # Use zmq to receive messages
        immediate_command_str = msg_socket.recv_string()

        print('\n{} - Received immediate command: {}'.format(time.ctime(), immediate_command_str))
        command_thread = threading.Thread(target=execute_command, args=(immediate_command_str,))
        command_thread.start()
        msg_socket.send_string("OK!")

    except zmq.ZMQError as zmq_error:
        print(f"ZMQ Error: {zmq_error}")
    except Exception as e:
        print(f"Error: {e}")


##########################################################################################################################

from drone import Drone
from drone import *


cmd_port = 12345
ctrl_port = 54321
status_port = 60001
local_host = MCU_host

MCU = None

##################################################### Initialization #####################################################

MCU_initialized = False
d1 = None

context = zmq.Context()
msg_socket = context.socket(zmq.REP)
msg_socket.bind("tcp://{}:{}".format(MCU_host, cmd_port))

print('{} - SERVER_receive_and_execute_immediate_command() is started!'.format(time.ctime()))

def drone_list_update(cmd):
    try:
        global drone_list
        drone_list = cmd
        print(drone_list)
    except Exception as e:
        print(f"MCU_Host: Error in drone_list_update: {e}")

def execute_command(immediate_command_str):
    try:
        print('{} - Immediate command is: {}'.format(time.ctime(), immediate_command_str))
        exec(immediate_command_str)
    except Exception as e:
        print(f"MCU_Host: Error in execute_command: {e}")

##########################################################################################################################

def initialize_MCU():
    try:
        global d1, MCU, MCU_initialized
        if not MCU and not MCU_initialized:
            MCU = Drone('/dev/serial0', 115200)
            d1 = MCU
            d1_str = 'MCU'
            print("MCU Connected")
            threading.Thread(target=MCU.send_status, args=(MCU_host,60001,)).start()
            MCU_initialized=True
        print("MCU getting ready for the params...")
        time.sleep(2) #getting ready for params
        MCU.get_vehicle_state()
        log('mcu_status')
    except Exception as e:
        print(f"MCU_Host: Error in initialize_MCU: {e}")

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

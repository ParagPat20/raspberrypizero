from drone import Drone
from drone import *


cmd_port = 12345
ctrl_port = 54321
status_port = 60003
local_host = MCU_host

MCU = None

##################################################### Initialization #####################################################

MCU_initialized = False
d1 = None

context = zmq.Context()
msg_socket = context.socket(zmq.REP)
msg_socket.bind("tcp://{}:{}".format(MCU_host, cmd_port))

log('{} - SERVER_receive_and_execute_immediate_command() is started!'.format(time.ctime()))

def drone_list_update(cmd):
    try:
        global drone_list
        drone_list = cmd
        log(drone_list)
    except Exception as e:
        log(f"MCU_Host: Error in drone_list_update: {e}")

def execute_command(immediate_command_str):
    try:
        log('{} - Immediate command is: {}'.format(time.ctime(), immediate_command_str))
        exec(immediate_command_str)

    except Exception as e:
        log(f"MCU_Host: Error in execute_command: {e}")

##########################################################################################################################

def initialize_MCU():
    try:
        global d1, MCU, MCU_initialized
        if not MCU and not MCU_initialized:
            MCU = Drone('/dev/serial0', 115200)
            d1 = MCU
            d1_str = 'MCU'
            log("MCU Connected")
            # threading.Thread(target=MCU.send_status, args=(MCU_host,60003,)).start()
            MCU_initialized=True
        log("MCU getting ready for the params...")
        time.sleep(2) #getting ready for params
        MCU.get_vehicle_state()
        log('MCU_status')
    except Exception as e:
        log(f"MCU_Host: Error in initialize_MCU: {e}")

##########################################################################################################################
log("Server started, have fun!")
##########################################################################################################################

while True:
    try:
        # Use zmq to receive messages
        immediate_command_str = msg_socket.recv_string()

        log('\n{} - Received immediate command: {}'.format(time.ctime(), immediate_command_str))
        command_thread = threading.Thread(target=execute_command, args=(immediate_command_str,))
        command_thread.start()
        msg_socket.send_string("OK!")

    except zmq.ZMQError as zmq_error:
        log(f"ZMQ Error: {zmq_error}")
    except Exception as e:
        log(f"Error: {e}")


##########################################################################################################################

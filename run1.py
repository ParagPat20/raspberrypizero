from drone import Drone
from drone import *



cmd_port = 12345
ctrl_port = 54321
status_port = 60001
local_host = CD1_host

CD1 = None

##################################################### Initialization #####################################################

CD1_initialized = False
d1 = None

context = zmq.Context()
msg_socket = context.socket(zmq.PULL)
msg_socket.bind("tcp://*:12345")

log('{} - SERVER_receive_and_execute_immediate_command() is started!'.format(time.ctime()))

def drone_list_update(cmd):
    try:
        global drone_list
        drone_list = cmd
        log(drone_list)
    except Exception as e:
        log(f"CD1_Host: Error in drone_list_update: {e}")

def execute_command(immediate_command_str):
    try:
        exec(immediate_command_str)
        log('{} - command {} executed successfully'.format(time.ctime(), immediate_command_str))

    except Exception as e:
        log(f"CD1_Host: Error in execute_command: {e}")

##########################################################################################################################

def initialize_CD1():
    try:
        global d1, CD1, CD1_initialized
        if not CD1 and not CD1_initialized:
            d1_str = 'CD1'
            CD1 = Drone(d1_str,'/dev/serial0', 115200)
            d1 = CD1
            log("CD1 Connected")
            time.sleep(2)
            CD1.get_vehicle_state()
            # threading.Thread(target=CD1.send_status, args=(CD1_host,60003,)).start()
            threading.Thread(target=CD1.security).start()
            CD1_initialized=True
            
        log("CD1 getting ready for the params...")
        time.sleep(2) #getting ready for params
        CD1.get_vehicle_state()
        log('CD1_status')
    except Exception as e:
        log(f"CD1_Host: Error in initialize_CD1: {e}")

##########################################################################################################################
log("CD1 Server started, have fun!")
##########################################################################################################################

while True:
    try:
        immediate_command_str = msg_socket.recv_string()
        print('\n{} - Received immediate command: {}'.format(time.ctime(), immediate_command_str))
        command_thread = threading.Thread(target=execute_command, args=(immediate_command_str,))
        command_thread.start()

    except zmq.ZMQError as zmq_error:
        log(f"ZMQ Error: {zmq_error}")
    except Exception as e:
        log(f"Error: {e}")
    except KeyboardInterrupt:
        log("KeyboardInterrupt")
        msg_socket.close()



##########################################################################################################################

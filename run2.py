from drone import Drone
import zmq
import time
import threading
from drone import *

cmd_port = 12345
ctrl_port = 54321
status_port = 60003
local_host = CD2_host  # Make sure CD2_host is defined somewhere

CD2 = None

##################################################### Initialization #####################################################

CD2_initialized = False
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
        log(f"CD2_Host: Error in drone_list_update: {e}")

def execute_command(immediate_command_str):
    try:
        exec(immediate_command_str)
        log('{} - command {} executed successfully'.format(time.ctime(), immediate_command_str))

    except Exception as e:
        log(f"CD2_Host: Error in execute_command: {e}")

##########################################################################################################################

def initialize_CD2():
    try:
        global d1, CD2, CD2_initialized
        if not CD2 and not CD2_initialized:
            CD2 = Drone('/dev/serial0', 115200)
            d1 = CD2
            d1_str = 'CD2'
            log("CD2 Connected")
            # threading.Thread(target=CD2.send_status, args=(CD2_host,60003,)).start()
            threading.Thread(target=CD2.security).start()
            CD2_initialized=True
            
        log("CD2 getting ready for the params...")
        time.sleep(2) #getting ready for params
        CD2.get_vehicle_state()
        log('CD2_status')
    except Exception as e:
        log(f"CD2_Host: Error in initialize_CD2: {e}")

##########################################################################################################################
log("CD2 Server started, have fun!")
##########################################################################################################################

try:
    while True:
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
finally:
    msg_socket.close()
    context.term()

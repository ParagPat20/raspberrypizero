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
msg_socket = context.socket(zmq.PULL)
msg_socket.bind("tcp://*:12345")

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
        print("Executing command:", repr(immediate_command_str))  # Add this line
        exec(immediate_command_str)
        log('{} - Command executed successfully'.format(time.ctime()))

    except Exception as e:
        log('{} - Error in execute_command: {}'.format(time.ctime(), e))


def run_mis(filename):
    try:
        # Open the mission file
        with open(f"{filename}.txt", 'r') as file:
            # Read each line from the file
            for line in file:
                # Skip empty lines
                if not line.strip():
                    continue
                line=str(line)

                # Execute the command
                exec(line)  # Assuming each line is a command

    except Exception as e:
        log(f"Error in run_mis: {e}")


##########################################################################################################################

def initialize_MCU():
    try:
        global d1, MCU, MCU_initialized
        if not MCU and not MCU_initialized:
            d1_str = 'MCU'
            MCU = Drone(d1_str,'/dev/serial0', 115200)
            d1 = MCU
            log("MCU Connected")
            time.sleep(2)
            MCU.get_vehicle_state()
            # threading.Thread(target=MCU.send_status, args=(MCU_host,60003,)).start()
            threading.Thread(target=MCU.security).start()
            MCU_initialized=True
            
        log("MCU getting ready for the params...")
        time.sleep(2) #getting ready for params
        MCU.get_vehicle_state()
        log('MCU_status')
    except Exception as e:
        log(f"MCU_Host: Error in initialize_MCU: {e}")

##########################################################################################################################
log("MCU Server started, have fun!")
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

from drone import Drone
from drone import *
import socket

# Get hostname to determine which drones to initialize
hostname = socket.gethostname()
print(f"Host Name : {hostname}")
cmd_port = 12345
ctrl_port = 54321

# Initialize drone variables
MCU = None
CD1 = None 
CD2 = None
d1 = None

# Track initialization status
MCU_initialized = False
CD1_initialized = False
CD2_initialized = False

# Configure which drones to initialize based on hostname
DRONE_CONFIG = {
    'mcu-raspberry': ['MCU'],
    'cd1-raspberry': ['CD1'],
    'cd2-raspberry': ['CD2']
}

# Setup ZMQ socket
context = zmq.Context(10)
msg_socket = context.socket(zmq.PULL)
msg_socket.bind("tcp://*:12345")
msg_socket.setsockopt(zmq.RCVHWM, 1000)

poller = zmq.Poller()
poller.register(msg_socket, zmq.POLLIN)

log('{} - SERVER_receive_and_execute_immediate_command() is started!'.format(time.ctime()))

def drone_list_update(cmd):
    try:
        global drone_list
        drone_list = cmd
        log(drone_list)
    except Exception as e:
        log(f"{hostname}: Error in drone_list_update: {e}")

def execute_command(immediate_command_str):
    try:
        log("Executing command: {}".format(repr(immediate_command_str)))
        exec(immediate_command_str)
        log('{} - Command executed successfully'.format(time.ctime()))
    except Exception as e:
        log('{} - Error in execute_command: {}'.format(time.ctime(), e))

# Initialization functions for each drone
def initialize_MCU():
    try:
        global d1, MCU, MCU_initialized
        if not MCU and not MCU_initialized:
            d1_str = 'MCU'
            MCU = Drone(d1_str, '/dev/serial0', 115200)
            d1 = MCU
            log("MCU Connected")
            time.sleep(5)
            MCU.get_vehicle_state()
            threading.Thread(target=MCU.security).start()
            MCU_initialized = True
        MCU.get_vehicle_state()
        log('MCU_status')
    except Exception as e:
        log(f"Error in initialize_MCU: {e}")

def initialize_CD1():
    try:
        global d1, CD1, CD1_initialized
        if not CD1 and not CD1_initialized:
            d1_str = 'CD1'
            CD1 = Drone(d1_str,'/dev/serial0', 115200)
            d1 = CD1
            log("CD1 Connected")
            time.sleep(5)
            CD1.get_vehicle_state()
            threading.Thread(target=CD1.security).start()
            CD1_initialized=True
        CD1.get_vehicle_state()
        log('CD1_status')
    except Exception as e:
        log(f"Error in initialize_CD1: {e}")

def initialize_CD2():
    try:
        global d1, CD2, CD2_initialized
        if not CD2 and not CD2_initialized:
            d1_str = 'CD2'
            CD2 = Drone(d1_str,'/dev/serial0', 115200)
            d1 = CD2
            log("CD2 Connected")
            time.sleep(5)
            CD2.get_vehicle_state()
            threading.Thread(target=CD2.security).start()
            CD2_initialized=True
        CD2.get_vehicle_state()
        log('CD2_status')
    except Exception as e:
        log(f"Error in initialize_CD2: {e}")

def initialize_drones():
    """Initialize drones based on hostname configuration"""
    if hostname not in DRONE_CONFIG:
        log(f"Unknown hostname: {hostname}")
        return
    
    drones_to_init = DRONE_CONFIG[hostname]
    
    for drone in drones_to_init:
        if drone == 'MCU':
            initialize_MCU()
        elif drone == 'CD1':
            initialize_CD1()
        elif drone == 'CD2':
            initialize_CD2()
            
    log(f"Initialized drones for {hostname}: {drones_to_init}")

def run_mis(filename):
    try:
        with open(f"{filename}.txt", 'r') as file:
            for line in file:
                if not line.strip():
                    continue
                line = str(line)
                exec(line)
    except Exception as e:
        log(f"Error in run_mis: {e}")

# Initialize appropriate drones based on hostname
initialize_drones()

log(f"{hostname} Server started, have fun!")

# Main loop
while True:
    socks = dict(poller.poll())

    if msg_socket in socks and socks[msg_socket] == zmq.POLLIN:
        try:
            immediate_command_str = msg_socket.recv(zmq.NOBLOCK)
            immediate_command_str = immediate_command_str.decode()
            command_thread = threading.Thread(target=execute_command, args=(immediate_command_str,))
            command_thread.start()

        except zmq.error.Again:
            pass

        except zmq.ZMQError as zmq_error:
            log(f"ZMQ Error: {zmq_error}")
            msg_socket.close()
            msg_socket = context.socket(zmq.PULL)
            msg_socket.bind("tcp://*:12345")
            poller.register(msg_socket, zmq.POLLIN)

        except Exception as e:
            log(f"Error: {e}")

    if KeyboardInterrupt:
        log("KeyboardInterrupt")
        msg_socket.close()


##########################################################################################################################

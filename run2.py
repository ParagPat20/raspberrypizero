from drone2 import Drone
from drone2 import *



cmd_port = 12345
ctrl_port = 54321

CD2 = None
CD3 = None
d1_str = ""
d2_str = ""

##################################################### Initialization #####################################################

CD2_initialized = False
d1 = None
d2 = None
CD3_initialized = False
context = zmq.Context(10)  # Allow up to 10 concurrent sockets
msg_socket = context.socket(zmq.PULL)
msg_socket.bind("tcp://*:12345")
msg_socket.setsockopt(zmq.RCVHWM, 1000)  # High water mark for incoming messages

poller = zmq.Poller()
poller.register(msg_socket, zmq.POLLIN)  # Monitor for incoming messages

log('{} - SERVER_receive_and_execute_immediate_command() is started!'.format(time.ctime()))

def drone_list_update(cmd):
    try:
        global drone_list
        drone_list = cmd
        log(drone_list)
    except Exception as e:
        log("CD2_Host: Error in drone_list_update: {}".format(e))

def execute_command(immediate_command_str):
    try:
        log("Executing command: {}".format(repr(immediate_command_str)))
        exec(immediate_command_str)
        log('{} - Command executed successfully'.format(time.ctime()))

    except Exception as e:
        log('{} - Error in execute_command: {}'.format(time.ctime(), e))



##########################################################################################################################

def initialize_CD2():
    try:
        global d1, CD2, CD2_initialized
        if not CD2 and not CD2_initialized:
            d1_str = 'CD2'
            CD2 = Drone(d1_str,'/dev/serial0', 115200)
            # CD2 = Drone(d1_str,'COM6',115200)
            d1 = CD2
            log("CD2 Connected")
            time.sleep(5)
            log("CD2 getting ready for the params...")
            CD2.get_vehicle_state()
            threading.Thread(target=CD2.security).start()
            CD2_initialized=True
        CD2.get_vehicle_state()
        log('CD2_status')
    except Exception as e:
        log("CD2_Host: Error in initialize_CD2: {}".format(e))

def initialize_CD3():
    try:
        global d2, CD3, CD3_initialized
        if not CD3 and not CD3_initialized:
            d2_str = 'CD3'
            CD3 = Drone(d2_str,'172.217.28.254:14553')
            d2 = CD3
            log("CD3 Connected")
            time.sleep(5)  # Adjust sleep time if needed
            log("CD3 getting ready for the params...")
            CD3.get_vehicle_state()
            threading.Thread(target=CD3.security).start()
            CD3_initialized = True
        CD3.get_vehicle_state()
        log('CD3_status')
    except Exception as e:
        log("CD3_Host: Error in initialize_CD3: {}".format(e))

def deinitialize_CD2():
    try:
        global d1, CD2, CD2_initialized
        CD2.name = "STOP"
        CD2.exit()
        CD2 = None
        d1 = None
        CD2_initialized = False
        

    except Exception as e:
        log("CD2_Host: Error in deinitialize_CD2: {}".format(e))

def deinitialize_CD3():
    try:
        global d2, CD3, CD3_initialized
        CD3.name = "STOP"
        CD3.exit()
        CD3 = None
        d2 = None
        CD3_initialized = False
        

    except Exception as e:
        log("CD3_Host: Error in deinitialize_CD3: {}".format(e))


##########################################################################################################################
log("CD2 Server started, have fun!")
log("CD3 Server started, have fun!")

while True:
    socks = dict(poller.poll())

    if msg_socket in socks and socks[msg_socket] == zmq.POLLIN:
        try:
            immediate_command_str = msg_socket.recv(zmq.NOBLOCK)
            immediate_command_str = immediate_command_str.decode()
            command_thread = threading.Thread(target=execute_command, args=(immediate_command_str,))
            command_thread.start()

        except zmq.error.Again:  # Handle non-blocking recv errors
            pass  # Wait for next poll event

        except zmq.ZMQError as zmq_error:
            log("ZMQ Error: {}".format(zmq_error))
            msg_socket.close()  # Recreate socket on ZMQ errors
            msg_socket = context.socket(zmq.PULL)
            msg_socket.bind("tcp://*:12345")
            poller.register(msg_socket, zmq.POLLIN)

        except Exception as e:
            log("Error: {}".fromat(e))

if KeyboardInterrupt:
    log("KeyboardInterrupt")
    msg_socket.close()


##########################################################################################################################

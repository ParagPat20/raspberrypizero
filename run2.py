from drone import Drone
from drone import *

cmd_port = 12345
ctrl_port = 54321
status_port = [60003, 60004]
local_host = CD2_host

CD2 = None
CD3 = None

######################################################## FORMATIONS ########################################################

def LINE(dis=1, alt=2):

    # CD2 #
    mcul,mcua,mcuh = recv_status(MCU_host,60001)
    print(mcul,mcua,mcuh)
    CD2.arm('GUIDED')
    CD2.takeoff(2)
    CD2.yaw(0)
    cd2l,cd2h = cu_lo(CD2)
    print(f"First Drone's Lat: {cd2l[0]} & Lon: {cd2l[1]} & Heading: {cd2h}")
    cdis = dis*2
    Cl=new_coords(mcul,cdis,0)
    CD2.goto(Cl,alt)
    CD2.yaw(0)
    CD2.poshold()


    cdis = dis * 1
    CD3.arm('GUIDED')
    CD3.takeoff(2)
    CD3.yaw(0)
    cd3l,cd3h = cu_lo(CD3)
    print(f"Second Drone's Lat: {cd3l[0]} & Lon: {cd3l[1]} & Heading: {cd3h}")
    Bl= new_coords(cd2l,cdis,0)
    CD3.goto(Bl,alt)
    CD3.yaw(0)
    CD3.poshold()

    send(MCU_host,'chat("LINECOMPLETE")')

def SQUARE(dis=1, alt=2):

    # CD2 #
    CD2.yaw(0)
    cd2l,cd2h = cu_lo(CD2)
    print(f"First Drone's Lat: {cd2l[0]} & Lon: {cd2l[1]} & Heading: {cd2h}")
    CD2.poshold()

    cdis = dis*1
    CD3.yaw(0)
    cd3l,cd3h = cu_lo(CD3)
    print(f"Second Drone's Lat: {cd3l[0]} & Lon: {cd3l[1]} & Heading: {cd3h}")
    Bl= new_coords(cd2l,cdis,90)
    CD3.goto(Bl,alt)
    CD3.yaw(0)
    CD3.poshold()

    send(MCU_host,'chat("SQAURECOMPLETE")')

##########################################################################################################################

##################################################### Initialization #####################################################

CD2_initialized = False
CD3_initialized = False
d1 = None
d2 = None

def drone_list_update(cmd):
    global drone_list
    drone_list = cmd
    print(drone_list)

msg_socket = socket.socket()
msg_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
msg_socket.bind((local_host, cmd_port))
msg_socket.listen(5)
print('{} - SERVER_receive_and_execute_immediate_command() is started!'.format(time.ctime()))

def execute_command(immediate_command_str):
    try:
        print('{} - Immediate command is: {}'.format(time.ctime(), immediate_command_str))
        exec(immediate_command_str)
    except Exception as e:
        print(f"Error: {e}")

##########################################################################################################################

def initialize_CD2():
    try:
        global d1, CD2, CD2_initialized
        if not CD2 and not CD2_initialized:
            CD2 = Drone('/dev/serial0', 115200)
            d1 = CD2
            d1_str = 'CD2'
            print("CD2 Connected")
            threading.Thread(target=CD2.send_status, args=(status_port[0],)).start()
            CD2_initialized=True
        CD2.get_vehicle_state()
    except Exception as e:
        log(f"Error in initialize_CD2: {e}")

def initialize_CD3():
    try:
        global d1, CD3, CD3_initialized
        if not CD3 and not CD3_initialized:
            CD3 = Drone('/dev/serial0', 115200)
            d1 = CD3
            d1_str = 'CD3'
            print("CD3 Connected")
            threading.Thread(target=CD3.send_status, args=(status_port[0],)).start()
            CD3_initialized=True
        CD3.get_vehicle_state()
    except Exception as e:
        log(f"Error in initialize_CD3: {e}")

##########################################################################################################################
try:
    log("Starting CD2_host at {}".format(socket.gethostbyname(socket.gethostname())))
except Exception as e:
    print(f"Error: {e}")

while True:
    try:
        client_connection, client_address = msg_socket.accept()
        print('\n{} - Received immediate command from {}.'.format(time.ctime(), client_address))
        immediate_command_str = client_connection.recv(1024).decode()

        # Use threading to run command execution in the background
        command_thread = threading.Thread(target=execute_command, args=(immediate_command_str,))
        command_thread.start()

        ack = "Received Command: " + str(immediate_command_str)
        client_connection.send(ack.encode())
        
    except Exception as e:
        print(f"Error: {e}")

##########################################################################################################################

from drone import Drone
from drone import *

local_host = '192.168.190.43'
cmd_port = 12345
ctrl_port = 54321
status_port = [60003, 60004]
local_host = CD2_host

CD2 = None
CD3 = None

#FORMATIONS#

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
        ack = "Received Command: " + str(immediate_command_str)
        client_connection.send(ack.encode())
    except Exception as e:
        print(f"Error: {e}")

while True:
    try:
        if "CD2" in drone_list and d1 is None and not CD2_initialized:
            CD2 = Drone(status_port[0], '/dev/serial0', 115200)
            d1 = CD2
            d1_str = 'CD2'
            print("CD2 Connected")
            CD2_initialized = True
            CD2.get_vehicle_state()
        elif "CD2" not in drone_list and d1 is not None:  # Use 'and' instead of '&'
            d1.exit()
            d1 = None
            time.sleep(2)

        if "CD3" in drone_list and d2 is None and not CD3_initialized:
            CD3 = Drone(status_port[1], '0.0.0.0:14553')
            d2 = CD3
            d2_str = 'CD3'
            print("CD3 Connected")
            CD3_initialized = True
            CD3.get_vehicle_state()
        elif "CD3" not in drone_list and d2 is not None:  # Use 'and' instead of '&'
            d2.exit()
            d2 = None
            time.sleep(2)

        client_connection, client_address = msg_socket.accept()
        print('\n{} - Received immediate command from {}.'.format(time.ctime(), client_address))
        immediate_command_str = client_connection.recv(1024).decode()

        # Use threading to run command execution in the background
        command_thread = threading.Thread(target=execute_command, args=(immediate_command_str,))
        command_thread.start()
        
    except Exception as e:
        print(f"Error: {e}")

        


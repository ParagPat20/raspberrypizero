from drone import Drone
from drone import *


cmd_port = 12345
ctrl_port = 54321
status_port = [60005, 60006]
local_host = CD4_host

CD4 = None
CD5 = None

##################################################### Initialization #####################################################

MCU_initialized = False
CD1_initialized = False
d1 = None
d2 = None

msg_socket = socket.socket()
msg_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
msg_socket.bind((local_host, cmd_port))
msg_socket.listen(5)
print('{} - SERVER_receive_and_execute_immediate_command() is started!'.format(time.ctime()))



def drone_list_update(cmd):
    try:
        global drone_list
        drone_list = cmd
        print(drone_list)
    except Exception as e:
        log(f"CD4_Host: Error in drone_list_update: {e}")

def execute_command(immediate_command_str):
    try:
        print('{} - Immediate command is: {}'.format(time.ctime(), immediate_command_str))
        exec(immediate_command_str)
    except Exception as e:
        log(f"CD4_Host: Error in execute_command: {e}")

##########################################################################################################################

def initialize_CD4():
    try:
        global d1, CD4, CD4_initialized
        if not CD4 and not CD4_initialized:
            CD4 = Drone('/dev/serial0', 115200)
            d1 = CD4
            d1_str = 'CD4'
            print("CD4 Connected")
            # threading.Thread(target=CD4.send_status, args=(CD4_host,60001,)).start()
            CD4_initialized=True
        log("CD4 getting ready for the params...")
        time.sleep(2) #getting ready for params
        CD4.get_vehicle_state()
    except Exception as e:
        log(f"CD4_Host: Error in initialize_CD4: {e}")

# def initialize_CD1():
#     try:
#         global d1, CD1, CD1_initialized
#         if not CD1 and not CD1_initialized:
#             CD1 = Drone('0.0.0.0:14552')
#             d1 = CD1
#             d1_str = 'CD1'
#             print("CD1 Connected")
#             # threading.Thread(target=CD1.send_status, args=(CD4_host,60002,)).start()
#             CD1_initialized=True
#         time.sleep(2) # time to get params
#         CD1.get_vehicle_state()
#     except Exception as e:
#         log(f"CD4_Host: Error in initialize_CD1: {e}")

##########################################################################################################################
print("Sending IP to Computer, please start the computer")
try:
    log("Starting CD4_host at {}".format(socket.gethostbyname(socket.gethostname())))
except Exception as e:
    print(f"Error: {e}")
print("Cheers! Server is already going on!")
##########################################################################################################################
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


# # FORMATIONS
# ######################################################## FORMATIONS ########################################################

# def LINE(dis=1, alt=2):
#     try:
#         # MCU #
#         MCU.arm('GUIDED')
#         MCU.takeoff(2)
#         MCU.yaw(0)
#         mcul, mcuh = cu_lo(MCU)
#         print(f"First Drone's Lat: {mcul[0]} & Lon: {mcul[1]} & Heading: {mcuh}")
#         MCU.poshold()

#         # CD1 #
#         cdis = dis * 1
#         CD1.arm('GUIDED')
#         CD1.takeoff(2)
#         CD1.yaw(0)
#         cd1l, cd1h = cu_lo(CD1)
#         print(f"Second Drone's Lat: {cd1l[0]} & Lon: {cd1l[1]} & Heading: {cd1h}")
#         Bl = new_coords(mcul, cdis, 0)
#         CD1.goto(Bl, alt)
#         CD1.yaw(0)
#         CD1.poshold()

#         send(CD2_host, 'LINE(' + str(dis) + ',' + str(alt) + ')')
#     except Exception as e:
#         log(f"MCU_Host: Error in LINE formation: {e}")

# def SQUARE(dis=1, alt=2):
#     try:
#         if in_line:
#             # MCU #
#             MCU.yaw(0)
#             mcul, mcuh = cu_lo(MCU)
#             print(f"First Drone's Lat: {mcul[0]} & Lon: {mcul[1]} & Heading: {mcuh}")
#             MCU.poshold()

#             # CD1 #
#             cdis = dis * 1
#             CD1.yaw(0)
#             cd1l, cd1h = cu_lo(CD1)
#             print(f"Second Drone's Lat: {cd1l[0]} & Lon: {cd1l[1]} & Heading: {cd1h}")
#             Bl = new_coords(mcul, cdis, 90)
#             CD1.goto(Bl, alt)
#             CD1.yaw(0)
#             CD1.poshold()

#             send(CD2_host, '(' + str(dis) + ',' + str(alt) + ')')
#             in_line = False
#         else:
#             print("They are not in line, Run LINE(1,2)")
#     except Exception as e:
#         log(f"MCU_Host: Error in SQUARE formation: {e}")

# def custom_goto(cmd):
#     try:
#         print(cmd)
#         eval(cmd)
#         drone1 = cmd[0]
#         drone2 = cmd[1]
#         distance = cmd[2]
#         angle = cmd[3]
#         alt = cmd[4]

#         if drone1 == 1:
#             d1l, d1h = cu_lo(MCU)
#         elif drone1 == 2:
#             d1l, d1h == cu_lo(CD1)
#         elif drone1 == 3:
#             d1l, a, d1h = recv_status(CD2_host, 60003)
#         elif drone1 == 4:
#             d1l, a, d1h = recv_status(CD2_host, 60004)

#         print(d1l, d1h)

#         nl = new_coords(d1l, distance, angle)

#         if drone2 == 1:
#             MCU.goto(nl, alt)
#         elif drone2 == 2:
#             CD1.goto(nl, alt)
#         elif drone2 == 3:
#             send(CD2_host, 'CD2.goto(' + str(nl) + ')')
#         elif drone2 == 4:
#             d2l, a, d2h = recv_status(CD2_host, 60004)
#             send(CD2_host, 'CD3.goto(' + str(nl) + ')')
#     except Exception as e:
#         log(f"MCU_Host: Error in custom_goto: {e}")

##########################################################################################################################

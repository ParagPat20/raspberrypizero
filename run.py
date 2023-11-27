from drone import Drone
from drone import *

local_host = '192.168.190.122'
cmd_port = 12345
ctrl_port = 54321
status_port = [60001, 60002]

MCU = None
CD1 = None

#FORMATIONS#

def LINE(dis=1, alt=2):

    # MCU #
    MCU.arm('GUIDED')
    MCU.takeoff(2)
    MCU.yaw(0)
    mcul,mcuh = cu_lo(MCU)
    print(f"First Drone's Lat: {mcul[0]} & Lon: {mcul[1]} & Heading: {mcuh}")
    MCU.poshold()
    
    # CD1 #
    cdis = dis * 1
    CD1.arm('GUIDED')
    CD1.takeoff(2)
    CD1.yaw(0)
    cd1l,cd1h = cu_lo(CD1)
    print(f"Second Drone's Lat: {cd1l[0]} & Lon: {cd1l[1]} & Heading: {cd1h}")
    Bl= new_coords(mcul,cdis,0)
    CD1.goto(Bl,alt)
    CD1.yaw(0)
    CD1.poshold()

    send(CD2_host,'LINE('+str(dis)+','+str(alt)+')')

def SQUARE(dis=1, alt=2):

    if in_line:

        # MCU #
        MCU.yaw(0)
        mcul,mcuh = cu_lo(MCU)
        print(f"First Drone's Lat: {mcul[0]} & Lon: {mcul[1]} & Heading: {mcuh}")
        MCU.poshold()

        # CD1 #
        cdis = dis * 1
        CD1.yaw(0)
        cd1l,cd1h = cu_lo(CD1)
        print(f"Second Drone's Lat: {cd1l[0]} & Lon: {cd1l[1]} & Heading: {cd1h}")
        Bl= new_coords(mcul,cdis,90)
        CD1.goto(Bl,alt)
        CD1.yaw(0)
        CD1.poshold()

        send(CD2_host,'('+str(dis)+','+str(alt)+')')
        in_line = False
    
    else:
        print("They are not in line, Run LINE(1,2)")


def custom_goto(cmd):
    print(cmd)
    eval(cmd)
    drone1 = cmd[0]
    drone2 = cmd[1]
    distance = cmd[2]
    angle = cmd[3]
    alt = cmd[4]

    if drone1 == 1:
        d1l,d1h = cu_lo(MCU)
    elif drone1 == 2:
        d1l,d1h == cu_lo(CD1)
    elif drone1 == 3:
        d1l,a,d1h = recv_status(CD2_host,60003)
    elif drone1 == 4:
        d1l,a,d1h = recv_status(CD2_host,60004)

    print(d1l,d1h)

    nl = new_coords(d1l, distance, angle)

    if drone2 == 1:
        MCU.goto(nl,alt)
    elif drone2 == 2:
        CD1.goto(nl,alt)
    elif drone2 == 3:
        send(CD2_host,'CD2.goto('+str(nl)+')')
    elif drone2 == 4:
        d2l,a,d2h = recv_status(CD2_host,60004)
        send(CD2_host,'CD3.goto('+str(nl)+')')











server=threading.Thread(target=server_receive_and_execute_immediate_command,args=(local_host,))
server.start()
while True:
    try:
        if "MCU" in drone_list and d1 is None:
            MCU = Drone(status_port[0], '/dev/serial0', 115200)
            d1 = MCU
            d1_str = 'MCU'
            print("MCU COnnected")
        elif "MCU" not in drone_list and d1 is not None:  # Use 'and' instead of '&'
            d1.exit()
            d1 = None
            time.sleep(2)

        if "CD1" in drone_list and d2 is None:
            CD1 = Drone(status_port[1], '0.0.0.0:14550')
            d2 = CD1
            d2_str = 'CD1'
            print("CD1 Connected")
        elif "CD1" not in drone_list and d2 is not None:  # Use 'and' instead of '&'
            d2.exit()
            d2 = None
            time.sleep(2)
        
        if wait_for_command == False:
            exec(immediate_command_str)
            print("Executed Command = {}".format(immediate_command_str))
            wait_for_command = True

        else:
            print("Waiting for commands")
            time.sleep(0.5)

    except Exception as e:
        print(f"Error: {e}")
        
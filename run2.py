from drone import Drone
from drone import *

local_host = '192.168.12.123'
cmd_port = 12345
ctrl_port = 54321
status_port = 60003, 60004

threading.Thread(target=server_receive_and_execute_immediate_command,args=(local_host,)).start()

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



while True:
    try:
        if "CD2" in drone_list and d1 is None:
            CD2 = Drone(status_port[0],'/dev/serial0',115200)
            d1 = CD2
            d1_str = 'CD2'
        elif "CD2" not in drone_list & d1 is not None:
            d1.exit()
            d1 = None
            time.sleep(2)

        if "CD3" in drone_list and d2 is None:
            CD3 = Drone(status_port[1], '0.0.0.0:14553')
            d2 = CD3
            d2_str = 'CD3'
        elif "CD3" not in drone_list & d2 is not None:
            d2.exit()
            d2 = None
            time.sleep(2)
        
            

    except Exception as e:
        print(f"Error in executing immediate command: {e}")

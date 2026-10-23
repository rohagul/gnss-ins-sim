
from pymavlink import mavutil
import math
import threading
import time


CONNECTION_STRING = "udp:172.23.0.1:14551"
GIMBAL_RATE_HZ = 15.0

master = None

def setup_mavlink(connection_string, source_system=0,source_component=0):
    global master
    try:
    
        master = mavutil.mavlink_connection(connection_string, source_system=0, source_component=0)
        print("Waiting for heartbeat from system...")
        master.wait_heartbeat()
        print("Mavlink Connection established.")
    except Exception as e:
        print(f"Failed to establish mavlink connection: {e}")
        master = None
   

def telemetry_receive(master):
   
    tags = {"ATTITUDE", "GLOBAL_POSITION_INT", "HEARTBEAT"}
    while True:
        msg = master.recv_match(type=list(tags), blocking=True, timeout=5)
        if msg is None:
            continue
        mtype = msg.get_type()

        if mtype == "ATTITUDE":
            print(f"[ATTITUDE] roll={math.degrees(msg.roll):6.1f}  "
                  f"pitch={math.degrees(msg.pitch):6.1f}  yaw={math.degrees(msg.yaw):6.1f}")
        elif mtype == "GLOBAL_POSITION_INT":
            print(f"[POS] lat={msg.lat/1e7:.6f}  lon={msg.lon/1e7:.6f}  "
                  f"alt={msg.relative_alt/1000.0:.1f} m") #alt above home location
        elif mtype == "HEARTBEAT":
            mode = mavutil.mode_string_v10(msg)
            print(f"[HEARTBEAT] mode={mode}  armed={bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)}")



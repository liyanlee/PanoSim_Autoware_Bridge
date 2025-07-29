import socket
from DataInterfacePython import *
import struct
import keyboard



data_fmt = "iidddd"

data_size = 40

def ModelStart(userData):
    userData["xDriver_speed_input"] = BusAccessor(userData["busId"], "xDriver_speed_input", "time@i,valid@b,speed@d,accel@d")

    userData["ego_control_steer"] = BusAccessor(userData["busId"], "ego_control.steer", "time@i,valid@b,steer@d")
    userData["ego_ctrl_mode"] = BusAccessor(userData["busId"], "ego_control.mode", "time@i,valid@b,mode@i")
    userData["sock"] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    userData["sock"].setblocking(False)
    userData["sock"].bind(("0.0.0.0", int(userData["parameters"]["Port"])))
    userData["last_data"] = None
    userData["last_enganed"] = -1



def ModelOutput(userData):
    data = None
    while True:
        try:
            # struct.pack('<ibdddii', 0, 1, 1, 0, 0, 1, 0)
            data, _ = userData["sock"].recvfrom(data_size)
            userData["last_data"] = data
            #print("successfull get data")
        except:
            #print("get data exception")
            break
    if userData["last_data"]:
        enganed,data_time, target_steering_angle,target_speed,target_accel,target_jerk = struct.unpack(data_fmt, userData["last_data"])
        print("get data enganed {} target_steering_angle {} target_speed {} target_accel {} target_jerk {}".format(enganed,target_steering_angle,target_speed,target_accel,target_jerk))
        if enganed > 0 :
            if userData["last_enganed"] != enganed:
                userData["ego_ctrl_mode"].writeHeader(*(userData["time"], 1, 6))
            userData["xDriver_speed_input"].writeHeader(*(userData["time"], 1,target_speed,target_accel))
            ctrl_steer = target_steering_angle*57.3*16.33563215
            print("save steer {}".format(ctrl_steer))
            userData["ego_control_steer"].writeHeader(*(userData["time"], 1,target_steering_angle*57.3*16.33563215 ))
        else:
            if userData["last_enganed"] != enganed:
                userData["xDriver_speed_input"].writeHeader(*(userData["time"], 0,0,0))
                #userData["ego_control_steer"].writeHeader(*(userData["time"], 0,0 ))
                userData["ego_ctrl_mode"].writeHeader(*(userData["time"], 0, 0))  
        userData["last_enganed"]  = enganed


def ModelTerminate(userData):
    userData["sock"].close()
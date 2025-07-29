import socket
from DataInterfacePython import *
import struct


ego_format = "time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d"
ego_key = "ego"
ego_extra_format = (
    "time@i,VX@d,VY@d,VZ@d,AVx@d,AVy@d,AVz@d,Ax@d,Ay@d,Az@d,AAx@d,AAy@d,AAz@d"
)
ego_extra_key = "ego_extra"

imu_format = "Timestamp@i,ACC_X@d,ACC_Y@d,ACC_Z@d,Gyro_X@d,Gyro_Y@d,Gyro_Z@d,Yaw@d,Pitch@d,Roll@d" 
imu_key = "IMU.0"

ego_all_format = "time@i,295@[,data@d"
ego_all_key = "ego_all"

global9_key = "global.9"
global9_fmt = "time@i,variable@d"


engage_key = "engage.status"
engage_fmt = "time@i,engage@i"


def ModelStart(userData):
    userData["egoBus"] = BusAccessor(userData["busId"], ego_key, ego_format)
    userData["egoExtraBus"] = BusAccessor(userData["busId"], ego_extra_key, ego_extra_format)
    userData["egoAllBus"] = BusAccessor(userData["busId"], ego_all_key, ego_all_format)
    userData["global9Bus"] = BusAccessor(userData["busId"], global9_key, global9_fmt)
    userData["engageBus"] = BusAccessor(userData["busId"], engage_key, engage_fmt)


    userData["gnss_step"] = 1000.0 / int(userData["parameters"]["GnssFreq"])
    userData["gnss_sock"] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    userData["last_gnss"] = 0
    userData["gnss_peer"] = (userData["parameters"]["RemoteIP"], int(userData["parameters"]["GnssPort"]))

    userData["IMUBus"] = BusAccessor(userData["busId"], imu_key, imu_format)
    userData["imu_step"] = 1000.0 / int(userData["parameters"]["ImuFreq"])
    userData["imu_sock"] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    userData["last_imu"] = 0
    userData["imu_peer"] = (userData["parameters"]["RemoteIP"], int(userData["parameters"]["ImuPort"]))

    userData["egostatus_step"] = 1000.0 / int(userData["parameters"]["EgoStatusFreq"])
    userData["egostatus_sock"] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    userData["last_egostatus"] = 0
    userData["egostatus_peer"] = (userData["parameters"]["RemoteIP"], int(userData["parameters"]["EgoStatusPort"]))

    userData["OffsetX"] = float(userData["parameters"]["OffSetX"])
    userData["OffsetY"] = float(userData["parameters"]["OffSetY"])


def ModelOutput(userData):
    if userData["time"] - userData["last_gnss"] >= userData["gnss_step"]:  # send gnss related
        ts, x, y, z, yaw, pitch, roll, speed =  userData["egoBus"].readHeader()
        x += userData["OffsetX"]
        y += userData["OffsetY"]
        send_data = struct.pack("<iddddddd",*(ts,x,y,z,yaw,pitch,roll,speed))
        userData["gnss_sock"].sendto(send_data,userData["gnss_peer"])
        #print("finish send gnss to peer time {}".format(userData["time"]))
        userData["last_gnss"] = userData["time"]
    
    if userData["time"] - userData["last_imu"] >= userData["imu_step"]:  # send IMU related
        realSize = userData["IMUBus"].getHeaderSize()
        userData["imu_sock"].sendto(userData["IMUBus"].getBus()[0:realSize], userData["imu_peer"])
        userData["last_imu"] = userData["time"]
        #print("finish send imu to peer time {}".format(userData["time"]))

    if userData["time"] - userData["last_egostatus"] >= userData["egostatus_step"]:   # send ego status related
        ts,VX,VY,VZ,AVx,AVy,AVz,Ax,Ay,Az,AAx,AAy,AAz = userData["egoExtraBus"].readHeader()
        _t,engage_status = userData["engageBus"].readHeader()
        _,g_9 = userData["global9Bus"].readHeader()
        g_9 = int(g_9)
        GearState = int(userData["egoAllBus"].readBody(93)[0])
        throttle = userData["egoAllBus"].readBody(102)[0]
        brake = userData["egoAllBus"].readBody(19)[0] 
        steer = userData["egoAllBus"].readBody(121)[0] 

        #print("brake {}".format(brake))
        #print("steer {}".format(steer))
        #print("throttle {}".format(throttle))

        send_data = struct.pack("<iiiidddddd", *(userData["time"],g_9,GearState,engage_status,throttle,brake,steer,VX,VY,AVz,))
        userData["egostatus_sock"].sendto(send_data,userData["egostatus_peer"])
        userData["last_egostatus"] = userData["time"]
        #print("finish send ego status to peer {}".format(userData["time"] ))


def ModelTerminate(userData):
    userData["gnss_sock"].close()
    userData["imu_sock"].close()
    userData["egostatus_sock"].close()
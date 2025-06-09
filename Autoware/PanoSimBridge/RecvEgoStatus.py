import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
from rclpy.qos import *
from builtin_interfaces.msg import Time
from autoware_auto_vehicle_msgs.msg import VelocityReport,SteeringReport,ControlModeReport,GearReport,TurnIndicatorsReport
from tier4_vehicle_msgs.msg import ActuationStatusStamped,ActuationStatus
from autoware_auto_vehicle_msgs.msg import Engage
import time
import struct
import traceback
import socket
import math


def convert_angle(angle):
    """
    convert angle into (-pi pi]
    """
    while True:
        if angle >= math.pi:
            angle -= math.pi
        elif angle < -math.pi:
                angle += math.pi
        else:
            break
    return angle

def to_ms(sec,nanosec):
    return sec*1000.0 + nanosec / 1000000


class EgoStatusNode(Node):
    def __init__(self, recvPort,freq):
        super().__init__('RecvEgoStatus')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
        )
        
        self.publisher_velocity_status = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status',qos)
        self.publisher_steering_status = self.create_publisher(SteeringReport, '/vehicle/status/steering_status',qos)
        self.publisher_control_mode_status = self.create_publisher(ControlModeReport, '/vehicle/status/control_mode',qos)
        self.publisher_gear_status = self.create_publisher(GearReport, '/vehicle/status/gear_status',qos)
        self.publisher_turn_indicators_status = self.create_publisher(TurnIndicatorsReport, '/vehicle/status/turn_indicators_status',qos)
        self.publisher_actuation_status = self.create_publisher(ActuationStatusStamped, '/vehicle/status/actuation_status',qos)

        self.engage_pub = self.create_publisher(Engage, '/autoware/engage', 10)

        #get data from panosim
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind(("0.0.0.0",recvPort))
    
        self.frequency = freq
        self.timer_period = (1000.0/ self.frequency)
        self.lastData = None
        self.get_data = False
        self.last_enage = -1
    
    def release(self):
        self.sock.close()
        self.destroy_node()
        rclpy.shutdown()

    def do(self):
        try:
            data,address= self.sock.recvfrom(4096)
            self.get_data = True
            self.lastData = data
        except Exception as e:
                if self.get_data is False:
                    time.sleep(0.001*self.timer_period)
                else:
                    self.get_data = False
                    _t,g_9,GearState,engageFlag,throttle,brake,steer,VX,VY,AVz = struct.unpack("<iiiidddddd",self.lastData)
                    steer = steer/17.33563215
                    if VX < 0.01:
                        VX = 0.0
                    if VY < 0.01:
                        VY = 0.0
                    if AVz < 0.01:
                        AVz = 0.0
                    current_time = self.get_clock().now()
                    CurSec = current_time.seconds_nanoseconds()[0]
                    CurNanosec = current_time.seconds_nanoseconds()[1]
            
                    # velocity
                    VelocityReport_= VelocityReport()
                    VelocityReport_.header.frame_id = 'base_link'
                    VelocityReport_.header.stamp = Time(sec=CurSec,nanosec=CurNanosec)
                    VelocityReport_.longitudinal_velocity = VX
                    VelocityReport_.lateral_velocity = VY
                    VelocityReport_.heading_rate = AVz
                    self.publisher_velocity_status.publish(VelocityReport_)

                    # steering status
                    
                    SteeringReport_ =SteeringReport()
                    SteeringReport_.stamp = Time(sec=CurSec,nanosec=CurNanosec)
                    SteeringReport_.steering_tire_angle = float(steer)
                    self.publisher_steering_status.publish(SteeringReport_)


                    """
                    publish actuation status message 
                    """

                    ActuationStatusStamped_ = ActuationStatusStamped()
                    ActuationStatusStamped_.header.frame_id = "base_link"
                    ActuationStatusStamped_.header.stamp = Time(sec=CurSec,nanosec=CurNanosec)
                    ActuationStatus_ = ActuationStatus()
                    ActuationStatus_.accel_status = throttle
                    ActuationStatus_.brake_status = 0.0 #brake
                    ActuationStatus_.steer_status = steer
                    ActuationStatusStamped_.status = ActuationStatus_
                    self.publisher_actuation_status.publish(ActuationStatusStamped_)


                    """
                    publish control mode status message 
                    """
                    ControlModeReport_ = ControlModeReport()
                    ControlModeReport_.stamp = Time(sec=CurSec,nanosec=CurNanosec)
                    ControlModeReport_.mode = 1
                    self.publisher_control_mode_status.publish(ControlModeReport_)
                    

                    """
                    publish control gear status message 
                    """
                    autoware_gear = 22
                    if GearState == -1:
                        autoware_gear= 20 # R
                    elif GearState == 0:
                        autoware_gear = 22 # N
                    elif GearState == 1:
                        autoware_gear = 2   #D
                    elif GearState == 5:
                        autoware_gear = 2  #D

                    autoware_gear = 1
                    GearReport_ = GearReport()
                    GearReport_.stamp = Time(sec=CurSec,nanosec=CurNanosec)
                    GearReport_.report = autoware_gear
                    self.publisher_gear_status.publish(GearReport_)


                    """
                    publish turn indicators status message 
                    """
                    report_ = 1
                    if g_9 == 32:
                        report_ = 3
                    elif g_9 == 16:
                        report_ = 2
                    elif g_9 == 0:
                        report_ = 1
                
                    TurnIndicatorsReport_ = TurnIndicatorsReport()
                    TurnIndicatorsReport_.stamp = Time(sec=CurSec,nanosec=CurNanosec)
                    TurnIndicatorsReport_.report = report_  
                    self.publisher_turn_indicators_status.publish(TurnIndicatorsReport_)     

                    self.last_sec = CurSec
                    self.last_nanosec = CurNanosec
                    self.last_ms = to_ms(self.last_sec,self.last_nanosec)

                    if engageFlag == -1:
                        engage = Engage()
                        engage.engage = False
                        self.engage_pub.publish(engage)
                    elif engageFlag == 1:
                        engage = Engage()
                        engage.engage = True
                        self.engage_pub.publish(engage) 
                    print("publush engage {}".format(engageFlag))
                    print("finish vehicle status {}".format(time.time()))
                    print("ros time {}".format(current_time.to_msg()))


def main(args=None):
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    egoStatusNode = EgoStatusNode(10002,100)
    #rclpy.spin(egoStatusNode)


    try:
        while rclpy.ok():
            rclpy.spin_once(egoStatusNode, timeout_sec=0.01)  # 让ROS 2处理事件，包括/clock
            egoStatusNode.do()  # 你的主循环逻辑
    except Exception as e:
        traceback.print_tb(e.__traceback__)
    finally:
        egoStatusNode.release()


if __name__ == '__main__':
    main()




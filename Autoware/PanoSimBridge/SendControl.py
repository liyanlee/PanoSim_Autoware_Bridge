import rclpy
from rclpy.qos import *

from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from rclpy.node import Node
from std_msgs.msg import Bool
from autoware_auto_planning_msgs.msg import Trajectory  # 或根据你的实际话题类型调整import rclpy
from rclpy.qos import *

from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from rclpy.node import Node
from std_msgs.msg import Bool
#from autoware_auto_planning_msgs.msg import Trajectory  # 或根据你的实际话题类型调整
from autoware_auto_vehicle_msgs.msg import Engage
#from autoware_adapi_v1_msgs.msg import Goal  # 或根据你的 Autoware 版本调整消息类型
from geometry_msgs.msg import PoseStamped
import time
import socket
import struct


send_fmt = "iidddd" #




class ControlSender(Node):
    def __init__(self,send_ip,send_port):
        super().__init__('ControlSender')
        

        # target values
        self.target_steering_angle = 0.
        self.target_speed = 0.
        self.target_accel = 0.
        self.target_jerk = 0.
        self.cur_time = 0
        # Subscribe to the control topic
        self.control_subscriber = self.create_subscription(AckermannControlCommand, '/control/command/control_cmd', self.ackermann_command_updated, 10)
        self.engage_pub = self.create_publisher(Engage, '/autoware/engage', 10)
        self.engage_sub = self.create_subscription(
            Engage, '/autoware/engage', self.engage_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/planning/mission_planning/goal', self.goal_callback, 10
        )
        self.engaged = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.send_ip = send_ip
        self.send_port = send_port

    def ackermann_command_updated(self,ros_ackermann_drive):
        """
        set target values
        """
        cur_time = self.get_clock().now()
        cur_time_ms = cur_time.nanoseconds // 1_000_000
        self.target_steering_angle = ros_ackermann_drive.lateral.steering_tire_angle
        self.target_speed = ros_ackermann_drive.longitudinal.speed
        self.target_accel = ros_ackermann_drive.longitudinal.acceleration
        self.target_jerk = ros_ackermann_drive.longitudinal.jerk
        print("time {} target_steering_angle {} target_speed {} target_accel {} target_jerk {}".
              format(cur_time,self.target_steering_angle,self.target_speed,self.target_accel,self.target_jerk))
        flag = 0
        if self.engaged is True:
            flag = 1

        data = struct.pack(send_fmt, flag, cur_time_ms,
                           self.target_steering_angle, self.target_speed, self.target_accel, self.target_jerk) 
        self.sock.sendto(data, (self.send_ip, self.send_port))  


    def engage_callback(self, msg):
        print("self engaga {} recv engage {}".format(self.engaged,msg.engage))
        if self.engaged == msg.engage:
            return
        else:
            self.engaged = msg.engage
            
    def goal_callback(self, msg):
        print("User set a new mission goal, auto-engage!")
        
        #self.engaged = True
        #engage_msg = Engage()
        #engage_msg.engage = True
        #self.engage_pub.publish(engage_msg)
        

def main(send_ip, send_port):
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    ctrlSender = ControlSender(send_ip,send_port)
    rclpy.spin(ctrlSender)
    ctrlSender.destroy_node()
    rclpy.shutdown()
    
import sys

send_ip = "192.168.10.10"
send_port = 9500

if __name__ == '__main__':  
    main(send_ip,send_port)


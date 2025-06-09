import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
from rclpy.qos import *
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu

import struct
import traceback
import socket
import time
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


class IMUNode(Node):
    def __init__(self, recvPort,freq):
        super().__init__('RecvIMU')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
        )
        self.publisher = self.create_publisher(Imu, '/sensing/imu/tamagawa/imu_raw',qos)
        self.publisher_IMU = self.create_publisher(Imu, '/sensing/imu',qos)

        #get data from panosim
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind(("0.0.0.0",recvPort))
    
        self.frequency = freq
        self.timer_period = (1000.0/ self.frequency)

        self.lastData = None
        self.get_data = False
    

    def do(self):
        #print("i am imu {} {}".format(msg.clock.sec,msg.clock.nanosec))
        try:
            data,address= self.sock.recvfrom(4096)
            self.get_data = True
            self.lastData = data
        except Exception as e:
                if self.get_data is False:
                    time.sleep(0.001*self.timer_period)
                else:
                    self.get_data = False
                    current_time = self.get_clock().now()
                    CurSec = current_time.seconds_nanoseconds()[0]
                    CurNanosec = current_time.seconds_nanoseconds()[1]
                    _,ACC_X,ACC_Y,ACC_Z,Gyro_X,Gyro_Y,Gyro_Z,Yaw,Pitch,Roll = struct.unpack("<iddddddddd",self.lastData)
            
                    Yaw = convert_angle(Yaw)
                    Pitch = convert_angle(Pitch)
                    Roll = convert_angle(Roll)
                    
                    header_ = Header()
                    header_.frame_id = 'imu'
                    header_.stamp = Time(sec=CurSec,nanosec=CurNanosec)

                    Imu_ = Imu()
                    Imu_.header = header_
                    Imu_.linear_acceleration.x = -ACC_X
                    Imu_.linear_acceleration.y = ACC_Y
                    Imu_.linear_acceleration.z = -ACC_Z

                    Imu_.angular_velocity.x = -Gyro_X
                    Imu_.angular_velocity.y = Gyro_Y
                    Imu_.angular_velocity.z = -Gyro_Z
                    #print(Imu_.angular_velocity)
        
                    Imu_.orientation.x = 0.0
                    Imu_.orientation.y = 0.0
                    Imu_.orientation.z = 0.0
                    Imu_.orientation.w = 1.0

                    Imu_.header = header_
                    self.publisher.publish(Imu_)
                    self.publisher_IMU.publish(Imu_)    
                    print("finish imu status {}".format(time.time()))
                    print("ros time {}".format(current_time.to_msg()))
                    #self.get_logger().info('I heard: "%s"' % msg.data)



def main(args=None):
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    imuNode = IMUNode(10001,100)
    try:
        while rclpy.ok():
            rclpy.spin_once(imuNode, timeout_sec=0.01)  # 让ROS 2处理事件，包括/clock
            imuNode.do()  # 你的主循环逻辑
    except Exception as e:
        traceback.print_tb(e.__traceback__)
    finally:
        imuNode.release()


if __name__ == '__main__':
    main()




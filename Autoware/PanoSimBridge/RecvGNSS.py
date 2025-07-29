import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
from rclpy.qos import *
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from autoware_adapi_v1_msgs.msg import MrmState
from autoware_auto_vehicle_msgs.msg import VelocityReport
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState

import struct
import traceback
import time
import socket
from scipy.spatial.transform import Rotation
import math
import time

# this if for chuangxinyuan
# x offset 33978.3  0.0 for NongDa
# 45.54 20.07 for Town01
offset_x = 0.0
# y offset
offset_y = 0.0
# wheelbase
wheel = 2.578


def to_ms(sec,nanosec):
    return sec*1000.0 + nanosec / 1000000

class GNSSNode(Node):
    def __init__(self, recvPort, freq):
        super().__init__('RecvGNSS')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
        )

        self.publisher_LocalizationWithCovarianceStamped = self.create_publisher(
            PoseWithCovarianceStamped, "/localization/pose_estimator/pose_with_covariance", qos
        )

        self.publisher_PoseWithCovarianceStamped = self.create_publisher(
            PoseWithCovarianceStamped, "/sensing/gnss/pose_with_covariance", qos
        )
        self.publisher_PoseStamped = self.create_publisher(
            PoseStamped, "/sensing/gnss/pose", qos
        )

        self.publisher_PoseInitial = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 
            QoSProfile(reliability = QoSReliabilityPolicy.RELIABLE,
                       depth = 1, 
                       durability = QoSDurabilityPolicy.VOLATILE)
        )
        # 移除这个发布器
        self.publisher_CurrentPos = self.create_publisher(
             PoseStamped, "current_pose", qos
         )

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.initialpose_callback,
            10)   

        self.create_subscription(
            LocalizationInitializationState,
            "/localization/initialization_state",
            self.localization_state_callback,
            10)   



        #get data from panosim
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind(("0.0.0.0",recvPort))
    
        self.frequency = freq
        self.timer_period = (1000.0/ self.frequency)

        self.lastData = None
        self.get_data = False
        self.need_gnss = True
        
        self.mrm_state = 3
        self.mrm_behavior = 2 
        self.current_velocity = 10

        self.create_subscription(
            MrmState,
            '/system/fail_safe/mrm_state',
            self.mrm_callback,
            10
        )


        self.create_subscription(
            VelocityReport,
            '/vehicle/status/velocity_status',
            self.velocity_callback,
            10
        )

        self.last_gnss_time = time.time()
    

    def initialpose_callback(self, msg):
        cut_time = time.time()
        #print("cur time is {}".format(cut_time))
        
        if (cut_time - self.last_gnss_time) > 5 and self.need_gnss is False:  # this means msg comes from rviz
            self.need_gnss = True
            self.last_gnss_time = cut_time
            print("reset gnss signal")
        


    def release(self):
        self.sock.close()
        self.destroy_node()
        rclpy.shutdown()

    
    def mrm_callback(self, msg):
        print(msg)
        self.mrm_state = msg.state
        self.mrm_behavior = msg.behavior

    def velocity_callback(self, msg):
        self.current_velocity = msg.longitudinal_velocity

    def localization_state_callback(self,msg):
        print(msg)
        if msg.state in (0,1):
            self.need_gnss = True


    def do(self):
        try:
            data,address = self.sock.recvfrom(4096)
            self.get_data = True
            self.lastData = data
        except Exception as e:
            if self.get_data is False:
                time.sleep(0)
            else:
                print("mrm {}".format(self.mrm_state))
                #print("get gnss data")
                self.get_data = False
                current_time = self.get_clock().now()
                CurSec = current_time.seconds_nanoseconds()[0]
                CurNanosec = current_time.seconds_nanoseconds()[1]
                print("time is sec:{} nanosec:{}".format(CurSec,CurNanosec))
                ts, x, y, z, yaw, pitch, roll, speed = struct.unpack("<iddddddd",self.lastData)
                header_ = Header()
                header_.frame_id = "map"
                header_.stamp = Time(sec=CurSec,nanosec=CurNanosec)

                XX = x + offset_x - wheel * math.cos(yaw)
                YY = y + offset_y - wheel * math.sin(yaw)
                XXX = 42.962
                YYY = 20.07
                # gnss_cov
                GNSS_cov = PoseWithCovarianceStamped()
                GNSS_cov.pose.pose.position.x = XX
                GNSS_cov.pose.pose.position.y = YY
                GNSS_cov.pose.pose.position.z = z
                euler_angles = [roll, pitch, yaw]
                rot_mat = Rotation.from_euler("xyz", euler_angles).as_matrix()
                quaternion = Rotation.from_matrix(rot_mat).as_quat()
                GNSS_cov.pose.pose.orientation.x = quaternion[0]
                GNSS_cov.pose.pose.orientation.y = quaternion[1]
                GNSS_cov.pose.pose.orientation.z = quaternion[2]
                GNSS_cov.pose.pose.orientation.w = quaternion[3]

                # 调整协方差值以适配 Eagleye
                GNSS_cov.pose.covariance = [0.0]*36
                GNSS_cov.pose.covariance[0] = 0.25   # x-x 协方差
                GNSS_cov.pose.covariance[7] = 0.25   # y-y 协方差
                GNSS_cov.pose.covariance[14] = 0.25  # z-z 协方差
                GNSS_cov.pose.covariance[21] = 0.1   # roll-roll 协方差
                GNSS_cov.pose.covariance[28] = 0.1   # pitch-pitch 协方差
                GNSS_cov.pose.covariance[35] = 0.1   # yaw-yaw 协方差
                GNSS_cov.header = header_

                #gnss_pose
                GNSS_Pose = PoseStamped()
                GNSS_Pose.pose.position.x = XX #x + offset_x - wheel * math.cos(yaw)
                GNSS_Pose.pose.position.y = YY #y + offset_y - wheel * math.sin(yaw)
                # x0 = 88.4
                # y0 = 138.771
                # GNSS_Pose.pose.position.x = (
                #     (x - x0) * math.cos(0.326) - (y - y0) * math.sin(0.326) + offset_x
                # )
                # GNSS_Pose.pose.position.y = (
                #     (x - x0) * math.sin(0.326) + (y - y0) * math.cos(0.326) + offset_y
                # )
                # print(GNSS_Pose.pose.position.x, GNSS_Pose.pose.position.y)
                GNSS_Pose.pose.position.z = z
                euler_angles = [roll, pitch, yaw]
                rot_mat = Rotation.from_euler("xyz", euler_angles).as_matrix()
                quaternion = Rotation.from_matrix(rot_mat).as_quat()
                GNSS_Pose.pose.orientation.x = quaternion[0]
                GNSS_Pose.pose.orientation.y = quaternion[1]
                GNSS_Pose.pose.orientation.z = quaternion[2]
                GNSS_Pose.pose.orientation.w = quaternion[3]
                GNSS_Pose.header = header_
                

                # 检查速度是否为0
                print(f"Current longitudinal_velocity: {self.current_velocity}")
                if self.need_gnss is True and self.mrm_behavior == 1 and self.mrm_state == 1 and abs(self.current_velocity) < 0.01:
                    time.sleep(1)
                    print("Ready to publish /initialpose")
                    self.need_gnss = False
                    #print("Publishing /initialpose with data:", GNSS_cov)
                    self.publisher_PoseInitial.publish(GNSS_cov)
                    # 移除这行
                    # self.publisher_PosLocallization.publish(GNSS_Pose)
                    print("finish save to initial pos")

                self.publisher_PoseStamped.publish(GNSS_Pose)
                self.publisher_PoseWithCovarianceStamped.publish(GNSS_cov)
                self.publisher_LocalizationWithCovarianceStamped.publish(GNSS_cov)
                self.publisher_CurrentPos.publish(GNSS_Pose)
                # 移除这行
                # self.publisher_PosLocallization.publish(GNSS_Pose)
                print("finish gnss status {}".format(time.time()))
                #print("ros time {}".format(current_time.to_msg()))
        


def main(args=None):
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    gnssNode = GNSSNode(10000, 10)
    try:
        while rclpy.ok():
            rclpy.spin_once(gnssNode, timeout_sec=0.01)  # 让ROS 2处理事件，包括/clock
            gnssNode.do()  # 你的主循环逻辑
    except Exception as e:
        traceback.print_tb(e.__traceback__)
    finally:
        gnssNode.release()


if __name__ == '__main__':
    main()





import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
from rclpy.qos import *
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2,PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from scipy.spatial.transform import Rotation
import numpy as np
import cv2
import array
import struct
import traceback
import time
import socket
from scipy.spatial.transform import Rotation
import math





def to_ms(sec,nanosec):
    return sec*1000.0 + nanosec / 1000000


class LidarNode(Node):
    def __init__(self, recvPort):
        super().__init__('ReceLidar')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        )
    
        #self.publisher_ = self.create_publisher(PointCloud2, '/lidar_points',qos)
        self.publisher_ = self.create_publisher(PointCloud2, '/points_raw',qos)
        self.address = "0.0.0.0"
        self.port = recvPort

        self.num_beam = 16
        self.mea_per_rot = 1440
        self.max_count = self.num_beam * self.mea_per_rot
        self.lidar_height = float(1.25)



    def do(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
            server_sock.bind((self.address, self.port))
            server_sock.listen()
            while True:
                point_cloud_sequence_num = 1
                sock, addr = server_sock.accept()
                with sock:
                    print(f"PanoSim connected Lidar: {addr}")
                    while rclpy.ok():
                        header = sock.recv(8, socket.MSG_WAITALL)
                        if not header:
                            print(f"PanoSim disconnected: {addr}")
                            break
                        ts, width = struct.unpack_from('<ii', header)
                        if width <= 0:
                            break
                        rclpy.spin_once(self, timeout_sec=0.01)
                        current_time = self.get_clock().now()
                        CurSec = current_time.seconds_nanoseconds()[0]
                        CurNanosec = current_time.seconds_nanoseconds()[1]
                        
                        point_cloud_data = sock.recv(width * 16, socket.MSG_WAITALL)
                        if not point_cloud_data:
                            break
                        
                        print("width is {}".format(width))
                        x_np_array = np.ndarray((width,), '<f', point_cloud_data, 0, (16,))
                        y_np_array = np.ndarray((width,), '<f', point_cloud_data, 4, (16,))
                        z_np_array = np.ndarray((width,), '<f', point_cloud_data, 8, (16,))
                        intensity_np_array = np.ndarray((width, ), '<f', point_cloud_data, 12, (16,))


                        points_lidar = [( x_np_array[index],
                                         y_np_array[index],
                                         z_np_array[index],
                                         intensity_np_array[index]
                                         ) for index in range(width) ]
                        
                        print("point lidar width {}".format(len(points_lidar)))

                        #header_ = Header()
                        #header_.frame_id="velodyne"
                        #header_.stamp = Time(sec=CurSec,nanosec=CurNanosec)
                        #height_ = 1
                        #width_ = len(points_lidar)
                        """
                        fields_ = [
                                        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                                        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                                        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                                        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
                                    ]
                        #point_step_ = self.max_count
                        #row_step_ = self.mea_per_rot * point_step_
                        
                        bin_data = b''.join([struct.pack('ffff', *point) for point in points_lidar])
                        msg = PointCloud2(
                            header=header_,
                            height=height_, 
                            width=width_,
                            is_dense=False,
                            is_bigendian=False,
                            fields=fields_,
                            point_step=point_step_,
                            row_step=row_step_,
                            #data=new_data
                            data = bin_data
                        )
                        """
                        header = Header()   
                        header.stamp = Time(sec=CurSec,nanosec=CurNanosec)
                        header.frame_id = "velodyne"  # Set the appropriate frame ID
                        fields_ = [
                                        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                                        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                                        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                                        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
                                    ]
                        pointcloud_msg = create_cloud(header, fields_, points_lidar)
                        self.publisher_.publish(pointcloud_msg)
                        point_cloud_sequence_num += 1
                        print("finish lidar status {}".format(time.time()))
                        print("ros time {}".format(current_time.to_msg()))


def main(args=None):
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    lidarNode = LidarNode(10004)
    while True:
        try:
            lidarNode.do()
        except Exception as e:
            traceback.print_tb(e.__traceback__)
            break
    lidarNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





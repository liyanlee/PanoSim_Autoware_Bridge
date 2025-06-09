import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
from rclpy.qos import *
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo,Image
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


class SimClock(Node):
    def __init__(self,name='clock'):
        super().__init__(name)
        self.format = "sec@i,nanosec@i" 
        self.key = "simtime"
        self.subscriber = self.create_subscription(Clock, "/clock", self.subCallback, 1)
        self.sec = 0
        self.nanosec = 0
        

    def subCallback(self,msg):
        self.sec = msg.clock.sec
        self.nanosec = msg.clock.nanosec
         
    def getClock(self):
        return Time(sec=self.sec,nanosec=self.nanosec)



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


class ImageNode(Node):
    def __init__(self, recvPort):
        super().__init__('ReceImage')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        )
        self.publisher_image = self.create_publisher(Image, '/sensing/camera/traffic_light/image_raw',qos)
        self.publisher_camera = self.create_publisher(CameraInfo, '/sensing/camera/traffic_light/camera_info',qos)

        self.timer = SimClock()
        self.address = "0.0.0.0"
        self.port = recvPort
        self.Height = 400
        self.Width = 800


    def do(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
            server_sock.bind((self.address, self.port))
            server_sock.listen()
            while True:
                jpeg_sequence_num = 1
                sock, addr = server_sock.accept()
                with sock:
                    
                    while rclpy.ok():
                        header = sock.recv(8)
                        if not header:
                            break
                        

                        ts, width = struct.unpack_from('<ii', header)
                        if width <= 0:
                            break
                        rclpy.spin_once(self, timeout_sec=0.01)
                        current_time = self.get_clock().now()
                        CurSec = current_time.seconds_nanoseconds()[0]
                        CurNanosec = current_time.seconds_nanoseconds()[1]
                        jpeg_data = b""
                        recv_length = 0
                        while recv_length < width:
                            buffer = sock.recv(width - recv_length)
                            if buffer:
                                jpeg_data += buffer
                                recv_length = len(jpeg_data)
                                #print("recv_lenght {}".format(recv_length))
                        if not jpeg_data:
                            break
                        jpg_array = np.frombuffer(jpeg_data, dtype=np.uint8)
                        # Decode the JPEG data to an image
                        bgr_image = cv2.imdecode(jpg_array, cv2.IMREAD_COLOR)

                        # Convert the BGR image to RGB
                        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)

                        header_ = Header()
                        header_.stamp = Time(sec=CurSec,nanosec=CurNanosec)
                        header_.frame_id = 'camera'
                        
                        image_ = Image()
                        image_.header = header_
                        image_.height = int(self.Height)
                        image_.width = int(self.Width)
                        image_.encoding = 'rgb8'
                        image_.is_bigendian = 1
                        image_.step = image_.width * 3         
                        image_.data=array.array('B',rgb_image.tobytes())
                


                        CameraInfo_ = CameraInfo()
                        CameraInfo_.header = header_
                        CameraInfo_.height = int(self.Height)
                        CameraInfo_.width = int(self.Width)
                        CameraInfo_.distortion_model = 'plumb_bob'
                        CameraInfo_.d = [0.0,0.0,0.0,0.0,0.0]
                        cx = int(self.Height)/2
                        cy = int(self.Width)/2
                        fx = 500.0
                        fy = 500.0
                        CameraInfo_.k = [fx,0.0,cx,0.0,fy,cy,0.0,0.0,1.0]        
                        CameraInfo_.r = [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]        
                        CameraInfo_.p = [fx,0.0,cx,0.0,0.0,fy,cy,0.0,0.0,0.0,1.0,0.0]  

                        self.publisher_image.publish(image_)
                        self.publisher_camera.publish(CameraInfo_)


                        jpeg_sequence_num += 1
                        print("jpeg_sequence {}".format(jpeg_sequence_num))
                        print("finish image status {}".format(time.time()))
                        print("ros time {}".format(current_time.to_msg()))



def main(args=None):
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    image = ImageNode(10003)
    while True:
        try:
            image.do()
        except Exception as e:
            traceback.print_tb(e.__traceback__)
            break
    image.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

class PoseRelay(Node):
    def __init__(self):
        super().__init__('pose_relay')
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/pose_estimator/pose_with_covariance',
            self.cb, 10)
        self.pub = self.create_publisher(
            PoseStamped,
            '/localization/pose_estimator/pose', 10)

    def cb(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.pub.publish(pose)

def main(args=None):
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = PoseRelay()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

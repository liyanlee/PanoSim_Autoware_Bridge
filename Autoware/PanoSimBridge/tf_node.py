import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class PoseToTF(Node):
    def __init__(self):
        super().__init__('pose_to_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(PoseStamped, '/localization/pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToTF()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

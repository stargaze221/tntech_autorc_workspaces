from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

class OdomRelayNode(Node):
    def __init__(self):
        super().__init__('odom_relay_node')
        self.sub = self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.relay_callback, 10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def relay_callback(self, msg):
        # Publish Odometry as-is
        self.pub.publish(msg)

        # Broadcast TF transform with CURRENT TIME
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()  # <-- Critical fix
        t.header.frame_id = msg.header.frame_id           # e.g. "odom"
        t.child_frame_id = msg.child_frame_id             # e.g. "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomRelayNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

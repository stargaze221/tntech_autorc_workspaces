import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool


class ManualControlNode(Node):
    def __init__(self):
        super().__init__('manual_control_node')

        # RC input subscriptions
        self.create_subscription(Int32, '/rc/ch1', self.ch1_callback, 10)  # Steering + E-Stop
        self.create_subscription(Int32, '/rc/ch2', self.ch2_callback, 10)  # Throttle

        # Publishers
        self.servo_pub = self.create_publisher(Int32, '/servo_cmd_man', 10)
        self.rpm_pub = self.create_publisher(Int32, '/vesc_rpm_cmd_man', 10)
        self.stop_pub = self.create_publisher(Bool, '/stop_flag', 10)

        # State
        self.emergency_stop_active = False

        self.get_logger().info("🕹️ Manual Control Node using CH1 for Emergency Stop")

    def publish_stop_flag(self, active: bool):
        if active != self.emergency_stop_active:
            self.emergency_stop_active = active
            self.stop_pub.publish(Bool(data=active))
            if active:
                self.get_logger().warn("🛑 Emergency Stop ACTIVATED (CH1 < 1200)")
            else:
                self.get_logger().info("▶️ Emergency Stop RELEASED (CH1 > 1800)")

    def ch1_callback(self, msg: Int32):
        pwm = msg.data
      
        # Steering angle logic (only if not in stop mode)
        angle = int((pwm - 1000) * 180.0 / 1000.0)
        angle = max(0, min(180, angle))
        self.servo_pub.publish(Int32(data=angle))
        self.get_logger().debug(f"CH1 → Steering Angle: {angle}")

    def ch2_callback(self, msg: Int32):
        pwm = msg.data
        # Emergency stop logic
        if pwm < 1200:
            self.publish_stop_flag(True)
        elif pwm > 1800:
            self.publish_stop_flag(False)
            
        pwm = msg.data
        rpm = int((pwm - 1500) * 6)
        rpm = max(-3000, min(3000, rpm))
        self.rpm_pub.publish(Int32(data=rpm))
        self.get_logger().debug(f"CH2 → Throttle RPM: {rpm}")


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

# simple_nav_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SimpleNavNode(Node):
    def __init__(self):
        super().__init__('simple_nav_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # send commands at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.start_time = self.get_clock().now()
        self.get_logger().info("SimpleNavNode started: driving forward for 5 seconds")

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        msg = Twist()

        if elapsed < 5.0:
            # drive forward 0.2 m/s
            msg.linear.x = 0.2
            msg.angular.z = 0.0
        else:
            # stop
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

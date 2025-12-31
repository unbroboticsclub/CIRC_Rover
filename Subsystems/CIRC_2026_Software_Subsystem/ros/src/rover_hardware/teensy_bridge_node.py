# teensy_bridge_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

import serial
import threading

PORT = "/dev/ttyACM0"  # change to your Teensy port
BAUD = 115200

class TeensyBridgeNode(Node):
    def __init__(self):
        super().__init__('teensy_bridge_node')

        # ROS 2
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.accel_pub = self.create_publisher(
            Float32MultiArray,
            '/accel/raw',
            10
        )

        # Serial
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.1)
            self.get_logger().info(f"Opened serial port {PORT} at {BAUD} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

        # Thread to read from Teensy
        self.running = True
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

    # ROS callback: incoming cmd_vel → text command to Teensy
    def cmd_vel_callback(self, msg: Twist):
        if not self.ser:
            return

        v = msg.linear.x      # m/s
        w = msg.angular.z     # rad/s

        cmd_str = f"V {v:.3f} W {w:.3f}\n"
        try:
            self.ser.write(cmd_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f"Failed to write to serial: {e}")

    # Thread: read serial lines from Teensy → publish accel data
    def serial_read_loop(self):
        if not self.ser:
            return

        while self.running:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                # Expect lines like: ACC,ax,ay,az
                # for example: "ACC,0.12,-0.03,9.81"
                parts = line.split(',')
                if parts[0] == "ACC" and len(parts) == 4:
                    try:
                        ax = float(parts[1])
                        ay = float(parts[2])
                        az = float(parts[3])

                        msg = Float32MultiArray()
                        msg.data = [ax, ay, az]
                        self.accel_pub.publish(msg)
                    except ValueError:
                        self.get_logger().warn(f"Bad ACC line: {line}")
                else:
                    # You can print debug or handle other message types
                    # self.get_logger().info(f"Teensy: {line}")
                    pass

            except Exception as e:
                self.get_logger().warn(f"Error in serial_read_loop: {e}")

    def destroy_node(self):
        self.running = False
        super().destroy_node()
        if self.ser:
            self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    node = TeensyBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ImuForwarder(Node):
    def __init__(self):
        super().__init__('imu_forwarder')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200
        )
        self.sub = self.create_subscription(
            Imu, '/utlidar/imu', self.callback, qos)
        self.pub = self.create_publisher(
            Imu, '/utlidar/imu_corrected', qos)
        self.get_logger().info('IMU forwarder started.')

    def callback(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

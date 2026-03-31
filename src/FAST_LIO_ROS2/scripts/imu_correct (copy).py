#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np


def stamp_to_sec(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def sec_to_stamp(sec: float) -> Time:
    msg = Time()
    msg.sec = int(sec)
    msg.nanosec = int((sec - int(sec)) * 1e9)
    return msg


class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200
        )

        self.subscription = self.create_subscription(
            Imu,
            '/utlidar/imu',
            self.listener_callback,
            qos
        )
        self.publisher_ = self.create_publisher(
            Imu,
            '/utlidar/imu_corrected',
            qos
        )

        self.time_offset_initialized = False
        self.time_offset_sec = 0.0
        self.last_corrected_stamp = 0.0

        self.init_count = 0
        self.init_max = 300
        self.gyr_bias = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.bias_ready = False

        self.get_logger().info('IMU time align node started.')

    def listener_callback(self, msg: Imu):
        imu_time = stamp_to_sec(msg.header.stamp)
        ros_now = self.get_clock().now().nanoseconds * 1e-9

        if not self.time_offset_initialized:
            self.time_offset_sec = ros_now - imu_time
            self.time_offset_initialized = True
            self.get_logger().info(
                f'IMU time offset initialized: {self.time_offset_sec:.6f} sec'
            )

        corrected_time = imu_time + self.time_offset_sec

        if corrected_time <= self.last_corrected_stamp:
            corrected_time = self.last_corrected_stamp + 1e-6

        self.last_corrected_stamp = corrected_time

        out = Imu()
        out.header = msg.header
        out.header.stamp = sec_to_stamp(corrected_time)
        out.header.frame_id = msg.header.frame_id if msg.header.frame_id else "utlidar_imu"

        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance

        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        curr_gyr = np.array([
            out.angular_velocity.x,
            out.angular_velocity.y,
            out.angular_velocity.z
        ], dtype=np.float64)

        if not self.bias_ready:
            self.gyr_bias += curr_gyr
            self.init_count += 1
            if self.init_count >= self.init_max:
                self.gyr_bias /= float(self.init_max)
                self.bias_ready = True
                self.get_logger().info(
                    f'Gyro bias ready: {self.gyr_bias.tolist()}'
                )
            self.publisher_.publish(out)
            return

        out.angular_velocity.x -= float(self.gyr_bias[0])
        out.angular_velocity.y -= float(self.gyr_bias[1])
        out.angular_velocity.z -= float(self.gyr_bias[2])

        self.publisher_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

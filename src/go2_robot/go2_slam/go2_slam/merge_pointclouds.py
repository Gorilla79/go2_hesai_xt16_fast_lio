#!/usr/bin/env python3
"""Simple pointcloud merger for two PointCloud2 topics.

This node subscribes to two PointCloud2 topics (default: /pointcloud and /lidar_points)
and republishes a merged PointCloud2 on /merged_pointcloud. It only uses the x,y,z,intensity
fields (if present) and drops other custom fields. This is a simple, best-effort merger
intended for testing/initial integration; for production, a C++/pcl-based merger is recommended.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
try:
    from sensor_msgs_py import point_cloud2 as pc2
except Exception:
    # fallback import path
    import sensor_msgs.point_cloud2 as pc2


class MergePointclouds(Node):
    def __init__(self):
        super().__init__('merge_pointclouds')

        self.declare_parameter('topic_a', '/pointcloud')
        self.declare_parameter('topic_b', '/lidar_points')
        self.declare_parameter('output_topic', '/merged_pointcloud')

        topic_a = self.get_parameter('topic_a').get_parameter_value().string_value
        topic_b = self.get_parameter('topic_b').get_parameter_value().string_value
        out = self.get_parameter('output_topic').get_parameter_value().string_value

        self.sub_a = self.create_subscription(PointCloud2, topic_a, self.cb_a, 10)
        self.sub_b = self.create_subscription(PointCloud2, topic_b, self.cb_b, 10)
        self.pub = self.create_publisher(PointCloud2, out, 10)

        self.last_a = None
        self.last_b = None
        self.timer = self.create_timer(0.1, self.publish_merged)

        self.get_logger().info(f'Merge node listening: {topic_a} + {topic_b} -> {out}')

    def cb_a(self, msg: PointCloud2):
        self.last_a = msg

    def cb_b(self, msg: PointCloud2):
        self.last_b = msg

    def publish_merged(self):
        # prefer to merge when we have both; if only one exists, publish it
        src_msgs = []
        if self.last_a is not None:
            src_msgs.append(self.last_a)
        if self.last_b is not None:
            src_msgs.append(self.last_b)

        if not src_msgs:
            return

        header = Header()
        header.stamp = src_msgs[-1].header.stamp
        header.frame_id = src_msgs[-1].header.frame_id

        points = []
        for pc in src_msgs:
            for p in pc2.read_points(pc, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                # ensure tuple of floats
                points.append((float(p[0]), float(p[1]), float(p[2]), float(p[3]) if len(p) > 3 else 0.0))

        if not points:
            return

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
        ]

        merged = pc2.create_cloud(header, fields, points)
        self.pub.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = MergePointclouds()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import math
import numpy as np

# Compatibility patch for old scikit-learn on Ubuntu/ROS2 Foxy
if not hasattr(np, 'float'):
    np.float = float
if not hasattr(np, 'int'):
    np.int = int
if not hasattr(np, 'bool'):
    np.bool = bool

from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class Lidar3DHumanBBoxNode(Node):
    def __init__(self):
        super().__init__('lidar_3d_human_bbox_node')

        # Topics
        self.declare_parameter('input_topic', '/lidar_points')
        self.declare_parameter('marker_topic', '/lidar_3d_bboxes')

        # ROI
        self.declare_parameter('min_x', -6.0)
        self.declare_parameter('max_x', 12.0)
        self.declare_parameter('min_y', -6.0)
        self.declare_parameter('max_y', 6.0)
        self.declare_parameter('min_z', -2.0)
        self.declare_parameter('max_z', 2.5)

        # Speed / sampling
        self.declare_parameter('process_every_n', 2)      # 2면 절반 프레임만 처리
        self.declare_parameter('voxel_leaf', 0.10)        # 실시간성 향상용 다운샘플
        self.declare_parameter('max_points_after_voxel', 3500)

        # Ground removal (fast)
        self.declare_parameter('ground_percentile', 8.0)
        self.declare_parameter('ground_margin', 0.18)

        # Clustering (XY only for speed)
        self.declare_parameter('dbscan_eps', 0.42)
        self.declare_parameter('dbscan_min_samples', 6)
        self.declare_parameter('cluster_min_points', 8)
        self.declare_parameter('cluster_max_points', 220)

        # Human candidate filter (relaxed a bit so boxes appear)
        self.declare_parameter('human_min_height', 0.45)
        self.declare_parameter('human_max_height', 2.2)
        self.declare_parameter('human_min_width', 0.12)
        self.declare_parameter('human_max_width', 1.20)
        self.declare_parameter('human_min_length', 0.12)
        self.declare_parameter('human_max_length', 1.20)
        self.declare_parameter('human_min_points', 8)
        self.declare_parameter('human_max_points', 220)
        self.declare_parameter('human_max_distance', 10.0)

        # Marker
        self.declare_parameter('marker_lifetime', 0.30)
        self.declare_parameter('line_width', 0.05)
        self.declare_parameter('text_height', 0.28)

        input_topic = self.get_parameter('input_topic').value
        marker_topic = self.get_parameter('marker_topic').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )

        self.sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.cloud_callback,
            qos
        )
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)

        self.frame_count = 0

        self.get_logger().info(f'Subscribed to: {input_topic}')
        self.get_logger().info(f'Publishing human markers to: {marker_topic}')

    def cloud_callback(self, msg: PointCloud2):
        self.frame_count += 1
        process_every_n = int(self.get_parameter('process_every_n').value)
        if process_every_n > 1 and (self.frame_count % process_every_n) != 0:
            return

        pts = self.pointcloud2_to_xyz(msg)
        if pts.shape[0] == 0:
            self.publish_delete_all(msg.header.frame_id)
            return

        pts = self.roi_filter(pts)
        if pts.shape[0] == 0:
            self.publish_delete_all(msg.header.frame_id)
            return

        pts = self.voxel_downsample(pts)
        if pts.shape[0] == 0:
            self.publish_delete_all(msg.header.frame_id)
            return

        pts = self.remove_ground_fast(pts)
        if pts.shape[0] == 0:
            self.publish_delete_all(msg.header.frame_id)
            return

        humans = self.detect_humans_fast(pts)
        markers = self.build_markers(humans, msg.header.frame_id)
        self.marker_pub.publish(markers)

    def pointcloud2_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        pts = []
        for p in point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        ):
            x, y, z = p[0], p[1], p[2]
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                pts.append([x, y, z])

        if not pts:
            return np.empty((0, 3), dtype=np.float32)

        return np.asarray(pts, dtype=np.float32)

    def roi_filter(self, pts: np.ndarray) -> np.ndarray:
        min_x = float(self.get_parameter('min_x').value)
        max_x = float(self.get_parameter('max_x').value)
        min_y = float(self.get_parameter('min_y').value)
        max_y = float(self.get_parameter('max_y').value)
        min_z = float(self.get_parameter('min_z').value)
        max_z = float(self.get_parameter('max_z').value)

        mask = (
            (pts[:, 0] >= min_x) & (pts[:, 0] <= max_x) &
            (pts[:, 1] >= min_y) & (pts[:, 1] <= max_y) &
            (pts[:, 2] >= min_z) & (pts[:, 2] <= max_z)
        )
        return pts[mask]

    def voxel_downsample(self, pts: np.ndarray) -> np.ndarray:
        leaf = float(self.get_parameter('voxel_leaf').value)
        if leaf <= 0.0 or pts.shape[0] == 0:
            return pts

        grid = np.floor(pts / leaf).astype(np.int32)
        _, unique_idx = np.unique(grid, axis=0, return_index=True)
        down = pts[unique_idx]

        max_points = int(self.get_parameter('max_points_after_voxel').value)
        if down.shape[0] > max_points:
            step = max(1, down.shape[0] // max_points)
            down = down[::step]

        return down

    def remove_ground_fast(self, pts: np.ndarray) -> np.ndarray:
        if pts.shape[0] < 20:
            return pts

        ground_percentile = float(self.get_parameter('ground_percentile').value)
        ground_margin = float(self.get_parameter('ground_margin').value)

        z_ref = np.percentile(pts[:, 2], ground_percentile)
        mask = pts[:, 2] > (z_ref + ground_margin)
        return pts[mask]

    def detect_humans_fast(self, pts: np.ndarray):
        if pts.shape[0] == 0:
            return []

        # XY-only clustering for speed
        xy = pts[:, :2]

        eps = float(self.get_parameter('dbscan_eps').value)
        min_samples = int(self.get_parameter('dbscan_min_samples').value)

        labels = DBSCAN(
            eps=eps,
            min_samples=min_samples,
            algorithm='ball_tree'
        ).fit_predict(xy)

        cluster_min_points = int(self.get_parameter('cluster_min_points').value)
        cluster_max_points = int(self.get_parameter('cluster_max_points').value)

        results = []

        unique_labels = set(labels.tolist())
        if -1 in unique_labels:
            unique_labels.remove(-1)

        for label in unique_labels:
            cluster = pts[labels == label]

            npts = cluster.shape[0]
            if npts < cluster_min_points or npts > cluster_max_points:
                continue

            bbox = self.compute_axis_aligned_bbox(cluster)
            if bbox is None:
                continue

            if self.is_human_candidate(bbox):
                results.append(bbox)

        return results

    def compute_axis_aligned_bbox(self, cluster: np.ndarray):
        xyz_min = np.min(cluster, axis=0)
        xyz_max = np.max(cluster, axis=0)

        size = xyz_max - xyz_min
        center = (xyz_min + xyz_max) / 2.0

        length = float(size[0])
        width = float(size[1])
        height = float(size[2])

        if length <= 0.01 or width <= 0.01 or height <= 0.01:
            return None

        x0, y0, z0 = xyz_min
        x1, y1, z1 = xyz_max

        corners_bottom = np.array([
            [x0, y0, z0],
            [x1, y0, z0],
            [x1, y1, z0],
            [x0, y1, z0],
        ], dtype=np.float32)

        corners_top = np.array([
            [x0, y0, z1],
            [x1, y0, z1],
            [x1, y1, z1],
            [x0, y1, z1],
        ], dtype=np.float32)

        return {
            'center': center.astype(np.float32),
            'size': np.array([length, width, height], dtype=np.float32),
            'corners_bottom': corners_bottom,
            'corners_top': corners_top,
            'num_points': int(cluster.shape[0]),
            'label': 'person'
        }

    def is_human_candidate(self, bbox) -> bool:
        length = float(bbox['size'][0])
        width = float(bbox['size'][1])
        height = float(bbox['size'][2])
        num_points = int(bbox['num_points'])
        center = bbox['center']
        dist_xy = math.sqrt(center[0] ** 2 + center[1] ** 2)

        min_h = float(self.get_parameter('human_min_height').value)
        max_h = float(self.get_parameter('human_max_height').value)
        min_w = float(self.get_parameter('human_min_width').value)
        max_w = float(self.get_parameter('human_max_width').value)
        min_l = float(self.get_parameter('human_min_length').value)
        max_l = float(self.get_parameter('human_max_length').value)
        min_pts = int(self.get_parameter('human_min_points').value)
        max_pts = int(self.get_parameter('human_max_points').value)
        max_dist = float(self.get_parameter('human_max_distance').value)

        cond_height = min_h <= height <= max_h
        cond_width = min_w <= width <= max_w
        cond_length = min_l <= length <= max_l
        cond_points = min_pts <= num_points <= max_pts
        cond_dist = dist_xy <= max_dist

        area_xy = max(length * width, 1e-6)
        density = num_points / area_xy

        cond_density = density >= 12.0

        return (
            cond_height and
            cond_width and
            cond_length and
            cond_points and
            cond_dist and
            cond_density
        )

    def build_markers(self, humans, frame_id):
        marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        lifetime_sec = float(self.get_parameter('marker_lifetime').value)
        line_width = float(self.get_parameter('line_width').value)
        text_height = float(self.get_parameter('text_height').value)

        for i, bbox in enumerate(humans):
            line_marker = Marker()
            line_marker.header.frame_id = frame_id
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = 'human_bbox_lines'
            line_marker.id = i
            line_marker.type = Marker.LINE_LIST
            line_marker.action = Marker.ADD
            line_marker.scale.x = line_width
            line_marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
            line_marker.lifetime.sec = int(lifetime_sec)
            line_marker.lifetime.nanosec = int((lifetime_sec % 1.0) * 1e9)

            corners = self.make_8_corners(bbox['corners_bottom'], bbox['corners_top'])
            line_marker.points = self.make_box_lines(corners)
            marker_array.markers.append(line_marker)

            text_marker = Marker()
            text_marker.header.frame_id = frame_id
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'human_bbox_text'
            text_marker.id = 1000 + i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.scale.z = text_height
            text_marker.color = ColorRGBA(r=1.0, g=0.4, b=0.0, a=1.0)
            text_marker.lifetime.sec = int(lifetime_sec)
            text_marker.lifetime.nanosec = int((lifetime_sec % 1.0) * 1e9)
            text_marker.pose.position.x = float(bbox['center'][0])
            text_marker.pose.position.y = float(bbox['center'][1])
            text_marker.pose.position.z = float(bbox['center'][2] + bbox['size'][2] * 0.6)
            text_marker.text = f"person | pts:{bbox['num_points']}"
            marker_array.markers.append(text_marker)

        return marker_array

    def make_8_corners(self, bottom, top):
        return np.vstack([bottom, top])

    def make_box_lines(self, corners):
        idx_pairs = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]

        points = []
        for a, b in idx_pairs:
            pa = Point(x=float(corners[a][0]), y=float(corners[a][1]), z=float(corners[a][2]))
            pb = Point(x=float(corners[b][0]), y=float(corners[b][1]), z=float(corners[b][2]))
            points.append(pa)
            points.append(pb)
        return points

    def publish_delete_all(self, frame_id):
        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.header.frame_id = frame_id
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = Lidar3DHumanBBoxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

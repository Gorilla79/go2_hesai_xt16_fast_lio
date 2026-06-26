#!/usr/bin/env python3

import math
import signal
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class CloudFrame:
    xyz_base: np.ndarray
    stamp: object


@dataclass
class MotionCandidate:
    candidate_id: int
    center: np.ndarray
    box_min: np.ndarray
    box_max: np.ndarray
    points: np.ndarray
    sector: str


@dataclass
class TrackState:
    track_id: int
    center: np.ndarray
    velocity: np.ndarray
    box_min: np.ndarray
    box_max: np.ndarray
    points: np.ndarray
    sector: str
    last_stamp_sec: float
    age: int = 1
    missed: int = 0
    semantic_label: str = "unknown"
    semantic_confidence: float = 0.0


class LidarVisionPerson3DV50(Node):
    """
    V5.0 - 360-degree motion detection for fast reactive avoidance.

    Main path:
      Hesai XT16 -> fast BEV occupancy -> temporal difference
      -> connected components -> motion candidates -> lightweight tracking

    Front camera:
      Used only when a front motion candidate exists.
      YOLO assigns semantic meaning to front candidates.

    This version intentionally does not cluster every static object.
    """

    def __init__(self) -> None:
        super().__init__("lidar_vision_person_3d_v5_0")

        self._declare_parameters()
        self._load_parameters()
        self._validate_parameters()

        self.running = True
        self.cloud_lock = threading.Lock()
        self.latest_cloud: Optional[CloudFrame] = None

        self.R_base_lidar = self._rotation_matrix(
            self.lidar_roll_deg,
            self.lidar_pitch_deg,
            self.lidar_yaw_deg,
        )
        self.t_base_lidar = np.array(
            [self.lidar_base_x, self.lidar_base_y, self.lidar_base_z],
            dtype=np.float32,
        )

        self.R_base_camera = self._rotation_matrix(
            self.camera_roll_deg,
            self.camera_pitch_deg,
            self.camera_yaw_deg,
        )
        self.t_base_camera = np.array(
            [self.camera_base_x, self.camera_base_y, self.camera_base_z],
            dtype=np.float32,
        )

        self.grid_size = int(
            math.ceil(
                (2.0 * self.bev_range_m)
                / self.bev_resolution_m
            )
        )

        self.previous_occupancy = np.zeros(
            (self.grid_size, self.grid_size),
            dtype=np.uint8,
        )
        self.background_probability = np.zeros(
            (self.grid_size, self.grid_size),
            dtype=np.float32,
        )
        self.frame_count = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self._cloud_callback,
            qos,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.obstacle_marker_topic,
            10,
        )
        self.dynamic_cloud_pub = self.create_publisher(
            PointCloud2,
            self.dynamic_cloud_topic,
            qos,
        )

        self.pipeline = rs.pipeline()
        rs_config = rs.config()
        rs_config.enable_stream(
            rs.stream.color,
            self.camera_width,
            self.camera_height,
            rs.format.bgr8,
            self.camera_fps,
        )
        profile = self.pipeline.start(rs_config)
        color_profile = profile.get_stream(
            rs.stream.color
        ).as_video_stream_profile()
        intrinsics = color_profile.get_intrinsics()

        self.fx = float(intrinsics.fx)
        self.fy = float(intrinsics.fy)
        self.cx = float(intrinsics.ppx)
        self.cy = float(intrinsics.ppy)
        self.raw_camera_width = int(intrinsics.width)
        self.raw_camera_height = int(intrinsics.height)

        self.get_logger().info(
            "RealSense intrinsic: "
            f"fx={self.fx:.2f}, fy={self.fy:.2f}, "
            f"cx={self.cx:.2f}, cy={self.cy:.2f}"
        )

        self.model = None
        if self.enable_front_semantics:
            self.get_logger().info(
                f"Loading YOLO model: {self.model_path}"
            )
            self.model = YOLO(self.model_path)
            self.get_logger().info("YOLO model loaded")

        self.tracks: Dict[int, TrackState] = {}
        self.next_track_id = 0
        self.cached_detections = []
        self.semantic_counter = 0

        self.last_raw_points = 0
        self.last_filtered_points = 0
        self.last_motion_cells = 0
        self.last_candidates = 0
        self.last_tracks = 0
        self.last_bev_ms = 0.0
        self.last_motion_ms = 0.0
        self.last_track_ms = 0.0
        self.last_yolo_ms = 0.0
        self.last_render_ms = 0.0
        self.last_pipeline_ms = 0.0
        self.last_log_time = 0.0

        if self.show_gui:
            cv2.namedWindow(
                self.window_name,
                cv2.WINDOW_NORMAL,
            )
            cv2.resizeWindow(
                self.window_name,
                self.window_width,
                self.window_height,
            )

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _declare_parameters(self) -> None:
        self.declare_parameter("model_path", "yolo11s-seg.pt")
        self.declare_parameter("lidar_topic", "/lidar_points")
        self.declare_parameter("lidar_points_frame", "lidar")
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter(
            "obstacle_marker_topic",
            "/dynamic_obstacle_markers",
        )
        self.declare_parameter(
            "dynamic_cloud_topic",
            "/dynamic_obstacle_points",
        )

        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 30)
        self.declare_parameter("image_rotate", "none")

        self.declare_parameter("enable_front_semantics", True)
        self.declare_parameter("semantic_every_n_frames", 3)
        self.declare_parameter("yolo_imgsz", 640)
        self.declare_parameter("yolo_confidence", 0.35)
        self.declare_parameter("yolo_iou", 0.55)
        self.declare_parameter("yolo_device", "0")
        self.declare_parameter("use_half", True)

        self.declare_parameter("target_fps", 10.0)

        self.declare_parameter("camera_base_x", 0.350)
        self.declare_parameter("camera_base_y", 0.010)
        self.declare_parameter("camera_base_z", 0.270)
        self.declare_parameter("camera_roll_deg", 0.6)
        self.declare_parameter("camera_pitch_deg", -9.8)
        self.declare_parameter("camera_yaw_deg", 1.9)

        self.declare_parameter("lidar_base_x", 0.150)
        self.declare_parameter("lidar_base_y", 0.000)
        self.declare_parameter("lidar_base_z", 0.250)
        self.declare_parameter("lidar_roll_deg", -6.0)
        self.declare_parameter("lidar_pitch_deg", 0.0)
        self.declare_parameter("lidar_yaw_deg", 90.0)

        self.declare_parameter("min_range_m", 0.30)
        self.declare_parameter("max_range_m", 10.0)
        self.declare_parameter("height_min_m", 0.12)
        self.declare_parameter("height_max_m", 2.50)

        self.declare_parameter("robot_x_min_m", -0.60)
        self.declare_parameter("robot_x_max_m", 0.60)
        self.declare_parameter("robot_y_min_m", -0.40)
        self.declare_parameter("robot_y_max_m", 0.40)
        self.declare_parameter("robot_z_min_m", -0.30)
        self.declare_parameter("robot_z_max_m", 0.65)

        self.declare_parameter("bev_range_m", 8.0)
        self.declare_parameter("bev_resolution_m", 0.10)
        self.declare_parameter("min_points_per_cell", 1)

        self.declare_parameter("background_warmup_frames", 20)
        self.declare_parameter("background_alpha", 0.03)
        self.declare_parameter("background_threshold", 0.65)
        self.declare_parameter("motion_use_frame_difference", True)

        self.declare_parameter("motion_open_kernel", 2)
        self.declare_parameter("motion_close_kernel", 5)
        self.declare_parameter("motion_dilate_kernel", 5)
        self.declare_parameter("min_motion_cells", 3)
        self.declare_parameter("max_motion_cells", 500)
        self.declare_parameter("component_margin_cells", 2)

        self.declare_parameter("min_candidate_points", 4)
        self.declare_parameter("min_candidate_height_m", 0.12)
        self.declare_parameter("max_candidate_height_m", 2.50)
        self.declare_parameter("max_candidate_width_m", 2.50)
        self.declare_parameter("max_candidate_depth_m", 2.50)
        self.declare_parameter("box_percentile_low", 5.0)
        self.declare_parameter("box_percentile_high", 95.0)

        self.declare_parameter("track_match_distance_m", 0.90)
        self.declare_parameter("track_velocity_alpha", 0.50)
        self.declare_parameter("track_max_missed", 5)
        self.declare_parameter("dynamic_speed_threshold_mps", 0.12)
        self.declare_parameter("dynamic_confirm_age", 2)

        self.declare_parameter("front_half_angle_deg", 55.0)
        self.declare_parameter("rear_half_angle_deg", 55.0)
        self.declare_parameter("camera_association_margin_px", 35)

        self.declare_parameter("show_gui", True)
        self.declare_parameter("window_width", 1500)
        self.declare_parameter("window_height", 820)
        self.declare_parameter("marker_lifetime_sec", 0.35)

    def _load_parameters(self) -> None:
        value = lambda name: self.get_parameter(name).value

        string_names = [
            "model_path",
            "lidar_topic",
            "lidar_points_frame",
            "output_frame",
            "obstacle_marker_topic",
            "dynamic_cloud_topic",
            "image_rotate",
            "yolo_device",
        ]
        integer_names = [
            "camera_width",
            "camera_height",
            "camera_fps",
            "semantic_every_n_frames",
            "yolo_imgsz",
            "min_points_per_cell",
            "background_warmup_frames",
            "motion_open_kernel",
            "motion_close_kernel",
            "motion_dilate_kernel",
            "min_motion_cells",
            "max_motion_cells",
            "component_margin_cells",
            "min_candidate_points",
            "track_max_missed",
            "dynamic_confirm_age",
            "camera_association_margin_px",
            "window_width",
            "window_height",
        ]
        float_names = [
            "yolo_confidence",
            "yolo_iou",
            "target_fps",
            "camera_base_x",
            "camera_base_y",
            "camera_base_z",
            "camera_roll_deg",
            "camera_pitch_deg",
            "camera_yaw_deg",
            "lidar_base_x",
            "lidar_base_y",
            "lidar_base_z",
            "lidar_roll_deg",
            "lidar_pitch_deg",
            "lidar_yaw_deg",
            "min_range_m",
            "max_range_m",
            "height_min_m",
            "height_max_m",
            "robot_x_min_m",
            "robot_x_max_m",
            "robot_y_min_m",
            "robot_y_max_m",
            "robot_z_min_m",
            "robot_z_max_m",
            "bev_range_m",
            "bev_resolution_m",
            "background_alpha",
            "background_threshold",
            "min_candidate_height_m",
            "max_candidate_height_m",
            "max_candidate_width_m",
            "max_candidate_depth_m",
            "box_percentile_low",
            "box_percentile_high",
            "track_match_distance_m",
            "track_velocity_alpha",
            "dynamic_speed_threshold_mps",
            "front_half_angle_deg",
            "rear_half_angle_deg",
            "marker_lifetime_sec",
        ]
        boolean_names = [
            "enable_front_semantics",
            "use_half",
            "motion_use_frame_difference",
            "show_gui",
        ]

        for name in string_names:
            setattr(self, name, str(value(name)))
        for name in integer_names:
            setattr(self, name, int(value(name)))
        for name in float_names:
            setattr(self, name, float(value(name)))
        for name in boolean_names:
            setattr(self, name, bool(value(name)))

        self.window_name = "LiDAR Vision V5.0 360 Motion BEV"

    def _validate_parameters(self) -> None:
        if self.lidar_points_frame not in ("lidar", "base"):
            raise ValueError(
                "lidar_points_frame must be 'lidar' or 'base'"
            )
        if self.image_rotate not in (
            "none",
            "cw90",
            "ccw90",
            "180",
        ):
            raise ValueError(
                "image_rotate must be none/cw90/ccw90/180"
            )

    def _signal_handler(self, _signum, _frame) -> None:
        self.running = False

    def _cloud_callback(self, msg: PointCloud2) -> None:
        xyz = self._parse_xyz(msg)

        if xyz is None or len(xyz) == 0:
            return

        self.last_raw_points = int(len(xyz))

        if self.lidar_points_frame == "lidar":
            xyz_base = (
                xyz @ self.R_base_lidar.T
                + self.t_base_lidar.reshape(1, 3)
            )
        else:
            xyz_base = xyz.copy()

        planar_range = np.linalg.norm(
            xyz_base[:, :2],
            axis=1,
        )

        keep = (
            np.isfinite(xyz_base).all(axis=1)
            & (planar_range >= self.min_range_m)
            & (planar_range <= self.max_range_m)
            & (xyz_base[:, 2] >= self.height_min_m)
            & (xyz_base[:, 2] <= self.height_max_m)
        )

        robot_body = (
            (xyz_base[:, 0] >= self.robot_x_min_m)
            & (xyz_base[:, 0] <= self.robot_x_max_m)
            & (xyz_base[:, 1] >= self.robot_y_min_m)
            & (xyz_base[:, 1] <= self.robot_y_max_m)
            & (xyz_base[:, 2] >= self.robot_z_min_m)
            & (xyz_base[:, 2] <= self.robot_z_max_m)
        )

        keep &= ~robot_body
        xyz_base = xyz_base[keep]

        self.last_filtered_points = int(len(xyz_base))

        frame = CloudFrame(
            xyz_base=xyz_base.astype(
                np.float32,
                copy=True,
            ),
            stamp=msg.header.stamp,
        )

        with self.cloud_lock:
            self.latest_cloud = frame

    @staticmethod
    def _parse_xyz(
        msg: PointCloud2,
    ) -> Optional[np.ndarray]:
        fields = {
            field.name: field
            for field in msg.fields
        }

        if not all(
            name in fields
            for name in ("x", "y", "z")
        ):
            return None

        dtype = np.dtype({
            "names": ["x", "y", "z"],
            "formats": [
                np.float32,
                np.float32,
                np.float32,
            ],
            "offsets": [
                fields["x"].offset,
                fields["y"].offset,
                fields["z"].offset,
            ],
            "itemsize": msg.point_step,
        })

        data = np.frombuffer(
            msg.data,
            dtype=dtype,
        )

        xyz = np.column_stack((
            data["x"],
            data["y"],
            data["z"],
        )).astype(
            np.float32,
            copy=False,
        )

        return xyz[
            np.isfinite(xyz).all(axis=1)
        ]

    def _latest_cloud_copy(
        self,
    ) -> Optional[CloudFrame]:
        with self.cloud_lock:
            if self.latest_cloud is None:
                return None

            cloud = self.latest_cloud
            return CloudFrame(
                xyz_base=cloud.xyz_base.copy(),
                stamp=cloud.stamp,
            )

    def run(self) -> None:
        period = 1.0 / max(
            self.target_fps,
            1.0,
        )
        last_process_time = 0.0

        while rclpy.ok() and self.running:
            current_time = time.monotonic()

            if current_time - last_process_time < period:
                self._handle_gui()
                time.sleep(0.001)
                continue

            last_process_time = current_time
            pipeline_start = time.perf_counter()

            cloud = self._latest_cloud_copy()
            image = self._read_camera_frame()

            bev_start = time.perf_counter()
            occupancy, cell_count = self._build_occupancy(
                cloud,
            )
            self.last_bev_ms = (
                time.perf_counter() - bev_start
            ) * 1000.0

            motion_start = time.perf_counter()
            motion_mask = self._build_motion_mask(
                occupancy,
            )
            candidates = self._motion_candidates(
                motion_mask,
                cell_count,
                cloud,
            )
            self.last_motion_ms = (
                time.perf_counter() - motion_start
            ) * 1000.0

            track_start = time.perf_counter()
            tracks = self._update_tracks(
                candidates,
                cloud,
            )
            self.last_track_ms = (
                time.perf_counter() - track_start
            ) * 1000.0

            front_tracks = [
                track
                for track in tracks
                if track.sector == "front"
            ]

            if (
                self.enable_front_semantics
                and self.model is not None
                and image is not None
                and front_tracks
            ):
                self.semantic_counter += 1

                if (
                    self.semantic_counter
                    % max(
                        self.semantic_every_n_frames,
                        1,
                    )
                    == 0
                ):
                    yolo_start = time.perf_counter()
                    self.cached_detections = (
                        self._run_yolo(image)
                    )
                    self.last_yolo_ms = (
                        time.perf_counter()
                        - yolo_start
                    ) * 1000.0

                self._apply_front_semantics(
                    front_tracks,
                    image.shape,
                    self.cached_detections,
                )
            else:
                self.last_yolo_ms = 0.0

            header = Header()
            header.frame_id = self.output_frame
            header.stamp = (
                cloud.stamp
                if cloud is not None
                else self.get_clock().now().to_msg()
            )

            self._publish_markers(
                tracks,
                header,
            )
            self._publish_dynamic_cloud(
                tracks,
                header,
            )

            render_start = time.perf_counter()

            if self.show_gui:
                canvas = self._render_dashboard(
                    image,
                    cloud,
                    occupancy,
                    motion_mask,
                    tracks,
                )
                cv2.imshow(
                    self.window_name,
                    canvas,
                )
                self._handle_gui()

            self.last_render_ms = (
                time.perf_counter()
                - render_start
            ) * 1000.0
            self.last_pipeline_ms = (
                time.perf_counter()
                - pipeline_start
            ) * 1000.0

            self._log_status(
                cloud,
                tracks,
            )

        self.stop()

    def _build_occupancy(
        self,
        cloud: Optional[CloudFrame],
    ) -> Tuple[np.ndarray, np.ndarray]:
        occupancy = np.zeros(
            (self.grid_size, self.grid_size),
            dtype=np.uint8,
        )
        cell_count = np.zeros(
            (self.grid_size, self.grid_size),
            dtype=np.uint16,
        )

        if cloud is None or len(cloud.xyz_base) == 0:
            return occupancy, cell_count

        points = cloud.xyz_base
        inside = (
            (points[:, 0] >= -self.bev_range_m)
            & (points[:, 0] < self.bev_range_m)
            & (points[:, 1] >= -self.bev_range_m)
            & (points[:, 1] < self.bev_range_m)
        )
        points = points[inside]

        grid_x = np.floor(
            (points[:, 0] + self.bev_range_m)
            / self.bev_resolution_m
        ).astype(np.int32)
        grid_y = np.floor(
            (points[:, 1] + self.bev_range_m)
            / self.bev_resolution_m
        ).astype(np.int32)

        valid = (
            (grid_x >= 0)
            & (grid_x < self.grid_size)
            & (grid_y >= 0)
            & (grid_y < self.grid_size)
        )
        grid_x = grid_x[valid]
        grid_y = grid_y[valid]

        np.add.at(
            cell_count,
            (grid_x, grid_y),
            1,
        )

        occupancy[
            cell_count >= self.min_points_per_cell
        ] = 255

        return occupancy, cell_count

    def _build_motion_mask(
        self,
        occupancy: np.ndarray,
    ) -> np.ndarray:
        self.frame_count += 1

        current_float = (
            occupancy.astype(np.float32)
            / 255.0
        )

        if (
            self.frame_count
            <= self.background_warmup_frames
        ):
            alpha = 1.0 / float(self.frame_count)
            self.background_probability = (
                (1.0 - alpha)
                * self.background_probability
                + alpha * current_float
            )
            self.previous_occupancy = occupancy.copy()
            self.last_motion_cells = 0
            return np.zeros_like(occupancy)

        background_static = (
            self.background_probability
            >= self.background_threshold
        ).astype(np.uint8) * 255

        new_occupancy = cv2.bitwise_and(
            occupancy,
            cv2.bitwise_not(background_static),
        )

        if self.motion_use_frame_difference:
            frame_difference = cv2.absdiff(
                occupancy,
                self.previous_occupancy,
            )
            motion_mask = cv2.bitwise_or(
                new_occupancy,
                frame_difference,
            )
        else:
            motion_mask = new_occupancy

        if self.motion_open_kernel > 1:
            kernel = np.ones(
                (
                    self.motion_open_kernel,
                    self.motion_open_kernel,
                ),
                dtype=np.uint8,
            )
            motion_mask = cv2.morphologyEx(
                motion_mask,
                cv2.MORPH_OPEN,
                kernel,
            )

        if self.motion_close_kernel > 1:
            kernel = np.ones(
                (
                    self.motion_close_kernel,
                    self.motion_close_kernel,
                ),
                dtype=np.uint8,
            )
            motion_mask = cv2.morphologyEx(
                motion_mask,
                cv2.MORPH_CLOSE,
                kernel,
            )

        if self.motion_dilate_kernel > 1:
            kernel = np.ones(
                (
                    self.motion_dilate_kernel,
                    self.motion_dilate_kernel,
                ),
                dtype=np.uint8,
            )
            motion_mask = cv2.dilate(
                motion_mask,
                kernel,
                iterations=1,
            )

        stable_mask = (
            motion_mask == 0
        )
        alpha = self.background_alpha

        self.background_probability[
            stable_mask
        ] = (
            (1.0 - alpha)
            * self.background_probability[stable_mask]
            + alpha
            * current_float[stable_mask]
        )

        self.previous_occupancy = occupancy.copy()
        self.last_motion_cells = int(
            np.count_nonzero(motion_mask)
        )

        return motion_mask

    def _motion_candidates(
        self,
        motion_mask: np.ndarray,
        cell_count: np.ndarray,
        cloud: Optional[CloudFrame],
    ) -> List[MotionCandidate]:
        if (
            cloud is None
            or len(cloud.xyz_base) == 0
            or np.count_nonzero(motion_mask) == 0
        ):
            self.last_candidates = 0
            return []

        component_count, labels, stats, _centroids = (
            cv2.connectedComponentsWithStats(
                motion_mask,
                connectivity=8,
            )
        )

        candidates = []
        candidate_id = 0

        points = cloud.xyz_base
        point_grid_x = np.floor(
            (points[:, 0] + self.bev_range_m)
            / self.bev_resolution_m
        ).astype(np.int32)
        point_grid_y = np.floor(
            (points[:, 1] + self.bev_range_m)
            / self.bev_resolution_m
        ).astype(np.int32)

        valid_grid = (
            (point_grid_x >= 0)
            & (point_grid_x < self.grid_size)
            & (point_grid_y >= 0)
            & (point_grid_y < self.grid_size)
        )

        for label_id in range(1, component_count):
            left = int(stats[label_id, cv2.CC_STAT_LEFT])
            top = int(stats[label_id, cv2.CC_STAT_TOP])
            width = int(stats[label_id, cv2.CC_STAT_WIDTH])
            height = int(stats[label_id, cv2.CC_STAT_HEIGHT])
            area = int(stats[label_id, cv2.CC_STAT_AREA])

            if (
                area < self.min_motion_cells
                or area > self.max_motion_cells
            ):
                continue

            margin = self.component_margin_cells
            gx_min = max(0, top - margin)
            gx_max = min(
                self.grid_size - 1,
                top + height - 1 + margin,
            )
            gy_min = max(0, left - margin)
            gy_max = min(
                self.grid_size - 1,
                left + width - 1 + margin,
            )

            point_mask = (
                valid_grid
                & (point_grid_x >= gx_min)
                & (point_grid_x <= gx_max)
                & (point_grid_y >= gy_min)
                & (point_grid_y <= gy_max)
            )

            candidate_points = points[point_mask]

            if (
                len(candidate_points)
                < self.min_candidate_points
            ):
                continue

            box_min = np.percentile(
                candidate_points,
                self.box_percentile_low,
                axis=0,
            )
            box_max = np.percentile(
                candidate_points,
                self.box_percentile_high,
                axis=0,
            )
            box_size = box_max - box_min

            if not (
                self.min_candidate_height_m
                <= box_size[2]
                <= self.max_candidate_height_m
            ):
                continue

            if (
                box_size[1] > self.max_candidate_width_m
                or box_size[0] > self.max_candidate_depth_m
            ):
                continue

            center = np.median(
                candidate_points,
                axis=0,
            ).astype(np.float32)

            candidates.append(MotionCandidate(
                candidate_id=candidate_id,
                center=center,
                box_min=box_min.astype(np.float32),
                box_max=box_max.astype(np.float32),
                points=candidate_points.astype(np.float32),
                sector=self._sector_from_xy(
                    center[0],
                    center[1],
                ),
            ))
            candidate_id += 1

        self.last_candidates = len(candidates)
        return candidates

    def _update_tracks(
        self,
        candidates: List[MotionCandidate],
        cloud: Optional[CloudFrame],
    ) -> List[TrackState]:
        stamp_sec = (
            self._stamp_to_sec(cloud.stamp)
            if cloud is not None
            else time.time()
        )

        unmatched_tracks = set(self.tracks.keys())
        unmatched_candidates = set(range(len(candidates)))
        pair_costs = []

        for track_id, track in self.tracks.items():
            dt = max(
                stamp_sec - track.last_stamp_sec,
                1e-3,
            )
            predicted = (
                track.center
                + track.velocity * dt
            )

            for candidate_index, candidate in enumerate(
                candidates
            ):
                distance = float(
                    np.linalg.norm(
                        candidate.center[:2]
                        - predicted[:2]
                    )
                )

                if distance <= self.track_match_distance_m:
                    pair_costs.append((
                        distance,
                        track_id,
                        candidate_index,
                    ))

        pair_costs.sort(
            key=lambda item: item[0]
        )

        for _, track_id, candidate_index in pair_costs:
            if (
                track_id not in unmatched_tracks
                or candidate_index not in unmatched_candidates
            ):
                continue

            track = self.tracks[track_id]
            candidate = candidates[candidate_index]
            dt = max(
                stamp_sec - track.last_stamp_sec,
                1e-3,
            )

            measured_velocity = (
                candidate.center
                - track.center
            ) / dt

            alpha = self.track_velocity_alpha
            track.velocity = (
                alpha * measured_velocity
                + (1.0 - alpha)
                * track.velocity
            ).astype(np.float32)
            track.center = candidate.center.copy()
            track.box_min = candidate.box_min.copy()
            track.box_max = candidate.box_max.copy()
            track.points = candidate.points.copy()
            track.sector = candidate.sector
            track.last_stamp_sec = stamp_sec
            track.age += 1
            track.missed = 0

            unmatched_tracks.remove(track_id)
            unmatched_candidates.remove(candidate_index)

        for track_id in list(unmatched_tracks):
            track = self.tracks[track_id]
            track.missed += 1

            if track.missed > self.track_max_missed:
                del self.tracks[track_id]

        for candidate_index in unmatched_candidates:
            candidate = candidates[candidate_index]
            track_id = self.next_track_id
            self.next_track_id += 1

            self.tracks[track_id] = TrackState(
                track_id=track_id,
                center=candidate.center.copy(),
                velocity=np.zeros(
                    3,
                    dtype=np.float32,
                ),
                box_min=candidate.box_min.copy(),
                box_max=candidate.box_max.copy(),
                points=candidate.points.copy(),
                sector=candidate.sector,
                last_stamp_sec=stamp_sec,
            )

        active_tracks = [
            track
            for track in self.tracks.values()
            if track.missed == 0
        ]
        active_tracks.sort(
            key=lambda track: track.track_id
        )
        self.last_tracks = len(active_tracks)

        return active_tracks

    def _run_yolo(
        self,
        image: np.ndarray,
    ):
        result = self.model.predict(
            source=image,
            imgsz=self.yolo_imgsz,
            conf=self.yolo_confidence,
            iou=self.yolo_iou,
            device=self.yolo_device,
            half=self.use_half,
            verbose=False,
        )[0]

        detections = []

        if result.boxes is None:
            return detections

        names = result.names

        for box, confidence, class_id in zip(
            result.boxes.xyxy.cpu().numpy(),
            result.boxes.conf.cpu().numpy(),
            result.boxes.cls.cpu().numpy().astype(np.int32),
        ):
            detections.append({
                "bbox": tuple(
                    box.astype(float).tolist()
                ),
                "confidence": float(confidence),
                "label": str(
                    names[int(class_id)]
                ),
            })

        return detections

    def _apply_front_semantics(
        self,
        front_tracks: List[TrackState],
        image_shape,
        detections,
    ) -> None:
        if not detections:
            return

        image_height, image_width = image_shape[:2]

        for track in front_tracks:
            uv = self._project_base_point(
                track.center
            )

            if uv is None:
                continue

            u, v = uv

            if not (
                0 <= u < image_width
                and 0 <= v < image_height
            ):
                continue

            best = None
            best_area = float("inf")

            for detection in detections:
                x1, y1, x2, y2 = (
                    detection["bbox"]
                )
                margin = (
                    self.camera_association_margin_px
                )

                if (
                    x1 - margin <= u <= x2 + margin
                    and y1 - margin <= v <= y2 + margin
                ):
                    area = max(
                        1.0,
                        (x2 - x1) * (y2 - y1),
                    )

                    if area < best_area:
                        best_area = area
                        best = detection

            if best is not None:
                track.semantic_label = best["label"]
                track.semantic_confidence = best["confidence"]

    def _project_base_point(
        self,
        point_base: np.ndarray,
    ) -> Optional[Tuple[float, float]]:
        relative = (
            point_base
            - self.t_base_camera
        )
        camera_body = (
            relative @ self.R_base_camera
        )

        x_optical = -camera_body[1]
        y_optical = -camera_body[2]
        z_optical = camera_body[0]

        if z_optical <= 0.05:
            return None

        u = (
            self.fx * x_optical
            / z_optical
            + self.cx
        )
        v = (
            self.fy * y_optical
            / z_optical
            + self.cy
        )

        if self.image_rotate == "cw90":
            return (
                self.raw_camera_height - 1.0 - v,
                u,
            )

        if self.image_rotate == "ccw90":
            return (
                v,
                self.raw_camera_width - 1.0 - u,
            )

        if self.image_rotate == "180":
            return (
                self.raw_camera_width - 1.0 - u,
                self.raw_camera_height - 1.0 - v,
            )

        return u, v

    def _read_camera_frame(
        self,
    ) -> Optional[np.ndarray]:
        try:
            frames = self.pipeline.poll_for_frames()

            if not frames:
                return None

            color_frame = frames.get_color_frame()

            if not color_frame:
                return None

            image = np.asanyarray(
                color_frame.get_data()
            )

            if self.image_rotate == "cw90":
                return cv2.rotate(
                    image,
                    cv2.ROTATE_90_CLOCKWISE,
                )

            if self.image_rotate == "ccw90":
                return cv2.rotate(
                    image,
                    cv2.ROTATE_90_COUNTERCLOCKWISE,
                )

            if self.image_rotate == "180":
                return cv2.rotate(
                    image,
                    cv2.ROTATE_180,
                )

            return image

        except Exception as error:
            self.get_logger().warning(
                f"RealSense read failed: {error}"
            )
            return None

    def _sector_from_xy(
        self,
        x: float,
        y: float,
    ) -> str:
        angle = math.degrees(
            math.atan2(y, x)
        )
        absolute_angle = abs(angle)

        if absolute_angle <= self.front_half_angle_deg:
            return "front"

        if (
            absolute_angle
            >= 180.0 - self.rear_half_angle_deg
        ):
            return "rear"

        return "left" if y >= 0.0 else "right"

    def _render_dashboard(
        self,
        image: Optional[np.ndarray],
        cloud: Optional[CloudFrame],
        occupancy: np.ndarray,
        motion_mask: np.ndarray,
        tracks: List[TrackState],
    ) -> np.ndarray:
        camera_panel = self._render_camera_panel(
            image,
            tracks,
        )
        bev_panel = self._render_bev_panel(
            occupancy,
            motion_mask,
            tracks,
        )

        camera_panel = cv2.resize(
            camera_panel,
            (700, 560),
        )
        bev_panel = cv2.resize(
            bev_panel,
            (700, 560),
        )

        dashboard = np.zeros(
            (720, 1400, 3),
            dtype=np.uint8,
        )
        dashboard[:560, :700] = camera_panel
        dashboard[:560, 700:] = bev_panel

        dynamic_tracks = [
            track
            for track in tracks
            if (
                track.age >= self.dynamic_confirm_age
                and np.linalg.norm(
                    track.velocity[:2]
                )
                >= self.dynamic_speed_threshold_mps
            )
        ]

        status_lines = [
            "V5.0 360 MOTION BEV | LiDAR primary / front camera semantic assist",
            (
                f"raw={self.last_raw_points} "
                f"filtered={self.last_filtered_points} "
                f"motion_cells={self.last_motion_cells} "
                f"candidates={self.last_candidates} "
                f"tracks={self.last_tracks} "
                f"dynamic={len(dynamic_tracks)}"
            ),
            (
                f"bev={self.last_bev_ms:.1f}ms "
                f"motion={self.last_motion_ms:.1f}ms "
                f"track={self.last_track_ms:.1f}ms "
                f"yolo={self.last_yolo_ms:.1f}ms "
                f"render={self.last_render_ms:.1f}ms "
                f"pipeline={self.last_pipeline_ms:.1f}ms"
            ),
            (
                f"cloud_age={self._cloud_age_ms(cloud):.1f}ms "
                f"warmup={min(self.frame_count, self.background_warmup_frames)}"
                f"/{self.background_warmup_frames}"
            ),
            "q or ESC: quit",
        ]

        for row, text in enumerate(status_lines):
            cv2.putText(
                dashboard,
                text,
                (18, 590 + row * 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        return dashboard

    def _render_camera_panel(
        self,
        image: Optional[np.ndarray],
        tracks: List[TrackState],
    ) -> np.ndarray:
        if image is None:
            panel = np.zeros(
                (480, 640, 3),
                dtype=np.uint8,
            )
            cv2.putText(
                panel,
                "NO CAMERA FRAME",
                (160, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
            )
            return panel

        panel = image.copy()

        for track in tracks:
            if track.sector != "front":
                continue

            uv = self._project_base_point(
                track.center
            )

            if uv is None:
                continue

            u, v = map(int, uv)
            speed = float(
                np.linalg.norm(
                    track.velocity[:2]
                )
            )
            dynamic = (
                track.age >= self.dynamic_confirm_age
                and speed
                >= self.dynamic_speed_threshold_mps
            )

            color = (
                (0, 0, 255)
                if dynamic
                else (0, 200, 255)
            )

            cv2.circle(
                panel,
                (u, v),
                7,
                color,
                -1,
            )
            cv2.putText(
                panel,
                (
                    f"ID{track.track_id} "
                    f"{track.semantic_label} "
                    f"{speed:.2f}m/s"
                ),
                (u + 8, v - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.47,
                color,
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            panel,
            "FRONT SEMANTIC ASSIST",
            (15, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        return panel

    def _render_bev_panel(
        self,
        occupancy: np.ndarray,
        motion_mask: np.ndarray,
        tracks: List[TrackState],
    ) -> np.ndarray:
        occupancy_view = cv2.rotate(
            occupancy,
            cv2.ROTATE_90_CLOCKWISE,
        )
        motion_view = cv2.rotate(
            motion_mask,
            cv2.ROTATE_90_CLOCKWISE,
        )

        panel = np.full(
            (
                self.grid_size,
                self.grid_size,
                3,
            ),
            245,
            dtype=np.uint8,
        )

        panel[
            occupancy_view > 0
        ] = (190, 190, 190)
        panel[
            motion_view > 0
        ] = (0, 170, 255)

        display_size = 700
        panel = cv2.resize(
            panel,
            (display_size, display_size),
            interpolation=cv2.INTER_NEAREST,
        )

        scale = (
            display_size
            / (2.0 * self.bev_range_m)
        )
        origin = np.array(
            [
                display_size // 2,
                display_size // 2,
            ],
            dtype=np.int32,
        )

        for track in tracks:
            px = int(
                origin[0]
                - track.center[1] * scale
            )
            py = int(
                origin[1]
                - track.center[0] * scale
            )

            speed = float(
                np.linalg.norm(
                    track.velocity[:2]
                )
            )
            dynamic = (
                track.age >= self.dynamic_confirm_age
                and speed
                >= self.dynamic_speed_threshold_mps
            )

            color = (
                (0, 0, 255)
                if dynamic
                else (0, 170, 0)
            )

            cv2.circle(
                panel,
                (px, py),
                7,
                color,
                -1,
            )

            velocity_end = (
                int(
                    px
                    - track.velocity[1]
                    * scale
                ),
                int(
                    py
                    - track.velocity[0]
                    * scale
                ),
            )

            cv2.arrowedLine(
                panel,
                (px, py),
                velocity_end,
                color,
                2,
                tipLength=0.25,
            )

            cv2.putText(
                panel,
                (
                    f"ID{track.track_id} "
                    f"{track.sector} "
                    f"{track.semantic_label} "
                    f"{speed:.2f}"
                ),
                (px + 8, py - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (20, 20, 20),
                2,
                cv2.LINE_AA,
            )

        cv2.rectangle(
            panel,
            (
                origin[0] - 10,
                origin[1] - 15,
            ),
            (
                origin[0] + 10,
                origin[1] + 15,
            ),
            (20, 20, 20),
            2,
        )
        cv2.arrowedLine(
            panel,
            tuple(origin),
            (
                origin[0],
                origin[1] - 45,
            ),
            (0, 90, 255),
            3,
            tipLength=0.25,
        )

        cv2.putText(
            panel,
            "360 BEV: GRAY=OCCUPIED / ORANGE=MOTION",
            (15, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.66,
            (20, 20, 20),
            2,
            cv2.LINE_AA,
        )

        return panel

    def _publish_markers(
        self,
        tracks: List[TrackState],
        header: Header,
    ) -> None:
        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.header = header
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(
            clear_marker
        )

        lifetime = Duration(
            seconds=self.marker_lifetime_sec
        ).to_msg()

        for track in tracks:
            speed = float(
                np.linalg.norm(
                    track.velocity[:2]
                )
            )
            dynamic = (
                track.age >= self.dynamic_confirm_age
                and speed
                >= self.dynamic_speed_threshold_mps
            )

            center = 0.5 * (
                track.box_min
                + track.box_max
            )
            size = np.maximum(
                track.box_max
                - track.box_min,
                np.array(
                    [0.10, 0.10, 0.20],
                    dtype=np.float32,
                ),
            )

            box = Marker()
            box.header = header
            box.ns = "motion_obstacle_boxes"
            box.id = track.track_id * 3
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float(center[0])
            box.pose.position.y = float(center[1])
            box.pose.position.z = float(center[2])
            box.pose.orientation.w = 1.0
            box.scale.x = float(size[0])
            box.scale.y = float(size[1])
            box.scale.z = float(size[2])

            if dynamic:
                box.color.r = 1.0
                box.color.g = 0.1
                box.color.b = 0.1
            else:
                box.color.r = 0.1
                box.color.g = 0.8
                box.color.b = 0.2

            box.color.a = 0.35
            box.lifetime = lifetime
            marker_array.markers.append(box)

            arrow = Marker()
            arrow.header = header
            arrow.ns = "motion_obstacle_velocity"
            arrow.id = track.track_id * 3 + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = float(track.center[0])
            arrow.pose.position.y = float(track.center[1])
            arrow.pose.position.z = float(track.center[2])
            yaw = math.atan2(
                track.velocity[1],
                track.velocity[0],
            )
            arrow.pose.orientation.z = math.sin(yaw * 0.5)
            arrow.pose.orientation.w = math.cos(yaw * 0.5)
            arrow.scale.x = max(0.05, speed)
            arrow.scale.y = 0.06
            arrow.scale.z = 0.06
            arrow.color.r = 1.0
            arrow.color.g = 0.5
            arrow.color.b = 0.0
            arrow.color.a = 1.0
            arrow.lifetime = lifetime
            marker_array.markers.append(arrow)

            text = Marker()
            text.header = header
            text.ns = "motion_obstacle_text"
            text.id = track.track_id * 3 + 2
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(track.center[0])
            text.pose.position.y = float(track.center[1])
            text.pose.position.z = float(
                track.box_max[2] + 0.25
            )
            text.pose.orientation.w = 1.0
            text.scale.z = 0.18
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = (
                f"ID{track.track_id} "
                f"{track.sector} "
                f"{track.semantic_label} "
                f"{speed:.2f}m/s"
            )
            text.lifetime = lifetime
            marker_array.markers.append(text)

        self.marker_pub.publish(
            marker_array
        )

    def _publish_dynamic_cloud(
        self,
        tracks: List[TrackState],
        header: Header,
    ) -> None:
        dynamic_points = []

        for track in tracks:
            speed = float(
                np.linalg.norm(
                    track.velocity[:2]
                )
            )

            if (
                track.age >= self.dynamic_confirm_age
                and speed
                >= self.dynamic_speed_threshold_mps
                and len(track.points) > 0
            ):
                dynamic_points.append(
                    track.points
                )

        if dynamic_points:
            points = np.concatenate(
                dynamic_points,
                axis=0,
            )
        else:
            points = np.empty(
                (0, 3),
                dtype=np.float32,
            )

        message = PointCloud2()
        message.header = header
        message.height = 1
        message.width = int(len(points))
        message.fields = [
            PointField(
                name="x",
                offset=0,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="y",
                offset=4,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="z",
                offset=8,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = (
            message.point_step
            * message.width
        )
        message.is_dense = True
        message.data = points.astype(
            np.float32,
            copy=False,
        ).tobytes()

        self.dynamic_cloud_pub.publish(
            message
        )

    def _cloud_age_ms(
        self,
        cloud: Optional[CloudFrame],
    ) -> float:
        if cloud is None:
            return float("nan")

        current_ns = (
            self.get_clock().now().nanoseconds
        )
        stamp_ns = (
            int(cloud.stamp.sec)
            * 1_000_000_000
            + int(cloud.stamp.nanosec)
        )

        return max(
            0.0,
            (
                current_ns - stamp_ns
            ) / 1_000_000.0,
        )

    def _log_status(
        self,
        cloud: Optional[CloudFrame],
        tracks: List[TrackState],
    ) -> None:
        current_time = time.monotonic()

        if (
            current_time - self.last_log_time
            < 1.0
        ):
            return

        self.last_log_time = current_time

        dynamic_count = sum(
            1
            for track in tracks
            if (
                track.age >= self.dynamic_confirm_age
                and np.linalg.norm(
                    track.velocity[:2]
                )
                >= self.dynamic_speed_threshold_mps
            )
        )

        self.get_logger().info(
            f"raw={self.last_raw_points} "
            f"filtered={self.last_filtered_points} "
            f"motion_cells={self.last_motion_cells} "
            f"candidates={self.last_candidates} "
            f"tracks={self.last_tracks} "
            f"dynamic={dynamic_count} "
            f"cloud_age={self._cloud_age_ms(cloud):.1f}ms "
            f"bev={self.last_bev_ms:.1f}ms "
            f"motion={self.last_motion_ms:.1f}ms "
            f"track={self.last_track_ms:.1f}ms "
            f"yolo={self.last_yolo_ms:.1f}ms "
            f"pipeline={self.last_pipeline_ms:.1f}ms"
        )

    def _handle_gui(self) -> None:
        if not self.show_gui:
            return

        key = cv2.waitKey(1) & 0xFF

        if key in (
            ord("q"),
            27,
        ):
            self.running = False

    def stop(self) -> None:
        self.running = False

        try:
            self.pipeline.stop()
        except Exception:
            pass

        if self.show_gui:
            cv2.destroyAllWindows()
            cv2.waitKey(1)

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return (
            float(stamp.sec)
            + float(stamp.nanosec)
            * 1e-9
        )

    @staticmethod
    def _rotation_matrix(
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ) -> np.ndarray:
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)

        rotation_x = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, cr, -sr],
                [0.0, sr, cr],
            ],
            dtype=np.float32,
        )
        rotation_y = np.array(
            [
                [cp, 0.0, sp],
                [0.0, 1.0, 0.0],
                [-sp, 0.0, cp],
            ],
            dtype=np.float32,
        )
        rotation_z = np.array(
            [
                [cy, -sy, 0.0],
                [sy, cy, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

        return (
            rotation_z
            @ rotation_y
            @ rotation_x
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = LidarVisionPerson3DV50()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(
        target=executor.spin,
        daemon=True,
    )
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import math
import struct
import threading
import time
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from unitree_go.msg import SportModeState


class LiDARVisionTopViewV411Node(Node):
    """
    V4.1 diagnostic node for coordinate and gravity-alignment verification.

    Inputs:
        - /lidar_points      : Hesai XT16 PointCloud2
        - /sportmodestate    : Unitree Go2 body IMU and motion state

    Outputs:
        - OpenCV side-by-side diagnostic view:
            left  : body-frame BEV before IMU gravity alignment
            right : gravity-aligned BEV

    Coordinate convention:
        x: forward
        y: left
        z: up

    Important:
        - This node does not perform person detection or motion tracking.
        - The goal is to verify coordinate direction, fixed LiDAR extrinsic,
          body roll/pitch compensation, ground plane orientation, and sector
          obstacle coordinates before rebuilding the object segmentation stage.
    """

    def __init__(self):
        super().__init__("lidar_vision_topview_v4_1_1_node")

        # ------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------
        self.declare_parameter("lidar_topic", "/lidar_points")
        self.declare_parameter("sportmode_topic", "/sportmodestate")

        # Latest calibration used by the previous LiDAR-Vision versions.
        self.declare_parameter("lidar_base_x", 0.150)
        self.declare_parameter("lidar_base_y", 0.000)
        self.declare_parameter("lidar_base_z", 0.250)
        self.declare_parameter("lidar_roll_deg", -6.0)
        self.declare_parameter("lidar_pitch_deg", 0.0)
        self.declare_parameter("lidar_yaw_deg", 90.0)

        # Gravity alignment
        self.declare_parameter("enable_gravity_alignment", True)
        self.declare_parameter("imu_roll_sign", 1.0)
        self.declare_parameter("imu_pitch_sign", 1.0)
        self.declare_parameter("imu_roll_offset_deg", 0.0)
        self.declare_parameter("imu_pitch_offset_deg", 0.0)
        self.declare_parameter("imu_timeout_sec", 0.20)
        self.declare_parameter("imu_lowpass_alpha", 0.15)

        # Point processing
        self.declare_parameter("min_range", 0.35)
        self.declare_parameter("max_range", 10.0)
        self.declare_parameter("min_z", -1.50)
        self.declare_parameter("max_z", 2.50)
        self.declare_parameter("self_x_min", -0.70)
        self.declare_parameter("self_x_max", 0.90)
        self.declare_parameter("self_y_min", -0.60)
        self.declare_parameter("self_y_max", 0.60)
        self.declare_parameter("self_z_min", -0.80)
        self.declare_parameter("self_z_max", 0.80)
        self.declare_parameter("voxel_size", 0.05)
        self.declare_parameter("max_display_points", 18000)

        # Ground-plane estimation
        self.declare_parameter("ground_fit_min_range", 0.80)
        self.declare_parameter("ground_fit_max_range", 5.00)
        self.declare_parameter("ground_fit_z_min", -1.20)
        self.declare_parameter("ground_fit_z_max", 0.25)
        self.declare_parameter("ground_fit_max_points", 5000)
        self.declare_parameter("ground_inlier_threshold", 0.08)
        self.declare_parameter("ground_refine_iterations", 3)
        self.declare_parameter("obstacle_ground_clearance", 0.12)

        # BEV and diagnostics
        self.declare_parameter("display_range_m", 6.0)
        self.declare_parameter("panel_size", 640)
        self.declare_parameter("display_fps", 5.0)
        self.declare_parameter("sector_half_angle_deg", 18.0)
        self.declare_parameter("sector_min_height", 0.10)
        self.declare_parameter("sector_max_height", 2.00)
        self.declare_parameter("print_interval_sec", 1.0)
        self.declare_parameter("window_name", "LiDAR V4.1.1 Coordinate Validation")
        self.declare_parameter("draw_sector_nearest_markers", False)
        self.declare_parameter("enable_click_coordinate_probe", True)
        self.declare_parameter("click_probe_radius_m", 0.20)
        self.declare_parameter("click_probe_min_points", 3)

        # ------------------------------------------------------------
        # Parameter values
        # ------------------------------------------------------------
        self.lidar_topic = str(self.get_parameter("lidar_topic").value)
        self.sportmode_topic = str(self.get_parameter("sportmode_topic").value)

        self.lidar_t = np.array(
            [
                float(self.get_parameter("lidar_base_x").value),
                float(self.get_parameter("lidar_base_y").value),
                float(self.get_parameter("lidar_base_z").value),
            ],
            dtype=np.float32,
        )
        self.lidar_rpy_deg = (
            float(self.get_parameter("lidar_roll_deg").value),
            float(self.get_parameter("lidar_pitch_deg").value),
            float(self.get_parameter("lidar_yaw_deg").value),
        )
        self.R_body_lidar = self.rotation_matrix_rpy(
            math.radians(self.lidar_rpy_deg[0]),
            math.radians(self.lidar_rpy_deg[1]),
            math.radians(self.lidar_rpy_deg[2]),
        )

        self.enable_gravity_alignment = bool(
            self.get_parameter("enable_gravity_alignment").value
        )
        self.imu_roll_sign = float(self.get_parameter("imu_roll_sign").value)
        self.imu_pitch_sign = float(self.get_parameter("imu_pitch_sign").value)
        self.imu_roll_offset = math.radians(
            float(self.get_parameter("imu_roll_offset_deg").value)
        )
        self.imu_pitch_offset = math.radians(
            float(self.get_parameter("imu_pitch_offset_deg").value)
        )
        self.imu_timeout_sec = float(
            self.get_parameter("imu_timeout_sec").value
        )
        self.imu_lowpass_alpha = float(
            self.get_parameter("imu_lowpass_alpha").value
        )

        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.min_z = float(self.get_parameter("min_z").value)
        self.max_z = float(self.get_parameter("max_z").value)

        self.self_bounds = (
            float(self.get_parameter("self_x_min").value),
            float(self.get_parameter("self_x_max").value),
            float(self.get_parameter("self_y_min").value),
            float(self.get_parameter("self_y_max").value),
            float(self.get_parameter("self_z_min").value),
            float(self.get_parameter("self_z_max").value),
        )

        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.max_display_points = int(
            self.get_parameter("max_display_points").value
        )

        self.ground_fit_min_range = float(
            self.get_parameter("ground_fit_min_range").value
        )
        self.ground_fit_max_range = float(
            self.get_parameter("ground_fit_max_range").value
        )
        self.ground_fit_z_min = float(
            self.get_parameter("ground_fit_z_min").value
        )
        self.ground_fit_z_max = float(
            self.get_parameter("ground_fit_z_max").value
        )
        self.ground_fit_max_points = int(
            self.get_parameter("ground_fit_max_points").value
        )
        self.ground_inlier_threshold = float(
            self.get_parameter("ground_inlier_threshold").value
        )
        self.ground_refine_iterations = int(
            self.get_parameter("ground_refine_iterations").value
        )
        self.obstacle_ground_clearance = float(
            self.get_parameter("obstacle_ground_clearance").value
        )

        self.display_range_m = float(
            self.get_parameter("display_range_m").value
        )
        self.panel_size = int(self.get_parameter("panel_size").value)
        self.display_fps = float(self.get_parameter("display_fps").value)
        self.sector_half_angle_deg = float(
            self.get_parameter("sector_half_angle_deg").value
        )
        self.sector_min_height = float(
            self.get_parameter("sector_min_height").value
        )
        self.sector_max_height = float(
            self.get_parameter("sector_max_height").value
        )
        self.print_interval_sec = float(
            self.get_parameter("print_interval_sec").value
        )
        self.window_name = str(self.get_parameter("window_name").value)
        self.draw_sector_nearest_markers = bool(
            self.get_parameter("draw_sector_nearest_markers").value
        )
        self.enable_click_coordinate_probe = bool(
            self.get_parameter("enable_click_coordinate_probe").value
        )
        self.click_probe_radius_m = float(
            self.get_parameter("click_probe_radius_m").value
        )
        self.click_probe_min_points = int(
            self.get_parameter("click_probe_min_points").value
        )

        # ------------------------------------------------------------
        # State
        # ------------------------------------------------------------
        self.lock = threading.Lock()
        self.latest_body_points: Optional[np.ndarray] = None
        self.latest_cloud_receive_time = 0.0
        self.latest_cloud_header_time = 0.0
        self.latest_cloud_frame = ""
        self.latest_cloud_count = 0

        self.latest_imu_receive_time = 0.0
        self.latest_imu_stamp = 0.0
        self.latest_roll = 0.0
        self.latest_pitch = 0.0
        self.latest_yaw = 0.0
        self.filtered_roll: Optional[float] = None
        self.filtered_pitch: Optional[float] = None
        self.filtered_yaw: Optional[float] = None

        self.last_print_time = 0.0
        self.running = True

        # Interactive coordinate probe state
        self.latest_raw_points_for_probe = None
        self.latest_aligned_points_for_probe = None
        self.latest_probe_result = None
        self.latest_probe_panel = None
        self.latest_dashboard_shape = None

        # ------------------------------------------------------------
        # QoS
        # ------------------------------------------------------------
        lidar_qos = QoSProfile(depth=1)
        lidar_qos.reliability = ReliabilityPolicy.RELIABLE
        lidar_qos.durability = DurabilityPolicy.VOLATILE

        imu_qos = QoSProfile(depth=100)
        imu_qos.reliability = ReliabilityPolicy.RELIABLE
        imu_qos.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.lidar_callback,
            lidar_qos,
        )
        self.create_subscription(
            SportModeState,
            self.sportmode_topic,
            self.sportmode_callback,
            imu_qos,
        )

        display_period = 1.0 / max(1.0, self.display_fps)
        self.create_timer(display_period, self.display_callback)
        self.mouse_callback_initialized = False

        self.get_logger().info(
            "V4.1.1 coordinate validation node started."
        )
        self.get_logger().info(
            f"LiDAR topic={self.lidar_topic}, "
            f"SportMode topic={self.sportmode_topic}"
        )
        self.get_logger().info(
            "Fixed LiDAR extrinsic "
            f"xyz={self.lidar_t.tolist()}, "
            f"rpy_deg={self.lidar_rpy_deg}"
        )
        self.get_logger().info(
            "Gravity alignment removes body roll/pitch while preserving yaw."
        )

    # ============================================================
    # ROS callbacks
    # ============================================================
    def sportmode_callback(self, msg: SportModeState):
        now = self.local_time_sec()

        roll = float(msg.imu_state.rpy[0])
        pitch = float(msg.imu_state.rpy[1])
        yaw = float(msg.imu_state.rpy[2])

        alpha = max(0.01, min(1.0, self.imu_lowpass_alpha))

        with self.lock:
            self.latest_imu_receive_time = now
            self.latest_imu_stamp = (
                float(msg.stamp.sec)
                + float(msg.stamp.nanosec) * 1e-9
            )
            self.latest_roll = roll
            self.latest_pitch = pitch
            self.latest_yaw = yaw

            if self.filtered_roll is None:
                self.filtered_roll = roll
                self.filtered_pitch = pitch
                self.filtered_yaw = yaw
            else:
                self.filtered_roll = self.angle_lowpass(
                    self.filtered_roll,
                    roll,
                    alpha,
                )
                self.filtered_pitch = self.angle_lowpass(
                    self.filtered_pitch,
                    pitch,
                    alpha,
                )
                self.filtered_yaw = self.angle_lowpass(
                    self.filtered_yaw,
                    yaw,
                    alpha,
                )

    def lidar_callback(self, msg: PointCloud2):
        receive_time = self.local_time_sec()

        try:
            points_lidar = self.pointcloud2_to_xyz(msg)
        except Exception as exc:
            self.get_logger().error(
                f"PointCloud2 parsing failed: {exc}"
            )
            return

        if len(points_lidar) == 0:
            return

        valid = np.isfinite(points_lidar).all(axis=1)
        points_lidar = points_lidar[valid]
        if len(points_lidar) == 0:
            return

        points_body = (
            points_lidar @ self.R_body_lidar.T
            + self.lidar_t.reshape(1, 3)
        )

        points_body = self.filter_points(points_body)
        points_body = self.voxel_downsample(points_body, self.voxel_size)

        if len(points_body) > self.max_display_points:
            stride = int(
                math.ceil(len(points_body) / self.max_display_points)
            )
            points_body = points_body[::stride]

        header_time = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )

        with self.lock:
            self.latest_body_points = points_body.astype(
                np.float32,
                copy=False,
            )
            self.latest_cloud_receive_time = receive_time
            self.latest_cloud_header_time = header_time
            self.latest_cloud_frame = msg.header.frame_id
            self.latest_cloud_count = int(msg.width * msg.height)

    # ============================================================
    # PointCloud2 and transforms
    # ============================================================
    @staticmethod
    def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
        field_offsets: Dict[str, int] = {
            field.name: int(field.offset)
            for field in msg.fields
        }

        for name in ("x", "y", "z"):
            if name not in field_offsets:
                raise ValueError(f"Missing PointCloud2 field: {name}")

        point_count = int(msg.width * msg.height)
        point_step = int(msg.point_step)

        if point_count <= 0 or point_step <= 0:
            return np.empty((0, 3), dtype=np.float32)

        raw = memoryview(msg.data)
        points = np.empty((point_count, 3), dtype=np.float32)

        endian = ">" if msg.is_bigendian else "<"
        unpack_format = endian + "f"

        x_offset = field_offsets["x"]
        y_offset = field_offsets["y"]
        z_offset = field_offsets["z"]

        for index in range(point_count):
            base = index * point_step
            points[index, 0] = struct.unpack_from(
                unpack_format,
                raw,
                base + x_offset,
            )[0]
            points[index, 1] = struct.unpack_from(
                unpack_format,
                raw,
                base + y_offset,
            )[0]
            points[index, 2] = struct.unpack_from(
                unpack_format,
                raw,
                base + z_offset,
            )[0]

        return points

    def filter_points(self, points: np.ndarray) -> np.ndarray:
        if len(points) == 0:
            return points

        distances = np.linalg.norm(points[:, :2], axis=1)

        mask = (
            (distances >= self.min_range)
            & (distances <= self.max_range)
            & (points[:, 2] >= self.min_z)
            & (points[:, 2] <= self.max_z)
        )

        x_min, x_max, y_min, y_max, z_min, z_max = self.self_bounds
        self_mask = (
            (points[:, 0] >= x_min)
            & (points[:, 0] <= x_max)
            & (points[:, 1] >= y_min)
            & (points[:, 1] <= y_max)
            & (points[:, 2] >= z_min)
            & (points[:, 2] <= z_max)
        )

        return points[mask & (~self_mask)]

    @staticmethod
    def voxel_downsample(
        points: np.ndarray,
        voxel_size: float,
    ) -> np.ndarray:
        if len(points) == 0 or voxel_size <= 0.0:
            return points

        keys = np.floor(points / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(
            keys,
            axis=0,
            return_index=True,
        )
        unique_indices.sort()
        return points[unique_indices]

    @staticmethod
    def rotation_matrix_rpy(
        roll: float,
        pitch: float,
        yaw: float,
    ) -> np.ndarray:
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        rx = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, cr, -sr],
                [0.0, sr, cr],
            ],
            dtype=np.float32,
        )
        ry = np.array(
            [
                [cp, 0.0, sp],
                [0.0, 1.0, 0.0],
                [-sp, 0.0, cp],
            ],
            dtype=np.float32,
        )
        rz = np.array(
            [
                [cy, -sy, 0.0],
                [sy, cy, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

        return rz @ ry @ rx

    def gravity_align_points(
        self,
        body_points: np.ndarray,
        roll: float,
        pitch: float,
    ) -> np.ndarray:
        if not self.enable_gravity_alignment:
            return body_points.copy()

        corrected_roll = (
            self.imu_roll_sign * roll
            + self.imu_roll_offset
        )
        corrected_pitch = (
            self.imu_pitch_sign * pitch
            + self.imu_pitch_offset
        )

        # Body -> yaw-only stabilized frame:
        # p_stabilized = Ry(pitch) * Rx(roll) * p_body
        rotation = (
            self.rotation_matrix_rpy(
                0.0,
                corrected_pitch,
                0.0,
            )
            @ self.rotation_matrix_rpy(
                corrected_roll,
                0.0,
                0.0,
            )
        )

        return body_points @ rotation.T

    # ============================================================
    # Ground plane and obstacle diagnostics
    # ============================================================
    def estimate_ground_plane(
        self,
        points: np.ndarray,
    ) -> Tuple[Optional[np.ndarray], float, int]:
        if points is None or len(points) < 30:
            return None, 0.0, 0

        distance = np.linalg.norm(points[:, :2], axis=1)
        mask = (
            (distance >= self.ground_fit_min_range)
            & (distance <= self.ground_fit_max_range)
            & (points[:, 2] >= self.ground_fit_z_min)
            & (points[:, 2] <= self.ground_fit_z_max)
        )
        candidates = points[mask]

        if len(candidates) < 30:
            return None, 0.0, 0

        if len(candidates) > self.ground_fit_max_points:
            stride = int(
                math.ceil(
                    len(candidates) / self.ground_fit_max_points
                )
            )
            candidates = candidates[::stride]

        # Start with the lower portion of the local cloud.
        z_threshold = np.percentile(candidates[:, 2], 45.0)
        working = candidates[
            candidates[:, 2] <= z_threshold
        ]

        if len(working) < 30:
            working = candidates

        normal = None
        d = 0.0

        for _ in range(max(1, self.ground_refine_iterations)):
            centroid = np.mean(working, axis=0)
            centered = working - centroid

            try:
                _, _, vh = np.linalg.svd(
                    centered,
                    full_matrices=False,
                )
            except np.linalg.LinAlgError:
                return None, 0.0, 0

            normal = vh[-1].astype(np.float32)
            norm = float(np.linalg.norm(normal))
            if norm < 1e-8:
                return None, 0.0, 0

            normal /= norm
            if normal[2] < 0.0:
                normal = -normal

            d = -float(np.dot(normal, centroid))
            residual = np.abs(candidates @ normal + d)
            inlier_mask = residual <= self.ground_inlier_threshold
            refined = candidates[inlier_mask]

            if len(refined) < 30:
                break

            working = refined

        if normal is None:
            return None, 0.0, 0

        residual = np.abs(candidates @ normal + d)
        inlier_count = int(
            np.count_nonzero(
                residual <= self.ground_inlier_threshold
            )
        )

        return normal, d, inlier_count

    def obstacle_points_above_ground(
        self,
        points: np.ndarray,
        normal: Optional[np.ndarray],
        d: float,
    ) -> np.ndarray:
        if points is None or len(points) == 0:
            return np.empty((0, 3), dtype=np.float32)

        if normal is None:
            return points[
                (points[:, 2] >= self.sector_min_height)
                & (points[:, 2] <= self.sector_max_height)
            ]

        signed_distance = points @ normal + d

        mask = (
            (signed_distance >= self.obstacle_ground_clearance)
            & (
                signed_distance
                <= self.sector_max_height
            )
        )
        return points[mask]

    def compute_sector_obstacles(
        self,
        obstacle_points: np.ndarray,
    ) -> Dict[str, Optional[Tuple[float, float, float]]]:
        result: Dict[str, Optional[Tuple[float, float, float]]] = {
            "FRONT": None,
            "LEFT": None,
            "RIGHT": None,
            "REAR": None,
        }

        if obstacle_points is None or len(obstacle_points) == 0:
            return result

        x = obstacle_points[:, 0]
        y = obstacle_points[:, 1]
        distance = np.hypot(x, y)
        angle = np.degrees(np.arctan2(y, x))

        sector_centers = {
            "FRONT": 0.0,
            "LEFT": 90.0,
            "RIGHT": -90.0,
            "REAR": 180.0,
        }

        for name, center in sector_centers.items():
            delta = self.wrap_degrees(angle - center)
            mask = np.abs(delta) <= self.sector_half_angle_deg

            if not np.any(mask):
                continue

            indices = np.where(mask)[0]
            local_distance = distance[indices]
            nearest_local_index = int(np.argmin(local_distance))
            point_index = int(indices[nearest_local_index])

            result[name] = (
                float(x[point_index]),
                float(y[point_index]),
                float(distance[point_index]),
            )

        return result

    # ============================================================
    # Visualization
    # ============================================================
    def display_callback(self):
        now = self.local_time_sec()

        with self.lock:
            if self.latest_body_points is None:
                return

            body_points = self.latest_body_points.copy()
            cloud_receive_time = self.latest_cloud_receive_time
            cloud_header_time = self.latest_cloud_header_time
            cloud_frame = self.latest_cloud_frame
            cloud_count = self.latest_cloud_count

            imu_receive_time = self.latest_imu_receive_time
            imu_stamp = self.latest_imu_stamp
            roll = (
                self.filtered_roll
                if self.filtered_roll is not None
                else self.latest_roll
            )
            pitch = (
                self.filtered_pitch
                if self.filtered_pitch is not None
                else self.latest_pitch
            )
            yaw = (
                self.filtered_yaw
                if self.filtered_yaw is not None
                else self.latest_yaw
            )

        imu_age = (
            now - imu_receive_time
            if imu_receive_time > 0.0
            else float("inf")
        )
        cloud_age = (
            now - cloud_receive_time
            if cloud_receive_time > 0.0
            else float("inf")
        )
        imu_valid = imu_age <= self.imu_timeout_sec

        stabilized_points = (
            self.gravity_align_points(
                body_points,
                roll,
                pitch,
            )
            if imu_valid
            else body_points.copy()
        )

        raw_normal, raw_d, raw_inliers = self.estimate_ground_plane(
            body_points
        )
        stabilized_normal, stabilized_d, stabilized_inliers = (
            self.estimate_ground_plane(stabilized_points)
        )

        raw_obstacles = self.obstacle_points_above_ground(
            body_points,
            raw_normal,
            raw_d,
        )
        stabilized_obstacles = self.obstacle_points_above_ground(
            stabilized_points,
            stabilized_normal,
            stabilized_d,
        )

        raw_sectors = self.compute_sector_obstacles(raw_obstacles)
        stabilized_sectors = self.compute_sector_obstacles(
            stabilized_obstacles
        )

        left = self.draw_bev_panel(
            body_points,
            raw_obstacles,
            raw_normal,
            raw_d,
            raw_sectors,
            title="RAW BODY-FRAME BEV",
            subtitle="Fixed LiDAR extrinsic only",
        )
        right = self.draw_bev_panel(
            stabilized_points,
            stabilized_obstacles,
            stabilized_normal,
            stabilized_d,
            stabilized_sectors,
            title="GRAVITY-ALIGNED BEV",
            subtitle=(
                "Go2 roll/pitch compensation"
                if imu_valid
                else "IMU TIMEOUT - alignment bypassed"
            ),
        )

        dashboard = np.hstack([left, right])
        with self.lock:
            self.latest_raw_points_for_probe = body_points.copy()
            self.latest_aligned_points_for_probe = stabilized_points.copy()
            self.latest_dashboard_shape = dashboard.shape

        self.draw_header(
            dashboard,
            roll,
            pitch,
            yaw,
            imu_age,
            cloud_age,
            imu_valid,
            cloud_frame,
            cloud_count,
            cloud_header_time,
            imu_stamp,
            raw_normal,
            stabilized_normal,
            raw_inliers,
            stabilized_inliers,
        )

        self.draw_probe_overlay(dashboard)

        cv2.imshow(self.window_name, dashboard)

        if (
            self.enable_click_coordinate_probe
            and not self.mouse_callback_initialized
        ):
            cv2.setMouseCallback(
                self.window_name,
                self.mouse_callback,
            )
            self.mouse_callback_initialized = True

        key = cv2.waitKey(1) & 0xFF

        if key in (27, ord("q")):
            self.running = False
            rclpy.shutdown()

        if now - self.last_print_time >= self.print_interval_sec:
            self.last_print_time = now
            self.print_diagnostics(
                roll,
                pitch,
                yaw,
                imu_age,
                cloud_age,
                imu_valid,
                raw_normal,
                stabilized_normal,
                raw_sectors,
                stabilized_sectors,
            )

    def draw_bev_panel(
        self,
        points: np.ndarray,
        obstacle_points: np.ndarray,
        normal: Optional[np.ndarray],
        d: float,
        sectors: Dict[str, Optional[Tuple[float, float, float]]],
        title: str,
        subtitle: str,
    ) -> np.ndarray:
        size = self.panel_size
        canvas = np.full(
            (size, size, 3),
            245,
            dtype=np.uint8,
        )

        origin_x = size // 2
        origin_y = size // 2
        pixels_per_meter = (
            (size * 0.44) / max(0.5, self.display_range_m)
        )

        self.draw_grid(
            canvas,
            origin_x,
            origin_y,
            pixels_per_meter,
        )

        if points is not None and len(points) > 0:
            inside = (
                (np.abs(points[:, 0]) <= self.display_range_m)
                & (np.abs(points[:, 1]) <= self.display_range_m)
            )
            draw_points = points[inside]

            if len(draw_points) > 0:
                px = (
                    origin_x
                    - draw_points[:, 1] * pixels_per_meter
                ).astype(np.int32)
                py = (
                    origin_y
                    - draw_points[:, 0] * pixels_per_meter
                ).astype(np.int32)

                z = draw_points[:, 2]
                colors = self.height_colors(z)

                valid = (
                    (px >= 0)
                    & (px < size)
                    & (py >= 0)
                    & (py < size)
                )

                canvas[
                    py[valid],
                    px[valid],
                ] = colors[valid]

        if obstacle_points is not None and len(obstacle_points) > 0:
            inside = (
                (np.abs(obstacle_points[:, 0]) <= self.display_range_m)
                & (np.abs(obstacle_points[:, 1]) <= self.display_range_m)
            )
            obstacle_draw = obstacle_points[inside]

            if len(obstacle_draw) > 0:
                px = (
                    origin_x
                    - obstacle_draw[:, 1] * pixels_per_meter
                ).astype(np.int32)
                py = (
                    origin_y
                    - obstacle_draw[:, 0] * pixels_per_meter
                ).astype(np.int32)
                valid = (
                    (px >= 0)
                    & (px < size)
                    & (py >= 0)
                    & (py < size)
                )
                canvas[py[valid], px[valid]] = (0, 0, 220)

        self.draw_robot(
            canvas,
            origin_x,
            origin_y,
            pixels_per_meter,
        )
        self.draw_sector_markers(
            canvas,
            sectors,
            origin_x,
            origin_y,
            pixels_per_meter,
        )

        cv2.putText(
            canvas,
            title,
            (18, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (20, 20, 20),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            subtitle,
            (18, 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (70, 70, 70),
            1,
            cv2.LINE_AA,
        )

        normal_text = self.normal_text(normal)
        cv2.putText(
            canvas,
            normal_text,
            (18, size - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (30, 30, 30),
            1,
            cv2.LINE_AA,
        )

        return canvas

    def draw_grid(
        self,
        canvas: np.ndarray,
        origin_x: int,
        origin_y: int,
        pixels_per_meter: float,
    ):
        size = canvas.shape[0]

        for meter in range(
            1,
            int(math.floor(self.display_range_m)) + 1,
        ):
            radius = int(round(meter * pixels_per_meter))
            cv2.circle(
                canvas,
                (origin_x, origin_y),
                radius,
                (215, 215, 215),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                canvas,
                f"{meter}m",
                (origin_x + 4, origin_y - radius + 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                (140, 140, 140),
                1,
                cv2.LINE_AA,
            )

        cv2.line(
            canvas,
            (origin_x, 0),
            (origin_x, size - 1),
            (195, 195, 195),
            1,
        )
        cv2.line(
            canvas,
            (0, origin_y),
            (size - 1, origin_y),
            (195, 195, 195),
            1,
        )

        cv2.putText(
            canvas,
            "FRONT +X",
            (origin_x - 42, 74),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (70, 70, 70),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            "LEFT +Y",
            (12, origin_y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (70, 70, 70),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            "RIGHT -Y",
            (size - 82, origin_y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (70, 70, 70),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            "REAR -X",
            (origin_x - 38, size - 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (70, 70, 70),
            1,
            cv2.LINE_AA,
        )

    @staticmethod
    def draw_robot(
        canvas: np.ndarray,
        origin_x: int,
        origin_y: int,
        pixels_per_meter: float,
    ):
        length = max(20, int(round(0.75 * pixels_per_meter)))
        width = max(14, int(round(0.42 * pixels_per_meter)))

        x1 = origin_x - width // 2
        x2 = origin_x + width // 2
        y1 = origin_y - length // 2
        y2 = origin_y + length // 2

        cv2.rectangle(
            canvas,
            (x1, y1),
            (x2, y2),
            (40, 40, 40),
            2,
        )
        cv2.arrowedLine(
            canvas,
            (origin_x, origin_y),
            (origin_x, y1 - 24),
            (0, 90, 220),
            3,
            cv2.LINE_AA,
            tipLength=0.28,
        )

    def draw_sector_markers(
        self,
        canvas: np.ndarray,
        sectors: Dict[str, Optional[Tuple[float, float, float]]],
        origin_x: int,
        origin_y: int,
        pixels_per_meter: float,
    ):
        label_positions = {
            "FRONT": (18, 82),
            "LEFT": (18, 104),
            "RIGHT": (18, 126),
            "REAR": (18, 148),
        }

        for name in ("FRONT", "LEFT", "RIGHT", "REAR"):
            value = sectors.get(name)
            y_text = label_positions[name][1]

            if value is None:
                text = f"{name:<5}: no obstacle"
                color = (130, 130, 130)
            else:
                x, y, distance = value
                text = (
                    f"{name:<5}: d={distance:.2f}m "
                    f"x={x:+.2f} y={y:+.2f}"
                )
                color = (0, 0, 190)

                px = int(round(origin_x - y * pixels_per_meter))
                py = int(round(origin_y - x * pixels_per_meter))

                if (
                    self.draw_sector_nearest_markers
                    and 0 <= px < canvas.shape[1]
                    and 0 <= py < canvas.shape[0]
                ):
                    cv2.drawMarker(
                        canvas,
                        (px, py),
                        (0, 0, 255),
                        markerType=cv2.MARKER_CROSS,
                        markerSize=10,
                        thickness=1,
                        line_type=cv2.LINE_AA,
                    )

            cv2.putText(
                canvas,
                text,
                (label_positions[name][0], y_text),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                color,
                1,
                cv2.LINE_AA,
            )


    # ============================================================
    # Interactive coordinate probe
    # ============================================================
    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if y < 84:
            return

        panel_width = self.panel_size
        if x < panel_width:
            panel = "RAW"
            local_x = x
            points = self.latest_raw_points_for_probe
        elif x < panel_width * 2:
            panel = "ALIGNED"
            local_x = x - panel_width
            points = self.latest_aligned_points_for_probe
        else:
            return

        if points is None or len(points) == 0:
            return

        origin_x = self.panel_size // 2
        origin_y = self.panel_size // 2
        pixels_per_meter = (
            (self.panel_size * 0.44)
            / max(0.5, self.display_range_m)
        )

        clicked_y = -(
            float(local_x) - float(origin_x)
        ) / pixels_per_meter
        clicked_x = -(
            float(y) - float(origin_y)
        ) / pixels_per_meter

        delta = points[:, :2] - np.array(
            [clicked_x, clicked_y],
            dtype=np.float32,
        )
        distance = np.linalg.norm(delta, axis=1)
        nearby = points[distance <= self.click_probe_radius_m]

        if len(nearby) < self.click_probe_min_points:
            self.latest_probe_result = {
                "panel": panel,
                "clicked_x": clicked_x,
                "clicked_y": clicked_y,
                "valid": False,
                "count": int(len(nearby)),
            }
            return

        median = np.median(nearby, axis=0)
        minimum = np.min(nearby, axis=0)
        maximum = np.max(nearby, axis=0)

        self.latest_probe_result = {
            "panel": panel,
            "clicked_x": clicked_x,
            "clicked_y": clicked_y,
            "valid": True,
            "count": int(len(nearby)),
            "median": tuple(float(v) for v in median),
            "min": tuple(float(v) for v in minimum),
            "max": tuple(float(v) for v in maximum),
        }

        self.get_logger().info(
            f"Coordinate probe {panel} | "
            f"x={median[0]:+.3f}m, "
            f"y={median[1]:+.3f}m, "
            f"z={median[2]:+.3f}m, "
            f"points={len(nearby)}, "
            f"extent=({maximum[0]-minimum[0]:.2f}, "
            f"{maximum[1]-minimum[1]:.2f}, "
            f"{maximum[2]-minimum[2]:.2f})m"
        )

    def draw_probe_overlay(self, dashboard):
        result = self.latest_probe_result
        if result is None:
            text = (
                "Click a point-cloud cluster to inspect its local "
                "x/y/z coordinate."
            )
            cv2.putText(
                dashboard,
                text,
                (16, dashboard.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (70, 70, 70),
                1,
                cv2.LINE_AA,
            )
            return

        if not result.get("valid", False):
            text = (
                f"Probe {result['panel']}: no cluster near click "
                f"(points={result['count']})"
            )
            color = (0, 0, 180)
        else:
            mx, my, mz = result["median"]
            text = (
                f"Probe {result['panel']}: "
                f"x={mx:+.2f}m y={my:+.2f}m z={mz:+.2f}m "
                f"points={result['count']}"
            )
            color = (0, 120, 0)

        cv2.rectangle(
            dashboard,
            (10, dashboard.shape[0] - 38),
            (dashboard.shape[1] - 10, dashboard.shape[0] - 4),
            (245, 245, 245),
            -1,
        )
        cv2.putText(
            dashboard,
            text,
            (16, dashboard.shape[0] - 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            color,
            1,
            cv2.LINE_AA,
        )

    def draw_header(
        self,
        dashboard: np.ndarray,
        roll: float,
        pitch: float,
        yaw: float,
        imu_age: float,
        cloud_age: float,
        imu_valid: bool,
        cloud_frame: str,
        cloud_count: int,
        cloud_header_time: float,
        imu_stamp: float,
        raw_normal: Optional[np.ndarray],
        stabilized_normal: Optional[np.ndarray],
        raw_inliers: int,
        stabilized_inliers: int,
    ):
        overlay_height = 84
        overlay = dashboard[:overlay_height].copy()
        overlay[:] = (250, 250, 250)
        dashboard[:overlay_height] = overlay

        status_color = (
            (0, 150, 0)
            if imu_valid
            else (0, 0, 220)
        )

        line1 = (
            "V4.1.1 Coordinate Validation | "
            f"roll={math.degrees(roll):+.2f} deg "
            f"pitch={math.degrees(pitch):+.2f} deg "
            f"yaw={math.degrees(yaw):+.2f} deg"
        )
        line2 = (
            f"IMU {'OK' if imu_valid else 'TIMEOUT'} "
            f"age={imu_age * 1000.0:.1f}ms | "
            f"cloud age={cloud_age * 1000.0:.1f}ms | "
            f"frame={cloud_frame} | raw_points={cloud_count}"
        )
        line3 = (
            f"ground raw={self.normal_short(raw_normal)} "
            f"inliers={raw_inliers} | "
            f"aligned={self.normal_short(stabilized_normal)} "
            f"inliers={stabilized_inliers}"
        )

        cv2.putText(
            dashboard,
            line1,
            (16, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (20, 20, 20),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            dashboard,
            line2,
            (16, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            status_color,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            dashboard,
            line3,
            (16, 72),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (50, 50, 50),
            1,
            cv2.LINE_AA,
        )

    @staticmethod
    def height_colors(z: np.ndarray) -> np.ndarray:
        normalized = np.clip(
            (z + 0.5) / 2.5,
            0.0,
            1.0,
        )
        blue = (255.0 * (1.0 - normalized)).astype(np.uint8)
        red = (255.0 * normalized).astype(np.uint8)
        green = (
            170.0
            * (1.0 - np.abs(normalized - 0.5) * 2.0)
        ).astype(np.uint8)

        return np.stack([blue, green, red], axis=1)

    @staticmethod
    def normal_text(normal: Optional[np.ndarray]) -> str:
        if normal is None:
            return "ground normal: unavailable"

        tilt = math.degrees(
            math.acos(
                max(-1.0, min(1.0, float(normal[2])))
            )
        )
        return (
            "ground normal: "
            f"[{normal[0]:+.3f}, {normal[1]:+.3f}, "
            f"{normal[2]:+.3f}] tilt={tilt:.2f}deg"
        )

    @staticmethod
    def normal_short(normal: Optional[np.ndarray]) -> str:
        if normal is None:
            return "N/A"
        return (
            f"[{normal[0]:+.2f},"
            f"{normal[1]:+.2f},"
            f"{normal[2]:+.2f}]"
        )

    # ============================================================
    # Logs and utilities
    # ============================================================
    def print_diagnostics(
        self,
        roll: float,
        pitch: float,
        yaw: float,
        imu_age: float,
        cloud_age: float,
        imu_valid: bool,
        raw_normal: Optional[np.ndarray],
        stabilized_normal: Optional[np.ndarray],
        raw_sectors: Dict[str, Optional[Tuple[float, float, float]]],
        stabilized_sectors: Dict[str, Optional[Tuple[float, float, float]]],
    ):
        self.get_logger().info(
            "V4.1 | "
            f"imu={'OK' if imu_valid else 'TIMEOUT'} "
            f"age={imu_age * 1000.0:.1f}ms | "
            f"cloud_age={cloud_age * 1000.0:.1f}ms | "
            f"rpy_deg=({math.degrees(roll):+.2f}, "
            f"{math.degrees(pitch):+.2f}, "
            f"{math.degrees(yaw):+.2f}) | "
            f"raw_n={self.normal_short(raw_normal)} | "
            f"aligned_n={self.normal_short(stabilized_normal)}"
        )

        for name in ("FRONT", "LEFT", "RIGHT", "REAR"):
            raw_text = self.sector_value_text(raw_sectors.get(name))
            aligned_text = self.sector_value_text(
                stabilized_sectors.get(name)
            )
            self.get_logger().info(
                f"  {name:<5} raw={raw_text} "
                f"aligned={aligned_text}"
            )

    @staticmethod
    def sector_value_text(
        value: Optional[Tuple[float, float, float]]
    ) -> str:
        if value is None:
            return "none"

        x, y, distance = value
        return (
            f"d={distance:.2f}m "
            f"x={x:+.2f} y={y:+.2f}"
        )

    @staticmethod
    def angle_lowpass(
        previous: float,
        current: float,
        alpha: float,
    ) -> float:
        delta = math.atan2(
            math.sin(current - previous),
            math.cos(current - previous),
        )
        return previous + alpha * delta

    @staticmethod
    def wrap_degrees(angle: np.ndarray) -> np.ndarray:
        return (angle + 180.0) % 360.0 - 180.0

    def local_time_sec(self) -> float:
        return (
            self.get_clock().now().nanoseconds
            * 1e-9
        )

    def destroy_node(self):
        self.running = False
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiDARVisionTopViewV411Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()

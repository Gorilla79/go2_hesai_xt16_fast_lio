#!/usr/bin/env python3

import math
import signal
import threading
import time
from typing import Optional, Dict, Any, List, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class LiDARVisionTopViewV32Node(Node):
    """
    LiDAR_Vision lightweight top-view fusion node.

    Purpose:
        - RealSense D435i RGB image input
        - YOLO11 segmentation person detection
        - Hesai XT16 /lidar_points association
        - Robot-centered 2D top-view visualization
        - No LiDAR point drawing on camera image for speed
        - Low-latency LiDAR downsampling and ROI filtering

    Coordinate convention:
        base frame:
            x: forward
            y: left
            z: up

        camera body:
            x: forward
            y: left
            z: up

        RealSense optical:
            X: right
            Y: down
            Z: forward
    """

    def __init__(self):
        super().__init__("lidar_vision_topview_v3_2_node")

        self.running = True
        self.pipeline_started = False

        # ------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------
        self.declare_parameter("model", "yolo11s-seg.pt")
        self.declare_parameter("device", "0")
        self.declare_parameter("half", True)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("conf", 0.35)
        self.declare_parameter("iou", 0.55)
        self.declare_parameter("target_fps", 10.0)

        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 30)
        self.declare_parameter("image_rotate", "none")

        self.declare_parameter("lidar_topic", "/lidar_points")
        self.declare_parameter("lidar_points_frame", "lidar")  # lidar or base

        # Latest calibration result
        self.declare_parameter("camera_base_x", 0.340)
        self.declare_parameter("camera_base_y", 0.040)
        self.declare_parameter("camera_base_z", 0.390)
        self.declare_parameter("camera_roll_deg", 0.0)
        self.declare_parameter("camera_pitch_deg", 0.0)
        self.declare_parameter("camera_yaw_deg", -0.5)

        self.declare_parameter("lidar_base_x", 0.150)
        self.declare_parameter("lidar_base_y", 0.000)
        self.declare_parameter("lidar_base_z", 0.250)
        self.declare_parameter("lidar_roll_deg", -6.0)
        self.declare_parameter("lidar_pitch_deg", 0.0)
        self.declare_parameter("lidar_yaw_deg", 90.0)

        # LiDAR ROI after transformed into robot base frame
        self.declare_parameter("lidar_x_min", 0.20)
        self.declare_parameter("lidar_x_max", 8.00)
        self.declare_parameter("lidar_y_min", -4.00)
        self.declare_parameter("lidar_y_max", 4.00)
        self.declare_parameter("lidar_z_min", -1.20)
        self.declare_parameter("lidar_z_max", 2.50)

        # Performance parameters
        self.declare_parameter("max_lidar_points_process", 12000)
        self.declare_parameter("max_lidar_points_topview", 1800)
        self.declare_parameter("skip_lidar_projection_if_points_over", 20000)

        # Person association
        self.declare_parameter("min_lidar_points_in_mask", 3)
        self.declare_parameter("mask_dilate_px", 3)
        self.declare_parameter("person_outlier_radius", 0.45)
        self.declare_parameter("near_cluster_depth_m", 0.45)
        self.declare_parameter("fixed_distance", 2.0)

        # V2: LiDAR association confidence thresholds
        self.declare_parameter("confidence_high_count", 30)
        self.declare_parameter("confidence_medium_count", 10)
        self.declare_parameter("confidence_high_spread", 0.45)
        self.declare_parameter("confidence_medium_spread", 0.70)
        self.declare_parameter("confidence_high_depth_std", 0.25)
        self.declare_parameter("confidence_medium_depth_std", 0.50)

        # V3: tracking memory, smoothing, and short-term persistence
        self.declare_parameter("enable_tracking", True)
        self.declare_parameter("track_match_distance", 0.80)
        self.declare_parameter("smoothing_alpha", 0.35)
        self.declare_parameter("max_missing_frames", 5)
        self.declare_parameter("stale_track_color_b", 160)
        self.declare_parameter("stale_track_color_g", 160)
        self.declare_parameter("stale_track_color_r", 160)

        # V3.1: duplicate detection and duplicate track suppression
        self.declare_parameter("enable_duplicate_filter", True)
        self.declare_parameter("duplicate_detection_distance", 0.45)
        self.declare_parameter("duplicate_track_distance", 0.55)
        self.declare_parameter("duplicate_bbox_iou", 0.45)
        self.declare_parameter("stale_active_suppression_distance", 0.80)
        self.declare_parameter("stale_control_max_missing", 1)

        # V3.2: false-positive suppression and control-count filtering
        self.declare_parameter("enable_false_positive_filter", True)
        self.declare_parameter("exclude_fallback_from_control", True)
        self.declare_parameter("allow_low_confidence_close_control", True)
        self.declare_parameter("low_control_max_distance", 2.00)
        self.declare_parameter("low_control_min_hits", 3)
        self.declare_parameter("fallback_control_max_distance", 1.20)
        self.declare_parameter("fallback_control_min_hits", 6)
        self.declare_parameter("reflection_min_distance", 1.50)
        self.declare_parameter("min_control_bbox_height_px", 18)
        self.declare_parameter("min_control_bbox_area_px", 180)
        self.declare_parameter("far_distance_threshold", 3.00)
        self.declare_parameter("very_far_distance_threshold", 5.00)

        # Top-view
        self.declare_parameter("topview_size", 800)
        self.declare_parameter("meter_to_pixel", 90.0)
        self.declare_parameter("max_forward_m", 6.0)
        self.declare_parameter("max_side_m", 3.0)
        self.declare_parameter("draw_lidar_topview", True)
        self.declare_parameter("draw_camera_window", True)
        self.declare_parameter("draw_topview_window", True)
        self.declare_parameter("show_mask_overlay", True)

        # ------------------------------------------------------------
        # Read parameters
        # ------------------------------------------------------------
        self.model_path = self.get_parameter("model").value
        self.device = self.get_parameter("device").value
        self.use_half = bool(self.get_parameter("half").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        self.target_fps = float(self.get_parameter("target_fps").value)

        self.width = int(self.get_parameter("camera_width").value)
        self.height = int(self.get_parameter("camera_height").value)
        self.camera_fps = int(self.get_parameter("camera_fps").value)
        self.image_rotate = self.get_parameter("image_rotate").value

        self.lidar_topic = self.get_parameter("lidar_topic").value
        self.lidar_points_frame = self.get_parameter("lidar_points_frame").value

        self.camera_t_base = np.array([
            float(self.get_parameter("camera_base_x").value),
            float(self.get_parameter("camera_base_y").value),
            float(self.get_parameter("camera_base_z").value),
        ], dtype=np.float32)

        self.lidar_t_base = np.array([
            float(self.get_parameter("lidar_base_x").value),
            float(self.get_parameter("lidar_base_y").value),
            float(self.get_parameter("lidar_base_z").value),
        ], dtype=np.float32)

        self.camera_roll_deg = float(self.get_parameter("camera_roll_deg").value)
        self.camera_pitch_deg = float(self.get_parameter("camera_pitch_deg").value)
        self.camera_yaw_deg = float(self.get_parameter("camera_yaw_deg").value)

        self.lidar_roll_deg = float(self.get_parameter("lidar_roll_deg").value)
        self.lidar_pitch_deg = float(self.get_parameter("lidar_pitch_deg").value)
        self.lidar_yaw_deg = float(self.get_parameter("lidar_yaw_deg").value)

        self.R_base_camera_body = self.euler_to_rot(
            math.radians(self.camera_roll_deg),
            math.radians(self.camera_pitch_deg),
            math.radians(self.camera_yaw_deg),
        )

        self.R_base_lidar = self.euler_to_rot(
            math.radians(self.lidar_roll_deg),
            math.radians(self.lidar_pitch_deg),
            math.radians(self.lidar_yaw_deg),
        )

        self.lidar_x_min = float(self.get_parameter("lidar_x_min").value)
        self.lidar_x_max = float(self.get_parameter("lidar_x_max").value)
        self.lidar_y_min = float(self.get_parameter("lidar_y_min").value)
        self.lidar_y_max = float(self.get_parameter("lidar_y_max").value)
        self.lidar_z_min = float(self.get_parameter("lidar_z_min").value)
        self.lidar_z_max = float(self.get_parameter("lidar_z_max").value)

        self.max_lidar_points_process = int(self.get_parameter("max_lidar_points_process").value)
        self.max_lidar_points_topview = int(self.get_parameter("max_lidar_points_topview").value)
        self.skip_lidar_projection_if_points_over = int(self.get_parameter("skip_lidar_projection_if_points_over").value)

        self.min_lidar_points_in_mask = int(self.get_parameter("min_lidar_points_in_mask").value)
        self.mask_dilate_px = int(self.get_parameter("mask_dilate_px").value)
        self.person_outlier_radius = float(self.get_parameter("person_outlier_radius").value)
        self.near_cluster_depth_m = float(self.get_parameter("near_cluster_depth_m").value)
        self.fixed_distance = float(self.get_parameter("fixed_distance").value)

        self.confidence_high_count = int(self.get_parameter("confidence_high_count").value)
        self.confidence_medium_count = int(self.get_parameter("confidence_medium_count").value)
        self.confidence_high_spread = float(self.get_parameter("confidence_high_spread").value)
        self.confidence_medium_spread = float(self.get_parameter("confidence_medium_spread").value)
        self.confidence_high_depth_std = float(self.get_parameter("confidence_high_depth_std").value)
        self.confidence_medium_depth_std = float(self.get_parameter("confidence_medium_depth_std").value)

        self.enable_tracking = bool(self.get_parameter("enable_tracking").value)
        self.track_match_distance = float(self.get_parameter("track_match_distance").value)
        self.smoothing_alpha = float(self.get_parameter("smoothing_alpha").value)
        self.max_missing_frames = int(self.get_parameter("max_missing_frames").value)
        self.stale_track_color = (
            int(self.get_parameter("stale_track_color_b").value),
            int(self.get_parameter("stale_track_color_g").value),
            int(self.get_parameter("stale_track_color_r").value),
        )

        self.enable_duplicate_filter = bool(self.get_parameter("enable_duplicate_filter").value)
        self.duplicate_detection_distance = float(self.get_parameter("duplicate_detection_distance").value)
        self.duplicate_track_distance = float(self.get_parameter("duplicate_track_distance").value)
        self.duplicate_bbox_iou = float(self.get_parameter("duplicate_bbox_iou").value)
        self.stale_active_suppression_distance = float(
            self.get_parameter("stale_active_suppression_distance").value
        )
        self.stale_control_max_missing = int(self.get_parameter("stale_control_max_missing").value)

        self.enable_false_positive_filter = bool(self.get_parameter("enable_false_positive_filter").value)
        self.exclude_fallback_from_control = bool(self.get_parameter("exclude_fallback_from_control").value)
        self.allow_low_confidence_close_control = bool(
            self.get_parameter("allow_low_confidence_close_control").value
        )
        self.low_control_max_distance = float(self.get_parameter("low_control_max_distance").value)
        self.low_control_min_hits = int(self.get_parameter("low_control_min_hits").value)
        self.fallback_control_max_distance = float(self.get_parameter("fallback_control_max_distance").value)
        self.fallback_control_min_hits = int(self.get_parameter("fallback_control_min_hits").value)
        self.reflection_min_distance = float(self.get_parameter("reflection_min_distance").value)
        self.min_control_bbox_height_px = int(self.get_parameter("min_control_bbox_height_px").value)
        self.min_control_bbox_area_px = int(self.get_parameter("min_control_bbox_area_px").value)
        self.far_distance_threshold = float(self.get_parameter("far_distance_threshold").value)
        self.very_far_distance_threshold = float(self.get_parameter("very_far_distance_threshold").value)

        self.topview_size = int(self.get_parameter("topview_size").value)
        self.meter_to_pixel = float(self.get_parameter("meter_to_pixel").value)
        self.max_forward_m = float(self.get_parameter("max_forward_m").value)
        self.max_side_m = float(self.get_parameter("max_side_m").value)
        self.draw_lidar_topview = bool(self.get_parameter("draw_lidar_topview").value)
        self.draw_camera_window = bool(self.get_parameter("draw_camera_window").value)
        self.draw_topview_window = bool(self.get_parameter("draw_topview_window").value)
        self.show_mask_overlay = bool(self.get_parameter("show_mask_overlay").value)

        # ------------------------------------------------------------
        # Runtime state
        # ------------------------------------------------------------
        self.pipeline = None
        self.profile = None
        self.config = None

        self.raw_fx = None
        self.raw_fy = None
        self.raw_cx = None
        self.raw_cy = None
        self.raw_w = None
        self.raw_h = None

        self.latest_lidar_points_base = None
        self.latest_lidar_points_topview = None
        self.lidar_lock = threading.Lock()

        self.last_process_time = 0.0
        self.last_log_time = time.time()
        self.infer_times = []
        self.latest_persons = []

        # V3 tracking state
        self.tracks = []
        self.next_track_id = 0

        # ------------------------------------------------------------
        # ROS subscriber
        # ------------------------------------------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub_lidar = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.lidar_callback,
            qos,
        )

        # ------------------------------------------------------------
        # Safe exit
        # ------------------------------------------------------------
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        # ------------------------------------------------------------
        # Start model/camera/windows
        # ------------------------------------------------------------
        self.get_logger().info(f"Loading YOLO model once: {self.model_path}")
        self.model = YOLO(self.model_path)
        self.get_logger().info("YOLO model loaded.")

        self.start_realsense()

        self.create_windows()

        self.timer = self.create_timer(0.001, self.main_loop)

        self.get_logger().info("LiDAR_Vision top-view V3.2 node started.")
        self.get_logger().info("Camera image does NOT draw LiDAR projection points for speed.")
        self.get_logger().info("Top-view uses downsampled LiDAR points only.")
        self.get_logger().info(
            f"V3.2 tracking enable={self.enable_tracking}, "
            f"match_distance={self.track_match_distance:.2f}m, "
            f"smoothing_alpha={self.smoothing_alpha:.2f}, "
            f"max_missing_frames={self.max_missing_frames}"
        )
        self.get_logger().info(
            f"V3.2 duplicate filter enable={self.enable_duplicate_filter}, "
            f"det_dist={self.duplicate_detection_distance:.2f}m, "
            f"track_dist={self.duplicate_track_distance:.2f}m, "
            f"bbox_iou={self.duplicate_bbox_iou:.2f}, "
            f"stale_active_dist={self.stale_active_suppression_distance:.2f}m"
        )
        self.get_logger().info(
            f"V3.2 false-positive filter enable={self.enable_false_positive_filter}, "
            f"exclude_fallback={self.exclude_fallback_from_control}, "
            f"low_close={self.allow_low_confidence_close_control}, "
            f"low_max_dist={self.low_control_max_distance:.2f}m, "
            f"reflection_min_dist={self.reflection_min_distance:.2f}m"
        )
        self.get_logger().info(
            f"Calibration camera xyz={self.camera_t_base.tolist()}, "
            f"rpy=({self.camera_roll_deg}, {self.camera_pitch_deg}, {self.camera_yaw_deg})"
        )
        self.get_logger().info(
            f"Calibration lidar xyz={self.lidar_t_base.tolist()}, "
            f"rpy=({self.lidar_roll_deg}, {self.lidar_pitch_deg}, {self.lidar_yaw_deg})"
        )

    # ============================================================
    # Setup / shutdown
    # ============================================================
    def signal_handler(self, signum, frame):
        self.running = False

    def start_realsense(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(
            rs.stream.color,
            self.width,
            self.height,
            rs.format.bgr8,
            self.camera_fps,
        )

        self.get_logger().info("Starting RealSense color pipeline...")

        try:
            self.profile = self.pipeline.start(self.config)
            self.pipeline_started = True
        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense pipeline: {e}")
            self.get_logger().error("Check owner with: sudo fuser -v /dev/video*")
            raise

        color_stream = self.profile.get_stream(rs.stream.color)
        intr = color_stream.as_video_stream_profile().get_intrinsics()

        self.raw_fx = float(intr.fx)
        self.raw_fy = float(intr.fy)
        self.raw_cx = float(intr.ppx)
        self.raw_cy = float(intr.ppy)
        self.raw_w = int(intr.width)
        self.raw_h = int(intr.height)

        self.get_logger().info(
            f"RealSense intrinsics fx={self.raw_fx:.2f}, fy={self.raw_fy:.2f}, "
            f"cx={self.raw_cx:.2f}, cy={self.raw_cy:.2f}, w={self.raw_w}, h={self.raw_h}"
        )

        warmup_start = time.time()
        while time.time() - warmup_start < 0.5:
            self.pipeline.poll_for_frames()
            time.sleep(0.01)

    def create_windows(self):
        try:
            if self.draw_camera_window:
                cv2.namedWindow("LiDAR_Vision YOLO View", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("LiDAR_Vision YOLO View", 900, 680)
                cv2.moveWindow("LiDAR_Vision YOLO View", 20, 40)

            if self.draw_topview_window:
                cv2.namedWindow("LiDAR_Vision 2D Top View", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("LiDAR_Vision 2D Top View", self.topview_size, self.topview_size)
                cv2.moveWindow("LiDAR_Vision 2D Top View", 940, 40)

            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().warn(f"OpenCV window creation failed: {e}")
            self.draw_camera_window = False
            self.draw_topview_window = False

    def stop(self):
        self.running = False

        if self.pipeline_started:
            self.get_logger().info("Stopping RealSense pipeline...")
            self.pipeline_started = False
            try:
                self.pipeline.stop()
            except Exception:
                pass

        try:
            cv2.destroyAllWindows()
            for _ in range(5):
                cv2.waitKey(1)
        except Exception:
            pass

    # ============================================================
    # Main loop
    # ============================================================
    def main_loop(self):
        if not self.running:
            return

        now = time.time()
        min_interval = 1.0 / max(self.target_fps, 0.1)

        if now - self.last_process_time < min_interval:
            self.handle_keyboard()
            return

        self.last_process_time = now

        raw_frame = self.get_realsense_color_nonblocking()
        if raw_frame is None:
            self.handle_keyboard()
            return

        frame = self.rotate_image(raw_frame)

        lidar_points_base, lidar_points_topview = self.get_latest_lidar_points_copy()

        detections, infer_ms = self.run_yolo_lidar(frame, lidar_points_base)

        if self.enable_duplicate_filter:
            detections = self.suppress_duplicate_detections(detections)

        if self.enable_tracking:
            persons = self.update_tracks(detections)
        else:
            persons = detections

        if self.enable_duplicate_filter:
            persons = self.suppress_duplicate_tracks(persons)

        self.latest_persons = persons

        if infer_ms is not None:
            self.infer_times.append(infer_ms)
            if len(self.infer_times) > 30:
                self.infer_times.pop(0)

        if self.draw_camera_window:
            debug = self.draw_yolo_view(frame, persons)
            cv2.imshow("LiDAR_Vision YOLO View", debug)

        if self.draw_topview_window:
            top = self.draw_top_view(persons, lidar_points_topview)
            cv2.imshow("LiDAR_Vision 2D Top View", top)

        self.handle_keyboard()
        self.print_status(lidar_points_base, persons)

    def handle_keyboard(self):
        try:
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                self.get_logger().info("Quit key pressed.")
                self.running = False
        except Exception:
            pass

    def get_realsense_color_nonblocking(self):
        if not self.pipeline_started:
            return None

        try:
            frames = self.pipeline.poll_for_frames()
            if not frames:
                return None

            color_frame = frames.get_color_frame()
            if not color_frame:
                return None

            return np.asanyarray(color_frame.get_data())

        except Exception as e:
            self.get_logger().warn(f"RealSense polling error: {e}")
            return None

    # ============================================================
    # LiDAR callback
    # ============================================================
    def lidar_callback(self, msg):
        raw_points = self.pointcloud2_to_xyz_numpy(msg)
        if raw_points is None or len(raw_points) == 0:
            return

        base_points = self.lidar_raw_to_base(raw_points)
        base_points = self.apply_lidar_roi(base_points)

        if base_points is None or len(base_points) == 0:
            return

        if len(base_points) > self.max_lidar_points_process:
            idx = np.random.choice(len(base_points), self.max_lidar_points_process, replace=False)
            process_points = base_points[idx]
        else:
            process_points = base_points

        if len(base_points) > self.max_lidar_points_topview:
            idx = np.random.choice(len(base_points), self.max_lidar_points_topview, replace=False)
            topview_points = base_points[idx]
        else:
            topview_points = base_points

        with self.lidar_lock:
            self.latest_lidar_points_base = process_points
            self.latest_lidar_points_topview = topview_points

    def pointcloud2_to_xyz_numpy(self, msg):
        field_map = {field.name: field for field in msg.fields}

        if "x" not in field_map or "y" not in field_map or "z" not in field_map:
            self.get_logger().warn("PointCloud2 has no x/y/z fields.")
            return None

        try:
            dtype = np.dtype({
                "names": ["x", "y", "z"],
                "formats": [np.float32, np.float32, np.float32],
                "offsets": [
                    field_map["x"].offset,
                    field_map["y"].offset,
                    field_map["z"].offset,
                ],
                "itemsize": msg.point_step,
            })

            arr = np.frombuffer(msg.data, dtype=dtype)
            points = np.stack((arr["x"], arr["y"], arr["z"]), axis=-1).astype(np.float32)
            points = points[np.isfinite(points).all(axis=1)]
            return points

        except Exception as e:
            self.get_logger().warn(f"PointCloud2 conversion failed: {e}")
            return None

    def lidar_raw_to_base(self, raw_points):
        if self.lidar_points_frame == "base":
            return raw_points

        return raw_points @ self.R_base_lidar.T + self.lidar_t_base.reshape(1, 3)

    def apply_lidar_roi(self, points):
        if points is None or len(points) == 0:
            return points

        mask = (
            (points[:, 0] >= self.lidar_x_min) &
            (points[:, 0] <= self.lidar_x_max) &
            (points[:, 1] >= self.lidar_y_min) &
            (points[:, 1] <= self.lidar_y_max) &
            (points[:, 2] >= self.lidar_z_min) &
            (points[:, 2] <= self.lidar_z_max)
        )

        return points[mask]

    def get_latest_lidar_points_copy(self):
        with self.lidar_lock:
            process = None if self.latest_lidar_points_base is None else self.latest_lidar_points_base.copy()
            topview = None if self.latest_lidar_points_topview is None else self.latest_lidar_points_topview.copy()

        return process, topview

    # ============================================================
    # YOLO + LiDAR fusion
    # ============================================================
    def run_yolo_lidar(self, frame, lidar_points_base):
        start = time.time()

        try:
            results = self.model.predict(
                source=frame,
                imgsz=self.imgsz,
                conf=self.conf,
                iou=self.iou,
                classes=[0],
                device=self.device,
                half=self.use_half,
                verbose=False,
            )
        except Exception as e:
            self.get_logger().warn(f"YOLO inference failed: {e}")
            return [], None

        infer_ms = (time.time() - start) * 1000.0

        projected = None

        if lidar_points_base is not None and len(lidar_points_base) > 0:
            projected = self.project_base_points_to_image(lidar_points_base, frame.shape)

        persons = self.extract_persons(results[0], frame.shape, projected)

        return persons, infer_ms

    def extract_persons(self, result, image_shape, projected):
        persons = []
        h, w = image_shape[:2]

        if result.boxes is None or len(result.boxes) == 0:
            return persons

        boxes = result.boxes.xyxy.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()
        clss = result.boxes.cls.cpu().numpy().astype(np.int32)

        has_masks = result.masks is not None
        masks = result.masks.data.cpu().numpy() if has_masks else None

        for i, box in enumerate(boxes):
            if clss[i] != 0:
                continue

            x1, y1, x2, y2 = box
            x1 = int(max(0, min(w - 1, x1)))
            y1 = int(max(0, min(h - 1, y1)))
            x2 = int(max(0, min(w - 1, x2)))
            y2 = int(max(0, min(h - 1, y2)))

            if x2 <= x1 or y2 <= y1:
                continue

            mask = None
            mask_for_lidar = None

            if has_masks and i < len(masks):
                mask_small = masks[i]
                mask = cv2.resize(mask_small, (w, h), interpolation=cv2.INTER_NEAREST)
                mask = (mask > 0.5).astype(np.uint8)

                if self.mask_dilate_px > 0:
                    kernel = np.ones((self.mask_dilate_px, self.mask_dilate_px), np.uint8)
                    mask_for_lidar = cv2.dilate(mask, kernel, iterations=1)
                else:
                    mask_for_lidar = mask

                center_u, center_v = self.get_mask_center(mask, x1, y1, x2, y2)
            else:
                center_u = 0.5 * (x1 + x2)
                center_v = 0.5 * (y1 + y2)

            lidar_info = None

            if projected is not None:
                lidar_info = self.compute_lidar_position_for_person(
                    projected,
                    x1,
                    y1,
                    x2,
                    y2,
                    mask_for_lidar,
                )

            if lidar_info is not None:
                base_x, base_y, base_z = lidar_info["position"]
                source = "lidar"
                lidar_count = lidar_info["count"]
                position_confidence = lidar_info["confidence"]
                xy_spread = lidar_info["xy_spread"]
                depth_std = lidar_info["depth_std"]
                reason = lidar_info["reason"]
            else:
                base_x, base_y, base_z = self.pixel_to_fallback_base_position(center_u, center_v)
                source = "fallback"
                lidar_count = 0
                position_confidence = "FALLBACK"
                xy_spread = 999.0
                depth_std = 999.0
                reason = "no_lidar_association"

            dist = math.sqrt(base_x * base_x + base_y * base_y)

            persons.append({
                "id": len(persons),
                "bbox": (x1, y1, x2, y2),
                "mask": mask,
                "center_uv": (center_u, center_v),
                "confidence": float(confs[i]),
                "base_position": (base_x, base_y, base_z),
                "distance": dist,
                "source": source,
                "lidar_count": lidar_count,
                "position_confidence": position_confidence,
                "xy_spread": xy_spread,
                "depth_std": depth_std,
                "reason": reason,
                "bbox_width": int(x2 - x1),
                "bbox_height": int(y2 - y1),
                "bbox_area": int((x2 - x1) * (y2 - y1)),
                "reflection_suspected": False,
                "weak_visual_candidate": False,
                "control_eligible": True,
                "control_reason": "not_evaluated",
            })

        persons.sort(key=lambda p: p["distance"])

        for idx, p in enumerate(persons):
            p["id"] = idx
            self.update_person_quality_flags(p)

        return persons

    def get_mask_center(self, mask, x1, y1, x2, y2):
        roi = mask[y1:y2, x1:x2]
        ys, xs = np.where(roi > 0)

        if len(xs) < 10:
            return 0.5 * (x1 + x2), 0.5 * (y1 + y2)

        return float(np.mean(xs + x1)), float(np.mean(ys + y1))

    def compute_lidar_position_for_person(self, projected, x1, y1, x2, y2, mask):
        """
        Compute a person's LiDAR-associated base-frame position and quality metrics.

        V3 keeps V2 confidence evaluation:
            HIGH     : enough points, compact XY cluster, stable depth
            MEDIUM   : usable but less stable
            LOW      : LiDAR exists but uncertain
            FALLBACK : handled outside this function when no LiDAR association exists
        """
        if projected is None:
            return None

        u = projected["u"]
        v = projected["v"]
        points_base = projected["points_base"]

        # 1. BBox filter
        bbox_mask = (
            (u >= x1) & (u < x2) &
            (v >= y1) & (v < y2)
        )

        if np.count_nonzero(bbox_mask) == 0:
            return None

        candidate_idx = np.where(bbox_mask)[0]

        # 2. Instance mask filter
        if mask is not None:
            uu = u[candidate_idx].astype(np.int32)
            vv = v[candidate_idx].astype(np.int32)
            inside = mask[vv, uu] > 0
            candidate_idx = candidate_idx[inside]

        if len(candidate_idx) < self.min_lidar_points_in_mask:
            return None

        # 3. Prefer center body region to reduce hand/arm/background association
        bx_w = max(1, x2 - x1)
        bx_h = max(1, y2 - y1)

        uu = u[candidate_idx]
        vv = v[candidate_idx]

        cx1 = x1 + int(0.20 * bx_w)
        cx2 = x2 - int(0.20 * bx_w)
        cy1 = y1 + int(0.10 * bx_h)
        cy2 = y2 - int(0.05 * bx_h)

        central_mask = (
            (uu >= cx1) & (uu <= cx2) &
            (vv >= cy1) & (vv <= cy2)
        )

        if np.count_nonzero(central_mask) >= self.min_lidar_points_in_mask:
            candidate_idx = candidate_idx[central_mask]

        person_points = points_base[candidate_idx]

        if len(person_points) < self.min_lidar_points_in_mask:
            return None

        # 4. Keep frontal valid points only
        forward_x = person_points[:, 0]

        valid_front = (
            (forward_x > 0.15) &
            (forward_x < self.lidar_x_max)
        )

        person_points = person_points[valid_front]

        if len(person_points) < self.min_lidar_points_in_mask:
            return None

        # 5. Select nearest depth cluster.
        # This favors the visible front surface and reduces wall/background pulling.
        forward_x = person_points[:, 0]
        near_x = np.percentile(forward_x, 10)

        near_cluster = person_points[
            forward_x <= near_x + self.near_cluster_depth_m
        ]

        if len(near_cluster) < self.min_lidar_points_in_mask:
            return None

        # 6. Lateral outlier rejection in base XY plane
        median_xyz = np.median(near_cluster, axis=0)

        diff_xy = near_cluster[:, :2] - median_xyz[:2]
        d_xy = np.linalg.norm(diff_xy, axis=1)
        keep = d_xy < self.person_outlier_radius

        if np.count_nonzero(keep) >= self.min_lidar_points_in_mask:
            near_cluster = near_cluster[keep]
            median_xyz = np.median(near_cluster, axis=0)

        # 7. Quality metrics
        lidar_count = int(len(near_cluster))

        if lidar_count >= 2:
            xy_center = np.median(near_cluster[:, :2], axis=0)
            xy_diff = near_cluster[:, :2] - xy_center.reshape(1, 2)
            xy_dist = np.linalg.norm(xy_diff, axis=1)

            xy_spread = float(np.percentile(xy_dist, 90))
            depth_std = float(np.std(near_cluster[:, 0]))
        else:
            xy_spread = 999.0
            depth_std = 999.0

        # 8. Distance-adaptive confidence decision
        person_distance = float(math.sqrt(median_xyz[0] * median_xyz[0] + median_xyz[1] * median_xyz[1]))
        thresholds = self.get_distance_adaptive_confidence_thresholds(person_distance)

        high_count = thresholds["high_count"]
        medium_count = thresholds["medium_count"]
        high_spread = thresholds["high_spread"]
        medium_spread = thresholds["medium_spread"]
        high_depth_std = thresholds["high_depth_std"]
        medium_depth_std = thresholds["medium_depth_std"]

        if (
            lidar_count >= high_count and
            xy_spread <= high_spread and
            depth_std <= high_depth_std
        ):
            confidence = "HIGH"
            reason = "compact_cluster"

        elif (
            lidar_count >= medium_count and
            xy_spread <= medium_spread and
            depth_std <= medium_depth_std
        ):
            confidence = "MEDIUM"
            reason = "usable_cluster"

        else:
            confidence = "LOW"

            if lidar_count < medium_count:
                reason = "few_lidar_points"
            elif xy_spread > medium_spread:
                reason = "wide_xy_spread"
            elif depth_std > medium_depth_std:
                reason = "large_depth_std"
            else:
                reason = "weak_cluster"

        return {
            "position": (
                float(median_xyz[0]),
                float(median_xyz[1]),
                float(median_xyz[2]),
            ),
            "count": lidar_count,
            "xy_spread": xy_spread,
            "depth_std": depth_std,
            "confidence": confidence,
            "reason": reason,
        }




    # ============================================================
    # V3.2 false-positive suppression / adaptive confidence
    # ============================================================
    def get_distance_adaptive_confidence_thresholds(self, distance):
        """
        Relax LiDAR point-count thresholds for distant people.

        Far people naturally return fewer LiDAR points. Without adaptive thresholds,
        they are often classified as LOW or FALLBACK even when the association is usable.
        """
        if distance >= self.very_far_distance_threshold:
            return {
                "high_count": max(6, int(self.confidence_high_count * 0.35)),
                "medium_count": max(3, int(self.confidence_medium_count * 0.40)),
                "high_spread": max(self.confidence_high_spread, 0.70),
                "medium_spread": max(self.confidence_medium_spread, 0.95),
                "high_depth_std": max(self.confidence_high_depth_std, 0.45),
                "medium_depth_std": max(self.confidence_medium_depth_std, 0.75),
            }

        if distance >= self.far_distance_threshold:
            return {
                "high_count": max(10, int(self.confidence_high_count * 0.55)),
                "medium_count": max(4, int(self.confidence_medium_count * 0.55)),
                "high_spread": max(self.confidence_high_spread, 0.60),
                "medium_spread": max(self.confidence_medium_spread, 0.85),
                "high_depth_std": max(self.confidence_high_depth_std, 0.35),
                "medium_depth_std": max(self.confidence_medium_depth_std, 0.65),
            }

        return {
            "high_count": self.confidence_high_count,
            "medium_count": self.confidence_medium_count,
            "high_spread": self.confidence_high_spread,
            "medium_spread": self.confidence_medium_spread,
            "high_depth_std": self.confidence_high_depth_std,
            "medium_depth_std": self.confidence_medium_depth_std,
        }

    def update_person_quality_flags(self, person):
        """
        Mark weak visual candidates and likely reflection/mirror false positives.

        This function does not remove detections from visualization. It only marks
        whether the track should be trusted for future control_count and decision logic.
        """
        pos_conf = person.get("position_confidence", "FALLBACK")
        source = person.get("source", "fallback")
        distance = float(person.get("distance", 999.0))
        lidar_count = int(person.get("lidar_count", 0))
        bbox_h = int(person.get("bbox_height", 0))
        bbox_area = int(person.get("bbox_area", 0))
        hit_count = int(person.get("hit_count", 0))

        reflection_suspected = False
        weak_visual_candidate = False
        control_eligible = True
        control_reason = "accepted"

        if bbox_h < self.min_control_bbox_height_px or bbox_area < self.min_control_bbox_area_px:
            weak_visual_candidate = True

        # Strong false-positive cue: image-only person with no LiDAR support at non-close range.
        if source == "fallback" and pos_conf == "FALLBACK" and lidar_count == 0:
            if distance >= self.reflection_min_distance:
                reflection_suspected = True

        if self.enable_false_positive_filter:
            if pos_conf in ["HIGH", "MEDIUM"]:
                control_eligible = True
                control_reason = "reliable_lidar"

            elif pos_conf == "LOW":
                if (
                    self.allow_low_confidence_close_control and
                    distance <= self.low_control_max_distance and
                    hit_count >= self.low_control_min_hits
                ):
                    control_eligible = True
                    control_reason = "low_close_persistent"
                else:
                    control_eligible = False
                    control_reason = "low_confidence_filtered"

            else:  # FALLBACK
                if self.exclude_fallback_from_control:
                    control_eligible = False
                    control_reason = "fallback_filtered"
                else:
                    if distance <= self.fallback_control_max_distance and hit_count >= self.fallback_control_min_hits:
                        control_eligible = True
                        control_reason = "fallback_close_persistent"
                    else:
                        control_eligible = False
                        control_reason = "fallback_untrusted"

            if reflection_suspected:
                control_eligible = False
                control_reason = "reflection_suspected"

            if weak_visual_candidate and pos_conf in ["LOW", "FALLBACK"]:
                control_eligible = False
                control_reason = "weak_visual_candidate"

        person["reflection_suspected"] = reflection_suspected
        person["weak_visual_candidate"] = weak_visual_candidate
        person["control_eligible"] = control_eligible
        person["control_reason"] = control_reason

    def update_track_quality_flags(self, track):
        """
        Re-evaluate quality flags after tracking updates because hit_count,
        missed_frames, and smoothed position can change the control decision.
        """
        self.update_person_quality_flags(track)

        if track.get("is_stale", False):
            missed = int(track.get("missed_frames", 999))
            if missed > self.stale_control_max_missing:
                track["control_eligible"] = False
                track["control_reason"] = "stale_for_control"


    # ============================================================
    # V3.2 duplicate suppression
    # ============================================================
    def suppress_duplicate_detections(self, detections):
        """
        Remove duplicate detections before tracking.

        This prevents one real person from creating multiple tracks when YOLO
        temporarily splits one person into two boxes/masks during motion.
        """
        if detections is None or len(detections) <= 1:
            return [] if detections is None else detections

        kept = []

        for det in detections:
            merged = False

            for ki, kept_det in enumerate(kept):
                if self.are_detections_duplicate(det, kept_det):
                    better = self.choose_better_detection(det, kept_det)
                    kept[ki] = better
                    merged = True
                    break

            if not merged:
                kept.append(det)

        kept.sort(key=lambda p: p["distance"])

        for idx, p in enumerate(kept):
            # This id is only detection-order id before tracking.
            p["id"] = idx

        return kept

    def are_detections_duplicate(self, det_a, det_b):
        pos_a = np.array(det_a["base_position"], dtype=np.float32)
        pos_b = np.array(det_b["base_position"], dtype=np.float32)
        dist_xy = float(np.linalg.norm(pos_a[:2] - pos_b[:2]))

        iou = self.bbox_iou(det_a.get("bbox"), det_b.get("bbox"))

        if dist_xy <= self.duplicate_detection_distance:
            return True

        if iou >= self.duplicate_bbox_iou:
            return True

        # If both cues are moderately similar, also treat as duplicate.
        if dist_xy <= self.duplicate_detection_distance * 1.35 and iou >= self.duplicate_bbox_iou * 0.60:
            return True

        return False

    def suppress_duplicate_tracks(self, persons):
        """
        Remove duplicate tracks after tracking update.

        Typical duplicate case:
            - T0 becomes STALE for a moment.
            - The same physical person is detected again as T1 ACTIVE.
            - T0 and T1 are spatially close.
            - Keep the better track and remove the duplicate.
        """
        if persons is None or len(persons) <= 1:
            return [] if persons is None else persons

        tracks = list(persons)
        remove_indices = set()

        for i in range(len(tracks)):
            if i in remove_indices:
                continue

            for j in range(i + 1, len(tracks)):
                if j in remove_indices:
                    continue

                if self.are_tracks_duplicate(tracks[i], tracks[j]):
                    keep_i = self.is_track_a_better_than_b(tracks[i], tracks[j])

                    if keep_i:
                        remove_indices.add(j)
                    else:
                        remove_indices.add(i)
                        break

        filtered = [
            t for idx, t in enumerate(tracks)
            if idx not in remove_indices
        ]

        filtered.sort(key=lambda p: p["distance"])
        return filtered

    def are_tracks_duplicate(self, track_a, track_b):
        pos_a = np.array(track_a["base_position"], dtype=np.float32)
        pos_b = np.array(track_b["base_position"], dtype=np.float32)
        dist_xy = float(np.linalg.norm(pos_a[:2] - pos_b[:2]))

        stale_a = bool(track_a.get("is_stale", False))
        stale_b = bool(track_b.get("is_stale", False))

        # Strong rule: stale and active near each other are almost always the same person.
        if stale_a != stale_b and dist_xy <= self.stale_active_suppression_distance:
            return True

        # General duplicate rule for active-active or stale-stale tracks.
        if dist_xy <= self.duplicate_track_distance:
            return True

        # Use bbox IoU only when both tracks have current or recent bbox.
        iou = self.bbox_iou(track_a.get("bbox"), track_b.get("bbox"))
        if iou >= self.duplicate_bbox_iou:
            return True

        return False

    def choose_better_detection(self, det_a, det_b):
        score_a = self.detection_quality_score(det_a)
        score_b = self.detection_quality_score(det_b)

        if score_a >= score_b:
            return det_a
        return det_b

    def detection_quality_score(self, det):
        confidence_rank = {
            "HIGH": 3,
            "MEDIUM": 2,
            "LOW": 1,
            "FALLBACK": 0,
        }

        pos_conf = det.get("position_confidence", "FALLBACK")
        conf_rank = confidence_rank.get(pos_conf, 0)

        lidar_count = float(det.get("lidar_count", 0))
        yolo_conf = float(det.get("confidence", 0.0))
        depth_std = float(det.get("depth_std", 999.0))
        xy_spread = float(det.get("xy_spread", 999.0))

        # Higher is better. Penalize unstable geometry.
        score = (
            conf_rank * 10000.0 +
            min(lidar_count, 2000.0) * 2.0 +
            yolo_conf * 100.0 -
            depth_std * 20.0 -
            xy_spread * 10.0
        )
        return score

    def is_track_a_better_than_b(self, track_a, track_b):
        score_a = self.track_quality_score(track_a)
        score_b = self.track_quality_score(track_b)
        return score_a >= score_b

    def track_quality_score(self, track):
        confidence_rank = {
            "HIGH": 3,
            "MEDIUM": 2,
            "LOW": 1,
            "FALLBACK": 0,
        }

        active_bonus = 100000.0 if not track.get("is_stale", False) else 0.0
        conf_rank = confidence_rank.get(track.get("position_confidence", "FALLBACK"), 0)
        hit_count = float(track.get("hit_count", 0))
        missed = float(track.get("missed_frames", 0))
        lidar_count = float(track.get("lidar_count", 0))
        yolo_conf = float(track.get("confidence", 0.0))

        # ACTIVE is strongly preferred over STALE.
        # Long-lived tracks are preferred over newborn duplicates.
        score = (
            active_bonus +
            conf_rank * 10000.0 +
            hit_count * 250.0 -
            missed * 500.0 +
            min(lidar_count, 2000.0) * 2.0 +
            yolo_conf * 100.0
        )
        return score

    @staticmethod
    def bbox_iou(box_a, box_b):
        if box_a is None or box_b is None:
            return 0.0

        ax1, ay1, ax2, ay2 = box_a
        bx1, by1, bx2, by2 = box_b

        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)

        inter_w = max(0, inter_x2 - inter_x1)
        inter_h = max(0, inter_y2 - inter_y1)
        inter_area = float(inter_w * inter_h)

        area_a = float(max(0, ax2 - ax1) * max(0, ay2 - ay1))
        area_b = float(max(0, bx2 - bx1) * max(0, by2 - by1))

        denom = area_a + area_b - inter_area
        if denom <= 1e-6:
            return 0.0

        return inter_area / denom

    def get_control_human_count(self, persons):
        """
        Count humans for future control logic.

        V3.2 policy:
            - Visualization may show HIGH/MEDIUM/LOW/FALLBACK.
            - Control count only includes tracks marked as control_eligible.
            - FALLBACK reflection/mirror-like detections are excluded by default.
        """
        count = 0

        for p in persons:
            if p.get("control_eligible", False):
                count += 1

        return count


    # ============================================================
    # V3 tracking memory / smoothing / lost-frame persistence
    # ============================================================
    def update_tracks(self, detections):
        """
        Convert per-frame detections into persistent tracks.

        V3 goals:
            - Keep stable track_id over frames
            - Smooth base-frame person position using EMA
            - Keep a track for a short time when detection is missed
            - Mark missed-but-kept tracks as STALE
        """
        now = time.time()

        if detections is None:
            detections = []

        # If no existing tracks, initialize from detections.
        if len(self.tracks) == 0:
            for det in detections:
                self.tracks.append(self.create_track(det, now))
            return self.get_visible_tracks()

        matches = []
        unmatched_track_indices = set(range(len(self.tracks)))
        unmatched_detection_indices = set(range(len(detections)))

        # Build all candidate pairs using XY distance in base frame.
        candidate_pairs = []
        for ti, track in enumerate(self.tracks):
            track_pos = np.array(track["base_position"], dtype=np.float32)

            for di, det in enumerate(detections):
                det_pos = np.array(det["base_position"], dtype=np.float32)
                dist_xy = float(np.linalg.norm(track_pos[:2] - det_pos[:2]))

                if dist_xy <= self.track_match_distance:
                    candidate_pairs.append((dist_xy, ti, di))

        # Greedy nearest-neighbor matching.
        candidate_pairs.sort(key=lambda x: x[0])

        for dist_xy, ti, di in candidate_pairs:
            if ti not in unmatched_track_indices:
                continue
            if di not in unmatched_detection_indices:
                continue

            matches.append((ti, di, dist_xy))
            unmatched_track_indices.remove(ti)
            unmatched_detection_indices.remove(di)

        # Update matched tracks.
        for ti, di, dist_xy in matches:
            self.update_track_with_detection(self.tracks[ti], detections[di], now, dist_xy)

        # Mark unmatched tracks as missed.
        for ti in list(unmatched_track_indices):
            self.mark_track_missed(self.tracks[ti], now)

        # Create new tracks from unmatched detections.
        for di in list(unmatched_detection_indices):
            self.tracks.append(self.create_track(detections[di], now))

        # Remove tracks missed for too long.
        self.tracks = [
            t for t in self.tracks
            if int(t.get("missed_frames", 0)) <= self.max_missing_frames
        ]

        return self.get_visible_tracks()

    def create_track(self, detection, now):
        track_id = self.next_track_id
        self.next_track_id += 1

        det_pos = np.array(detection["base_position"], dtype=np.float32)

        track = dict(detection)
        track["id"] = track_id
        track["track_id"] = track_id
        track["raw_base_position"] = tuple(float(v) for v in det_pos)
        track["smoothed_position"] = tuple(float(v) for v in det_pos)
        track["base_position"] = tuple(float(v) for v in det_pos)
        track["distance"] = float(math.sqrt(det_pos[0] * det_pos[0] + det_pos[1] * det_pos[1]))
        track["first_seen_time"] = now
        track["last_seen_time"] = now
        track["last_update_time"] = now
        track["age_frames"] = 1
        track["hit_count"] = 1
        track["missed_frames"] = 0
        track["is_stale"] = False
        track["track_state"] = "ACTIVE"
        track["match_distance"] = 0.0
        self.update_track_quality_flags(track)
        return track

    def update_track_with_detection(self, track, detection, now, match_distance):
        measured = np.array(detection["base_position"], dtype=np.float32)
        previous = np.array(track.get("smoothed_position", track["base_position"]), dtype=np.float32)

        alpha = min(max(self.smoothing_alpha, 0.0), 1.0)
        smoothed = alpha * measured + (1.0 - alpha) * previous

        previous_track_id = track["track_id"]
        previous_first_seen = track.get("first_seen_time", now)
        previous_age = int(track.get("age_frames", 0))
        previous_hits = int(track.get("hit_count", 0))

        track.clear()
        track.update(detection)

        track["id"] = previous_track_id
        track["track_id"] = previous_track_id
        track["raw_base_position"] = tuple(float(v) for v in measured)
        track["smoothed_position"] = tuple(float(v) for v in smoothed)
        track["base_position"] = tuple(float(v) for v in smoothed)
        track["distance"] = float(math.sqrt(smoothed[0] * smoothed[0] + smoothed[1] * smoothed[1]))
        track["first_seen_time"] = previous_first_seen
        track["last_seen_time"] = now
        track["last_update_time"] = now
        track["age_frames"] = previous_age + 1
        track["hit_count"] = previous_hits + 1
        track["missed_frames"] = 0
        track["is_stale"] = False
        track["track_state"] = "ACTIVE"
        track["match_distance"] = float(match_distance)
        self.update_track_quality_flags(track)

    def mark_track_missed(self, track, now):
        missed = int(track.get("missed_frames", 0)) + 1

        track["missed_frames"] = missed
        track["last_update_time"] = now
        track["is_stale"] = True
        track["track_state"] = "STALE"

        # Keep the last smoothed position.
        pos = np.array(track.get("smoothed_position", track["base_position"]), dtype=np.float32)
        track["base_position"] = tuple(float(v) for v in pos)
        track["distance"] = float(math.sqrt(pos[0] * pos[0] + pos[1] * pos[1]))

        # No current-frame mask exists for a stale track.
        # The last bbox is kept only for visual debugging and is labeled as STALE.
        self.update_track_quality_flags(track)

    def get_visible_tracks(self):
        visible = [
            dict(t) for t in self.tracks
            if int(t.get("missed_frames", 0)) <= self.max_missing_frames
        ]

        visible.sort(key=lambda p: p["distance"])

        return visible


    # ============================================================
    # Projection math
    # ============================================================
    def project_base_points_to_image(self, points_base, image_shape):
        if points_base is None or len(points_base) == 0:
            return None

        h_rot, w_rot = image_shape[:2]

        p_rel_base = points_base - self.camera_t_base.reshape(1, 3)

        # base -> camera body
        p_body = p_rel_base @ self.R_base_camera_body

        x_body = p_body[:, 0]
        y_body = p_body[:, 1]
        z_body = p_body[:, 2]

        # body -> optical
        X_opt = -y_body
        Y_opt = -z_body
        Z_opt = x_body

        valid = Z_opt > 0.05
        if np.count_nonzero(valid) == 0:
            return None

        X_opt = X_opt[valid]
        Y_opt = Y_opt[valid]
        Z_opt = Z_opt[valid]
        points_valid = points_base[valid]

        u0 = (self.raw_fx * X_opt / Z_opt) + self.raw_cx
        v0 = (self.raw_fy * Y_opt / Z_opt) + self.raw_cy

        raw_valid = (
            (u0 >= 0) & (u0 < self.raw_w) &
            (v0 >= 0) & (v0 < self.raw_h)
        )

        if np.count_nonzero(raw_valid) == 0:
            return None

        u0 = u0[raw_valid]
        v0 = v0[raw_valid]
        points_valid = points_valid[raw_valid]
        Z_opt = Z_opt[raw_valid]

        u_rot, v_rot = self.original_uv_to_rotated_uv(u0, v0)

        rot_valid = (
            (u_rot >= 0) & (u_rot < w_rot) &
            (v_rot >= 0) & (v_rot < h_rot)
        )

        if np.count_nonzero(rot_valid) == 0:
            return None

        return {
            "u": u_rot[rot_valid].astype(np.int32),
            "v": v_rot[rot_valid].astype(np.int32),
            "points_base": points_valid[rot_valid],
            "z_opt": Z_opt[rot_valid],
        }

    def pixel_to_fallback_base_position(self, u_rot, v_rot):
        u0, v0 = self.rotated_uv_to_original_uv(u_rot, v_rot)

        z_opt = self.fixed_distance
        X_opt = (u0 - self.raw_cx) * z_opt / self.raw_fx
        Y_opt = (v0 - self.raw_cy) * z_opt / self.raw_fy

        x_body = z_opt
        y_body = -X_opt
        z_body = -Y_opt

        p_body = np.array([x_body, y_body, z_body], dtype=np.float32)
        p_base = self.R_base_camera_body @ p_body + self.camera_t_base

        return float(p_base[0]), float(p_base[1]), float(p_base[2])

    # ============================================================
    # Image rotation
    # ============================================================
    def rotate_image(self, img):
        if self.image_rotate == "none":
            return img
        if self.image_rotate == "cw90":
            return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        if self.image_rotate == "ccw90":
            return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if self.image_rotate == "180":
            return cv2.rotate(img, cv2.ROTATE_180)
        return img

    def original_uv_to_rotated_uv(self, u0, v0):
        if self.image_rotate == "none":
            return u0, v0

        if self.image_rotate == "cw90":
            u = (self.raw_h - 1) - v0
            v = u0
            return u, v

        if self.image_rotate == "ccw90":
            u = v0
            v = (self.raw_w - 1) - u0
            return u, v

        if self.image_rotate == "180":
            u = (self.raw_w - 1) - u0
            v = (self.raw_h - 1) - v0
            return u, v

        return u0, v0

    def rotated_uv_to_original_uv(self, u, v):
        if self.image_rotate == "none":
            return u, v

        if self.image_rotate == "cw90":
            u0 = v
            v0 = (self.raw_h - 1) - u
            return u0, v0

        if self.image_rotate == "ccw90":
            u0 = (self.raw_w - 1) - v
            v0 = u
            return u0, v0

        if self.image_rotate == "180":
            u0 = (self.raw_w - 1) - u
            v0 = (self.raw_h - 1) - v
            return u0, v0

        return u, v

    @staticmethod
    def get_confidence_color(position_confidence):
        if position_confidence == "HIGH":
            return (0, 255, 0)       # Green
        if position_confidence == "MEDIUM":
            return (0, 255, 255)     # Yellow
        if position_confidence == "LOW":
            return (0, 165, 255)     # Orange
        return (0, 0, 255)           # Red

    def get_track_draw_color(self, person):
        if person.get("is_stale", False):
            return self.stale_track_color
        return self.get_confidence_color(person.get("position_confidence", "FALLBACK"))


    # ============================================================
    # Visualization
    # ============================================================
    def draw_yolo_view(self, frame, persons):
        out = frame.copy()

        for person in persons:
            x1, y1, x2, y2 = person["bbox"]
            mask = person["mask"]
            source = person["source"]
            position_confidence = person.get("position_confidence", "FALLBACK")
            track_state = person.get("track_state", "ACTIVE")
            color = self.get_track_draw_color(person)

            if self.show_mask_overlay and mask is not None and not person.get("is_stale", False):
                overlay = np.zeros_like(out)
                if position_confidence in ["HIGH", "MEDIUM"]:
                    overlay[:, :, 1] = mask * 255
                elif position_confidence == "LOW":
                    overlay[:, :, 1] = mask * 128
                    overlay[:, :, 2] = mask * 255
                else:
                    overlay[:, :, 2] = mask * 255
                out = cv2.addWeighted(out, 1.0, overlay, 0.30, 0.0)

            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)

            u, v = person["center_uv"]
            cv2.circle(out, (int(u), int(v)), 5, (0, 255, 255), -1)

            x, y, z = person["base_position"]

            warn_tag = ""
            if person.get("reflection_suspected", False):
                warn_tag = " REFLECT?"
            elif not person.get("control_eligible", True):
                warn_tag = " NOCTRL"

            label = (
                f"T{person.get('track_id', person['id'])} {track_state} "
                f"{source} {position_confidence}{warn_tag} "
                f"{person['distance']:.2f}m "
                f"x={x:.2f} y={y:.2f} "
                f"n={person['lidar_count']} "
                f"miss={person.get('missed_frames', 0)}"
            )

            cv2.putText(
                out,
                label,
                (x1, max(25, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                color,
                2,
                cv2.LINE_AA,
            )

        avg_ms = sum(self.infer_times) / len(self.infer_times) if self.infer_times else 0.0

        control_count = self.get_control_human_count(persons)

        status = (
            f"LiDAR_Vision V3.2 | tracks={len(persons)} | control={control_count} | avg={avg_ms:.1f}ms | "
            f"Green=HIGH, Yellow=MED, Orange=LOW, Red=FALLBACK, Gray=STALE"
        )

        cv2.putText(out, status, (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, status, (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 255), 1, cv2.LINE_AA)

        return out

    def draw_top_view(self, persons, lidar_points_topview):
        size = self.topview_size
        canvas = np.zeros((size, size, 3), dtype=np.uint8)
        canvas[:] = (20, 20, 20)

        origin_x = size // 2
        origin_y = int(size * 0.82)

        self.draw_grid(canvas, origin_x, origin_y)

        if self.draw_lidar_topview and lidar_points_topview is not None:
            self.draw_lidar_points_topview(canvas, lidar_points_topview, origin_x, origin_y)

        self.draw_robot(canvas, origin_x, origin_y)
        self.draw_sensor_positions(canvas, origin_x, origin_y)

        for person in persons:
            self.draw_person_on_topview(canvas, person, origin_x, origin_y)

        self.draw_topview_labels(canvas, origin_x, origin_y)

        return canvas

    def draw_grid(self, canvas, origin_x, origin_y):
        size = canvas.shape[0]
        grid_color = (55, 55, 55)
        axis_color = (180, 180, 180)

        max_forward_px = int(self.max_forward_m * self.meter_to_pixel)
        max_side_px = int(self.max_side_m * self.meter_to_pixel)

        for m in np.arange(0.0, self.max_forward_m + 0.001, 0.5):
            y = int(origin_y - m * self.meter_to_pixel)
            if 0 <= y < size:
                cv2.line(canvas, (origin_x - max_side_px, y),
                         (origin_x + max_side_px, y), grid_color, 1)
                cv2.putText(canvas, f"{m:.1f}m",
                            (origin_x + max_side_px + 8, y + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                            (120, 120, 120), 1, cv2.LINE_AA)

        for m in np.arange(-self.max_side_m, self.max_side_m + 0.001, 0.5):
            x = int(origin_x - m * self.meter_to_pixel)
            if 0 <= x < size:
                cv2.line(canvas, (x, origin_y),
                         (x, origin_y - max_forward_px), grid_color, 1)

        cv2.line(canvas, (origin_x, origin_y),
                 (origin_x, max(0, origin_y - max_forward_px)), axis_color, 2)
        cv2.line(canvas, (origin_x - max_side_px, origin_y),
                 (origin_x + max_side_px, origin_y), axis_color, 2)

    def draw_robot(self, canvas, origin_x, origin_y):
        body_w = 52
        body_h = 76

        x1 = origin_x - body_w // 2
        y1 = origin_y - body_h // 2
        x2 = origin_x + body_w // 2
        y2 = origin_y + body_h // 2

        cv2.rectangle(canvas, (x1, y1), (x2, y2), (80, 160, 255), -1)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (255, 255, 255), 2)

        cv2.arrowedLine(
            canvas,
            (origin_x, origin_y),
            (origin_x, origin_y - 75),
            (0, 255, 255),
            3,
            tipLength=0.25,
        )

        cv2.putText(canvas, "GO2", (origin_x - 24, origin_y + 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.60,
                    (255, 255, 255), 1, cv2.LINE_AA)

    def draw_sensor_positions(self, canvas, origin_x, origin_y):
        lx, ly, _ = self.lidar_t_base
        lpx, lpy = self.base_to_canvas(lx, ly, origin_x, origin_y)

        if self.inside(lpx, lpy, canvas.shape[0]):
            cv2.circle(canvas, (lpx, lpy), 7, (255, 120, 0), -1)
            cv2.putText(canvas, "Hesai", (lpx + 8, lpy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                        (255, 160, 80), 1, cv2.LINE_AA)

        cx, cy, _ = self.camera_t_base
        cpx, cpy = self.base_to_canvas(cx, cy, origin_x, origin_y)

        if self.inside(cpx, cpy, canvas.shape[0]):
            cv2.circle(canvas, (cpx, cpy), 7, (0, 180, 255), -1)
            cv2.putText(canvas, "D435i", (cpx + 8, cpy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                        (0, 220, 255), 1, cv2.LINE_AA)

    def draw_lidar_points_topview(self, canvas, lidar_points, origin_x, origin_y):
        for p in lidar_points:
            px, py = self.base_to_canvas(float(p[0]), float(p[1]), origin_x, origin_y)
            if self.inside(px, py, canvas.shape[0]):
                cv2.circle(canvas, (px, py), 1, (90, 90, 90), -1)

    def draw_person_on_topview(self, canvas, person, origin_x, origin_y):
        x, y, _ = person["base_position"]
        px, py = self.base_to_canvas(x, y, origin_x, origin_y)

        if not self.inside(px, py, canvas.shape[0]):
            return

        source = person["source"]
        position_confidence = person.get("position_confidence", "FALLBACK")
        track_state = person.get("track_state", "ACTIVE")
        color = self.get_track_draw_color(person)

        # Draw person as top-view body block
        block_w_px = int(0.55 * self.meter_to_pixel)
        block_d_px = int(0.45 * self.meter_to_pixel)

        x1 = px - block_w_px // 2
        y1 = py - block_d_px // 2
        x2 = px + block_w_px // 2
        y2 = py + block_d_px // 2

        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, -1)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (255, 255, 255), 2)
        cv2.line(canvas, (origin_x, origin_y), (px, py), color, 2)

        dist = person["distance"]

        text1 = (
            f"T{person.get('track_id', person['id'])} "
            f"{track_state} {dist:.2f}m {position_confidence}"
        )
        text2 = (
            f"x={x:.2f}, y={y:.2f}, "
            f"n={person['lidar_count']} "
            f"miss={person.get('missed_frames', 0)}"
        )
        text3 = f"{person.get('reason', '')} | {person.get('control_reason', '')}"

        cv2.putText(canvas, text1, (px + 18, py - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, text2, (px + 18, py + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                    (180, 255, 180), 1, cv2.LINE_AA)
        cv2.putText(canvas, text3, (px + 18, py + 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.34,
                    (180, 180, 180), 1, cv2.LINE_AA)

    def draw_topview_labels(self, canvas, origin_x, origin_y):
        cv2.putText(canvas, "LiDAR_Vision Robot-Centered 2D Top View",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.70, (255, 255, 255), 2, cv2.LINE_AA)

        cv2.putText(canvas, "+X Forward",
                    (origin_x + 15, 60), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 255), 1, cv2.LINE_AA)

        cv2.putText(canvas, "+Y Left",
                    (30, origin_y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 255), 1, cv2.LINE_AA)

        cv2.putText(canvas, "-Y Right",
                    (canvas.shape[1] - 130, origin_y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 255), 1, cv2.LINE_AA)

        cv2.putText(canvas,
                    "Gray dots: LiDAR | Green: HIGH | Yellow: MEDIUM | Orange: LOW | Red: FALLBACK | Gray block: STALE",
                    (20, canvas.shape[0] - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.43,
                    (180, 180, 180), 1, cv2.LINE_AA)

    def base_to_canvas(self, base_x, base_y, origin_x, origin_y):
        px = int(origin_x - base_y * self.meter_to_pixel)
        py = int(origin_y - base_x * self.meter_to_pixel)
        return px, py

    @staticmethod
    def inside(px, py, size):
        return 0 <= px < size and 0 <= py < size

    # ============================================================
    # Math
    # ============================================================
    @staticmethod
    def euler_to_rot(roll, pitch, yaw):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)

        rx = np.array([
            [1, 0, 0],
            [0, cr, -sr],
            [0, sr, cr],
        ], dtype=np.float32)

        ry = np.array([
            [cp, 0, sp],
            [0, 1, 0],
            [-sp, 0, cp],
        ], dtype=np.float32)

        rz = np.array([
            [cy, -sy, 0],
            [sy, cy, 0],
            [0, 0, 1],
        ], dtype=np.float32)

        return rz @ ry @ rx

    # ============================================================
    # Status
    # ============================================================
    def print_status(self, lidar_points, persons):
        now = time.time()
        if now - self.last_log_time < 1.0:
            return

        self.last_log_time = now

        avg_ms = sum(self.infer_times) / len(self.infer_times) if self.infer_times else 0.0
        lidar_n = 0 if lidar_points is None else len(lidar_points)

        active_n = sum(1 for p in persons if not p.get("is_stale", False))
        stale_n = sum(1 for p in persons if p.get("is_stale", False))
        control_count = self.get_control_human_count(persons)

        self.get_logger().info(
            f"status | tracks={len(persons)} | active={active_n} | stale={stale_n} | "
            f"control_count={control_count} | "
            f"avg={avg_ms:.1f}ms | lidar_process_points={lidar_n}"
        )

        for p in persons:
            x, y, z = p["base_position"]
            self.get_logger().info(
                f"  T{p.get('track_id', p['id'])} {p.get('track_state', 'ACTIVE')} "
                f"{p['source']} {p.get('position_confidence', 'FALLBACK')} "
                f"dist={p['distance']:.2f}m "
                f"x={x:.2f}, y={y:.2f}, z={z:.2f}, "
                f"yolo_conf={p['confidence']:.2f}, "
                f"n={p['lidar_count']}, "
                f"missed={p.get('missed_frames', 0)}, "
                f"hits={p.get('hit_count', 0)}, "
                f"match={p.get('match_distance', 0.0):.2f}, "
                f"control={p.get('control_eligible', False)}, "
                f"control_reason={p.get('control_reason', '')}, "
                f"reflection={p.get('reflection_suspected', False)}, "
                f"spread={p.get('xy_spread', 999.0):.2f}, "
                f"depth_std={p.get('depth_std', 999.0):.2f}, "
                f"reason={p.get('reason', '')}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = LiDARVisionTopViewV32Node()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.stop()
            node.destroy_node()

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

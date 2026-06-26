#!/usr/bin/env python3

import argparse
import atexit
import math
import signal
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class RealSenseLidarDepthCalib(Node):
    """
    RealSense depth and LiDAR point cloud alignment checker.

    Coordinate convention:
        base x: forward
        base y: left
        base z: up

    Camera body convention:
        x: forward
        y: left
        z: up

    Camera optical convention:
        X: right
        Y: down
        Z: forward
    """

    def __init__(self, args):
        super().__init__("go2_realsense_lidar_depth_calib_v2")

        self.args = args
        self.running = True
        self.pipeline_started = False

        # RealSense
        self.width = args.width
        self.height = args.height
        self.camera_fps = args.camera_fps
        self.image_rotate = args.image_rotate

        self.pipeline = None
        self.config = None
        self.profile = None
        self.align = None
        self.depth_scale = None

        self.raw_fx = None
        self.raw_fy = None
        self.raw_cx = None
        self.raw_cy = None
        self.raw_w = None
        self.raw_h = None

        # LiDAR
        self.lidar_topic = args.lidar_topic
        self.lidar_points_frame = args.lidar_points_frame
        self.latest_lidar_points_base = None
        self.lidar_lock = threading.Lock()

        # Initial sensor pose from your mounting description
        self.camera_base_x = args.camera_base_x
        self.camera_base_y = args.camera_base_y
        self.camera_base_z = args.camera_base_z

        self.camera_roll_deg = args.camera_roll_deg
        self.camera_pitch_deg = args.camera_pitch_deg
        self.camera_yaw_deg = args.camera_yaw_deg

        self.lidar_base_x = args.lidar_base_x
        self.lidar_base_y = args.lidar_base_y
        self.lidar_base_z = args.lidar_base_z

        self.lidar_roll_deg = args.lidar_roll_deg
        self.lidar_pitch_deg = args.lidar_pitch_deg
        self.lidar_yaw_deg = args.lidar_yaw_deg

        self.R_base_camera_body = np.eye(3, dtype=np.float32)
        self.R_base_lidar = np.eye(3, dtype=np.float32)
        self.update_extrinsics()

        # ROI
        self.lidar_x_min = args.lidar_x_min
        self.lidar_x_max = args.lidar_x_max
        self.lidar_y_min = args.lidar_y_min
        self.lidar_y_max = args.lidar_y_max
        self.lidar_z_min = args.lidar_z_min
        self.lidar_z_max = args.lidar_z_max
        self.max_lidar_points = args.max_lidar_points

        # Comparison
        self.max_compare_points = args.max_compare_points
        self.depth_min = args.depth_min
        self.depth_max = args.depth_max
        self.depth_sample_radius = args.depth_sample_radius
        self.good_error_threshold = args.good_error_threshold

        # UI
        self.show_lidar_on_rgb = True
        self.last_status_time = time.time()
        self.last_stats = {}

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        atexit.register(self.stop)

        self.create_lidar_subscriber()
        self.start_realsense()
        self.create_windows_and_trackbars()

        print("[INFO] Calibration viewer started.", flush=True)
        print("[INFO] Press q or ESC to quit.", flush=True)
        print("[INFO] Adjust Camera Pitch/Yaw/Z while comparing RealSense depth and LiDAR projection.", flush=True)

    # ---------------------------------------------------------
    # Setup
    # ---------------------------------------------------------
    def signal_handler(self, signum, frame):
        print("\n[INFO] Stop signal received.", flush=True)
        self.running = False

    def create_lidar_subscriber(self):
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_lidar = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.lidar_callback,
            qos
        )

        print(f"[INFO] Subscribed LiDAR topic: {self.lidar_topic}", flush=True)

    def start_realsense(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(
            rs.stream.color,
            self.width,
            self.height,
            rs.format.bgr8,
            self.camera_fps
        )

        self.config.enable_stream(
            rs.stream.depth,
            self.width,
            self.height,
            rs.format.z16,
            self.camera_fps
        )

        print("[INFO] Starting RealSense color + depth pipeline...", flush=True)

        self.profile = self.pipeline.start(self.config)
        self.pipeline_started = True

        self.align = rs.align(rs.stream.color)

        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = float(depth_sensor.get_depth_scale())

        color_stream = self.profile.get_stream(rs.stream.color)
        intr = color_stream.as_video_stream_profile().get_intrinsics()

        self.raw_fx = float(intr.fx)
        self.raw_fy = float(intr.fy)
        self.raw_cx = float(intr.ppx)
        self.raw_cy = float(intr.ppy)
        self.raw_w = int(intr.width)
        self.raw_h = int(intr.height)

        print(
            f"[INFO] RealSense intrinsics: fx={self.raw_fx:.2f}, fy={self.raw_fy:.2f}, "
            f"cx={self.raw_cx:.2f}, cy={self.raw_cy:.2f}, w={self.raw_w}, h={self.raw_h}",
            flush=True
        )
        print(f"[INFO] RealSense depth scale: {self.depth_scale}", flush=True)

        warmup_start = time.time()
        while time.time() - warmup_start < 0.5:
            self.pipeline.poll_for_frames()
            time.sleep(0.01)

    def create_windows_and_trackbars(self):
        cv2.namedWindow("RGB + LiDAR Projection", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Depth Difference View", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Top View", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Calibration Controls", cv2.WINDOW_NORMAL)

        cv2.resizeWindow("RGB + LiDAR Projection", 900, 650)
        cv2.resizeWindow("Depth Difference View", 900, 650)
        cv2.resizeWindow("Top View", 800, 800)
        cv2.resizeWindow("Calibration Controls", 520, 360)

        cv2.moveWindow("RGB + LiDAR Projection", 20, 40)
        cv2.moveWindow("Depth Difference View", 940, 40)
        cv2.moveWindow("Top View", 20, 730)
        cv2.moveWindow("Calibration Controls", 940, 730)

        blank = np.zeros((360, 520, 3), dtype=np.uint8)
        cv2.imshow("Calibration Controls", blank)

        # Trackbar ranges use offsets to represent negative values.
        cv2.createTrackbar("cam_pitch_x10", "Calibration Controls", int(self.camera_pitch_deg * 10 + 300), 600, self.on_trackbar)
        cv2.createTrackbar("cam_yaw_x10", "Calibration Controls", int(self.camera_yaw_deg * 10 + 300), 600, self.on_trackbar)
        cv2.createTrackbar("cam_roll_x10", "Calibration Controls", int(self.camera_roll_deg * 10 + 300), 600, self.on_trackbar)

        cv2.createTrackbar("cam_x_cm", "Calibration Controls", int(self.camera_base_x * 100), 100, self.on_trackbar)
        cv2.createTrackbar("cam_y_cm+100", "Calibration Controls", int(self.camera_base_y * 100 + 100), 200, self.on_trackbar)
        cv2.createTrackbar("cam_z_cm", "Calibration Controls", int(self.camera_base_z * 100), 100, self.on_trackbar)

        cv2.createTrackbar("lidar_yaw_deg+180", "Calibration Controls", int(self.lidar_yaw_deg + 180), 360, self.on_trackbar)

        cv2.waitKey(1)

    def on_trackbar(self, value):
        self.read_trackbar_values()
        self.update_extrinsics()

    def read_trackbar_values(self):
        try:
            self.camera_pitch_deg = (cv2.getTrackbarPos("cam_pitch_x10", "Calibration Controls") - 300) / 10.0
            self.camera_yaw_deg = (cv2.getTrackbarPos("cam_yaw_x10", "Calibration Controls") - 300) / 10.0
            self.camera_roll_deg = (cv2.getTrackbarPos("cam_roll_x10", "Calibration Controls") - 300) / 10.0

            self.camera_base_x = cv2.getTrackbarPos("cam_x_cm", "Calibration Controls") / 100.0
            self.camera_base_y = (cv2.getTrackbarPos("cam_y_cm+100", "Calibration Controls") - 100) / 100.0
            self.camera_base_z = cv2.getTrackbarPos("cam_z_cm", "Calibration Controls") / 100.0

            self.lidar_yaw_deg = cv2.getTrackbarPos("lidar_yaw_deg+180", "Calibration Controls") - 180.0

        except Exception:
            pass

    def update_extrinsics(self):
        self.camera_t_base = np.array(
            [self.camera_base_x, self.camera_base_y, self.camera_base_z],
            dtype=np.float32
        )

        self.lidar_t_base = np.array(
            [self.lidar_base_x, self.lidar_base_y, self.lidar_base_z],
            dtype=np.float32
        )

        self.R_base_camera_body = self.euler_to_rot(
            math.radians(self.camera_roll_deg),
            math.radians(self.camera_pitch_deg),
            math.radians(self.camera_yaw_deg)
        )

        self.R_base_lidar = self.euler_to_rot(
            math.radians(self.lidar_roll_deg),
            math.radians(self.lidar_pitch_deg),
            math.radians(self.lidar_yaw_deg)
        )

    # ---------------------------------------------------------
    # Main loop
    # ---------------------------------------------------------
    def run(self):
        try:
            while self.running:
                rclpy.spin_once(self, timeout_sec=0.0)
                self.read_trackbar_values()
                self.update_extrinsics()

                color, depth_m = self.get_aligned_realsense_frames()
                if color is None or depth_m is None:
                    time.sleep(0.002)
                    continue

                color_rot = self.rotate_image(color)
                depth_rot = self.rotate_depth(depth_m)

                lidar_points_base = self.get_latest_lidar_points_base_copy()
                projected = None

                if lidar_points_base is not None and len(lidar_points_base) > 0:
                    projected = self.project_base_points_to_rotated_image(
                        lidar_points_base,
                        color_rot.shape
                    )

                rgb_view, diff_view, top_view = self.make_visualizations(
                    color_rot,
                    depth_rot,
                    lidar_points_base,
                    projected
                )

                cv2.imshow("RGB + LiDAR Projection", rgb_view)
                cv2.imshow("Depth Difference View", diff_view)
                cv2.imshow("Top View", top_view)
                cv2.imshow("Calibration Controls", self.make_control_panel())

                key = cv2.waitKey(1) & 0xFF

                if key == ord("q") or key == 27:
                    self.running = False

                if key == ord("s"):
                    self.print_current_params()

                self.print_status()

        except KeyboardInterrupt:
            self.running = False

        finally:
            self.stop()

    def get_aligned_realsense_frames(self):
        if not self.pipeline_started:
            return None, None

        try:
            frames = self.pipeline.poll_for_frames()
            if not frames:
                return None, None

            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                return None, None

            color = np.asanyarray(color_frame.get_data())
            depth_raw = np.asanyarray(depth_frame.get_data()).astype(np.float32)
            depth_m = depth_raw * self.depth_scale

            return color, depth_m

        except Exception as e:
            print(f"[ERROR] RealSense frame error: {e}", flush=True)
            return None, None

    # ---------------------------------------------------------
    # LiDAR
    # ---------------------------------------------------------
    def lidar_callback(self, msg):
        raw_points = self.pointcloud2_to_xyz_numpy(msg)
        if raw_points is None or len(raw_points) == 0:
            return

        base_points = self.lidar_raw_to_base(raw_points)
        base_points = self.apply_lidar_roi(base_points)

        if len(base_points) > self.max_lidar_points:
            idx = np.random.choice(len(base_points), self.max_lidar_points, replace=False)
            base_points = base_points[idx]

        with self.lidar_lock:
            self.latest_lidar_points_base = base_points

    def pointcloud2_to_xyz_numpy(self, msg):
        field_map = {field.name: field for field in msg.fields}

        if "x" not in field_map or "y" not in field_map or "z" not in field_map:
            return None

        try:
            dtype = np.dtype({
                "names": ["x", "y", "z"],
                "formats": [np.float32, np.float32, np.float32],
                "offsets": [
                    field_map["x"].offset,
                    field_map["y"].offset,
                    field_map["z"].offset
                ],
                "itemsize": msg.point_step
            })

            arr = np.frombuffer(msg.data, dtype=dtype)
            points = np.stack((arr["x"], arr["y"], arr["z"]), axis=-1).astype(np.float32)
            points = points[np.isfinite(points).all(axis=1)]
            return points

        except Exception as e:
            print(f"[WARN] PointCloud2 conversion failed: {e}", flush=True)
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

    def get_latest_lidar_points_base_copy(self):
        with self.lidar_lock:
            if self.latest_lidar_points_base is None:
                return None
            return self.latest_lidar_points_base.copy()

    # ---------------------------------------------------------
    # Projection
    # ---------------------------------------------------------
    def project_base_points_to_rotated_image(self, points_base, image_shape):
        if points_base is None or len(points_base) == 0:
            return None

        h_rot, w_rot = image_shape[:2]

        p_rel_base = points_base - self.camera_t_base.reshape(1, 3)

        # base -> camera body
        p_body = p_rel_base @ self.R_base_camera_body

        x_body = p_body[:, 0]
        y_body = p_body[:, 1]
        z_body = p_body[:, 2]

        # camera body -> optical
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
        Z_opt = Z_opt[raw_valid]
        points_valid = points_valid[raw_valid]

        u_rot, v_rot = self.original_uv_to_rotated_uv(u0, v0)

        rot_valid = (
            (u_rot >= 0) & (u_rot < w_rot) &
            (v_rot >= 0) & (v_rot < h_rot)
        )

        if np.count_nonzero(rot_valid) == 0:
            return None

        u_rot = u_rot[rot_valid].astype(np.int32)
        v_rot = v_rot[rot_valid].astype(np.int32)
        Z_opt = Z_opt[rot_valid]
        points_valid = points_valid[rot_valid]

        return {
            "u": u_rot,
            "v": v_rot,
            "lidar_depth": Z_opt,
            "points_base": points_valid
        }

    # ---------------------------------------------------------
    # Depth comparison
    # ---------------------------------------------------------
    def sample_depth_values(self, depth_m, u, v):
        radius = self.depth_sample_radius
        h, w = depth_m.shape[:2]

        sampled = np.zeros(len(u), dtype=np.float32)

        for i in range(len(u)):
            x = int(u[i])
            y = int(v[i])

            x1 = max(0, x - radius)
            x2 = min(w, x + radius + 1)
            y1 = max(0, y - radius)
            y2 = min(h, y + radius + 1)

            patch = depth_m[y1:y2, x1:x2]
            valid = patch[(patch > self.depth_min) & (patch < self.depth_max)]

            if len(valid) == 0:
                sampled[i] = 0.0
            else:
                sampled[i] = float(np.median(valid))

        return sampled

    def compute_depth_errors(self, depth_m, projected):
        if projected is None:
            return None

        u = projected["u"]
        v = projected["v"]
        lidar_depth = projected["lidar_depth"]

        if len(u) == 0:
            return None

        if len(u) > self.max_compare_points:
            idx = np.random.choice(len(u), self.max_compare_points, replace=False)
            u = u[idx]
            v = v[idx]
            lidar_depth = lidar_depth[idx]
            points_base = projected["points_base"][idx]
        else:
            points_base = projected["points_base"]

        rs_depth = self.sample_depth_values(depth_m, u, v)

        valid = (
            (rs_depth > self.depth_min) &
            (rs_depth < self.depth_max) &
            np.isfinite(lidar_depth)
        )

        if np.count_nonzero(valid) == 0:
            return None

        u = u[valid]
        v = v[valid]
        lidar_depth = lidar_depth[valid]
        rs_depth = rs_depth[valid]
        points_base = points_base[valid]

        error = lidar_depth - rs_depth
        abs_error = np.abs(error)

        return {
            "u": u,
            "v": v,
            "rs_depth": rs_depth,
            "lidar_depth": lidar_depth,
            "error": error,
            "abs_error": abs_error,
            "points_base": points_base
        }

    # ---------------------------------------------------------
    # Visualization
    # ---------------------------------------------------------
    def make_visualizations(self, color, depth_m, lidar_points_base, projected):
        errors = self.compute_depth_errors(depth_m, projected)

        rgb_view = color.copy()
        depth_view = self.make_depth_colormap(depth_m)
        top_view = self.make_top_view(lidar_points_base, errors)

        if errors is not None:
            self.draw_error_points(rgb_view, errors)
            self.draw_error_points(depth_view, errors)
            self.update_stats(errors)
        else:
            self.last_stats = {
                "matched": 0,
                "median_abs_error": None,
                "mean_abs_error": None,
                "median_error": None
            }

        self.draw_text_overlay(rgb_view, "RGB + LiDAR Projection")
        self.draw_text_overlay(depth_view, "RealSense Depth + LiDAR Error")

        return rgb_view, depth_view, top_view

    def draw_error_points(self, image, errors):
        u = errors["u"]
        v = errors["v"]
        error = errors["error"]
        abs_error = errors["abs_error"]

        for i in range(len(u)):
            e = float(error[i])
            ae = float(abs_error[i])

            if ae < self.good_error_threshold:
                color = (0, 255, 0)      # good
            elif e > 0:
                color = (255, 0, 0)      # LiDAR is farther than RealSense
            else:
                color = (0, 0, 255)      # LiDAR is closer than RealSense

            cv2.circle(image, (int(u[i]), int(v[i])), 2, color, -1)

    def make_depth_colormap(self, depth_m):
        depth_clip = np.clip(depth_m, 0.0, self.depth_max)
        depth_norm = (depth_clip / self.depth_max * 255.0).astype(np.uint8)
        color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

        invalid = depth_m <= self.depth_min
        color[invalid] = (0, 0, 0)

        return color

    def update_stats(self, errors):
        abs_error = errors["abs_error"]
        error = errors["error"]

        self.last_stats = {
            "matched": int(len(abs_error)),
            "median_abs_error": float(np.median(abs_error)),
            "mean_abs_error": float(np.mean(abs_error)),
            "median_error": float(np.median(error)),
        }

    def draw_text_overlay(self, img, title):
        stats = self.last_stats

        lines = [
            title,
            f"Cam xyz=({self.camera_base_x:.2f}, {self.camera_base_y:.2f}, {self.camera_base_z:.2f}) m",
            f"Cam rpy=({self.camera_roll_deg:.1f}, {self.camera_pitch_deg:.1f}, {self.camera_yaw_deg:.1f}) deg",
            f"LiDAR yaw={self.lidar_yaw_deg:.1f} deg | image_rotate={self.image_rotate}",
        ]

        if stats.get("matched", 0) > 0:
            lines.append(
                f"matched={stats['matched']} | median_abs={stats['median_abs_error']:.3f}m | "
                f"mean_abs={stats['mean_abs_error']:.3f}m | med_err={stats['median_error']:.3f}m"
            )
        else:
            lines.append("matched=0 | No valid depth comparison")

        lines.append("Green: good | Blue: LiDAR farther | Red: LiDAR closer | Press 's' to print params")

        y = 25
        for line in lines:
            cv2.putText(
                img,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )
            cv2.putText(
                img,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 0),
                1,
                cv2.LINE_AA
            )
            y += 24

    def make_control_panel(self):
        panel = np.zeros((360, 520, 3), dtype=np.uint8)
        panel[:] = (30, 30, 30)

        lines = [
            "Calibration Controls",
            "",
            "Adjust trackbars:",
            "- cam_pitch_x10: main Y-axis rotation tuning",
            "- cam_yaw_x10: left/right projection tuning",
            "- cam_z_cm: up/down projection tuning",
            "- lidar_yaw_deg+180: LiDAR top-view direction",
            "",
            "Recommended target:",
            "- Put a flat board/person at 1.0~2.0m",
            "- Make projected points green",
            "- Minimize median_abs_error",
            "",
            f"Current cam pitch: {self.camera_pitch_deg:.1f} deg",
            f"Current cam yaw  : {self.camera_yaw_deg:.1f} deg",
            f"Current cam z    : {self.camera_base_z:.2f} m",
            f"Current lidar yaw: {self.lidar_yaw_deg:.1f} deg",
        ]

        y = 28
        for line in lines:
            cv2.putText(panel, line, (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                        (230, 230, 230), 1, cv2.LINE_AA)
            y += 22

        return panel

    def make_top_view(self, lidar_points_base, errors):
        size = 800
        canvas = np.zeros((size, size, 3), dtype=np.uint8)
        canvas[:] = (20, 20, 20)

        origin_x = size // 2
        origin_y = int(size * 0.82)
        meter_to_pixel = 90.0

        self.draw_top_grid(canvas, origin_x, origin_y, meter_to_pixel)

        if lidar_points_base is not None:
            pts = lidar_points_base
            if len(pts) > 5000:
                idx = np.random.choice(len(pts), 5000, replace=False)
                pts = pts[idx]

            for p in pts:
                px = int(origin_x - p[1] * meter_to_pixel)
                py = int(origin_y - p[0] * meter_to_pixel)
                if 0 <= px < size and 0 <= py < size:
                    cv2.circle(canvas, (px, py), 1, (80, 80, 80), -1)

        # Draw compared points by error color
        if errors is not None:
            pts = errors["points_base"]
            err = errors["error"]
            abs_err = errors["abs_error"]

            for i in range(len(pts)):
                x = pts[i, 0]
                y = pts[i, 1]

                px = int(origin_x - y * meter_to_pixel)
                py = int(origin_y - x * meter_to_pixel)

                if not (0 <= px < size and 0 <= py < size):
                    continue

                if abs_err[i] < self.good_error_threshold:
                    color = (0, 255, 0)
                elif err[i] > 0:
                    color = (255, 0, 0)
                else:
                    color = (0, 0, 255)

                cv2.circle(canvas, (px, py), 3, color, -1)

        # Robot
        cv2.rectangle(canvas, (origin_x - 24, origin_y - 35), (origin_x + 24, origin_y + 35), (80, 160, 255), -1)
        cv2.rectangle(canvas, (origin_x - 24, origin_y - 35), (origin_x + 24, origin_y + 35), (255, 255, 255), 2)
        cv2.arrowedLine(canvas, (origin_x, origin_y), (origin_x, origin_y - 70), (0, 255, 255), 3)

        # Sensors
        cam_px = int(origin_x - self.camera_base_y * meter_to_pixel)
        cam_py = int(origin_y - self.camera_base_x * meter_to_pixel)
        lidar_px = int(origin_x - self.lidar_base_y * meter_to_pixel)
        lidar_py = int(origin_y - self.lidar_base_x * meter_to_pixel)

        cv2.circle(canvas, (cam_px, cam_py), 7, (0, 220, 255), -1)
        cv2.putText(canvas, "D435i", (cam_px + 8, cam_py), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 1)

        cv2.circle(canvas, (lidar_px, lidar_py), 7, (255, 120, 0), -1)
        cv2.putText(canvas, "XT16", (lidar_px + 8, lidar_py), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 160, 80), 1)

        cv2.putText(canvas, "Top View: Gray LiDAR | Green Good | Blue Farther | Red Closer",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)

        return canvas

    def draw_top_grid(self, canvas, origin_x, origin_y, meter_to_pixel):
        size = canvas.shape[0]
        grid_color = (55, 55, 55)
        axis_color = (180, 180, 180)

        for m in np.arange(0.0, 6.1, 0.5):
            y = int(origin_y - m * meter_to_pixel)
            cv2.line(canvas, (origin_x - int(3 * meter_to_pixel), y),
                     (origin_x + int(3 * meter_to_pixel), y), grid_color, 1)
            cv2.putText(canvas, f"{m:.1f}m", (origin_x + int(3 * meter_to_pixel) + 8, y + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)

        for m in np.arange(-3.0, 3.1, 0.5):
            x = int(origin_x - m * meter_to_pixel)
            cv2.line(canvas, (x, origin_y), (x, origin_y - int(6 * meter_to_pixel)), grid_color, 1)

        cv2.line(canvas, (origin_x, origin_y), (origin_x, origin_y - int(6 * meter_to_pixel)), axis_color, 2)
        cv2.line(canvas, (origin_x - int(3 * meter_to_pixel), origin_y),
                 (origin_x + int(3 * meter_to_pixel), origin_y), axis_color, 2)

        cv2.putText(canvas, "+X Forward", (origin_x + 15, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        cv2.putText(canvas, "+Y Left", (30, origin_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        cv2.putText(canvas, "-Y Right", (size - 130, origin_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

    # ---------------------------------------------------------
    # Rotation / image mapping
    # ---------------------------------------------------------
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

    def rotate_depth(self, depth):
        return self.rotate_image(depth)

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

    # ---------------------------------------------------------
    # Math
    # ---------------------------------------------------------
    @staticmethod
    def euler_to_rot(roll, pitch, yaw):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)

        Rx = np.array([
            [1, 0, 0],
            [0, cr, -sr],
            [0, sr, cr]
        ], dtype=np.float32)

        Ry = np.array([
            [cp, 0, sp],
            [0, 1, 0],
            [-sp, 0, cp]
        ], dtype=np.float32)

        Rz = np.array([
            [cy, -sy, 0],
            [sy, cy, 0],
            [0, 0, 1]
        ], dtype=np.float32)

        return Rz @ Ry @ Rx

    # ---------------------------------------------------------
    # Status / cleanup
    # ---------------------------------------------------------
    def print_status(self):
        now = time.time()
        if now - self.last_status_time < 2.0:
            return

        self.last_status_time = now

        stats = self.last_stats
        if stats.get("matched", 0) > 0:
            print(
                f"[STATUS] matched={stats['matched']} | "
                f"median_abs={stats['median_abs_error']:.3f}m | "
                f"mean_abs={stats['mean_abs_error']:.3f}m | "
                f"med_err={stats['median_error']:.3f}m | "
                f"cam_pitch={self.camera_pitch_deg:.1f} | "
                f"cam_yaw={self.camera_yaw_deg:.1f} | "
                f"cam_z={self.camera_base_z:.2f} | "
                f"lidar_yaw={self.lidar_yaw_deg:.1f}",
                flush=True
            )
        else:
            print(
                f"[STATUS] No matched depth points | "
                f"cam_pitch={self.camera_pitch_deg:.1f} | "
                f"cam_yaw={self.camera_yaw_deg:.1f} | "
                f"cam_z={self.camera_base_z:.2f} | "
                f"lidar_yaw={self.lidar_yaw_deg:.1f}",
                flush=True
            )

    def print_current_params(self):
        print("\n========== CURRENT CALIBRATION PARAMETERS ==========", flush=True)
        print(f"--image-rotate {self.image_rotate}", flush=True)
        print(f"--camera-base-x {self.camera_base_x:.3f}", flush=True)
        print(f"--camera-base-y {self.camera_base_y:.3f}", flush=True)
        print(f"--camera-base-z {self.camera_base_z:.3f}", flush=True)
        print(f"--camera-roll-deg {self.camera_roll_deg:.2f}", flush=True)
        print(f"--camera-pitch-deg {self.camera_pitch_deg:.2f}", flush=True)
        print(f"--camera-yaw-deg {self.camera_yaw_deg:.2f}", flush=True)
        print(f"--lidar-base-x {self.lidar_base_x:.3f}", flush=True)
        print(f"--lidar-base-y {self.lidar_base_y:.3f}", flush=True)
        print(f"--lidar-base-z {self.lidar_base_z:.3f}", flush=True)
        print(f"--lidar-roll-deg {self.lidar_roll_deg:.2f}", flush=True)
        print(f"--lidar-pitch-deg {self.lidar_pitch_deg:.2f}", flush=True)
        print(f"--lidar-yaw-deg {self.lidar_yaw_deg:.2f}", flush=True)
        print("====================================================\n", flush=True)

    def stop(self):
        if getattr(self, "pipeline_started", False):
            print("[INFO] Stopping RealSense pipeline...", flush=True)
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


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)

    parser.add_argument(
        "--image-rotate",
        default="none",
        choices=["none", "cw90", "ccw90", "180"]
    )

    parser.add_argument("--lidar-topic", default="/lidar_points")
    parser.add_argument(
        "--lidar-points-frame",
        default="lidar",
        choices=["lidar", "base"]
    )

    # Initial values based on your mounting description
    parser.add_argument("--camera-base-x", type=float, default=0.38)
    parser.add_argument("--camera-base-y", type=float, default=0.00)
    parser.add_argument("--camera-base-z", type=float, default=0.12)

    parser.add_argument("--camera-roll-deg", type=float, default=0.0)
    parser.add_argument("--camera-pitch-deg", type=float, default=0.0)
    parser.add_argument("--camera-yaw-deg", type=float, default=0.0)

    parser.add_argument("--lidar-base-x", type=float, default=0.15)
    parser.add_argument("--lidar-base-y", type=float, default=0.00)
    parser.add_argument("--lidar-base-z", type=float, default=0.25)

    parser.add_argument("--lidar-roll-deg", type=float, default=0.0)
    parser.add_argument("--lidar-pitch-deg", type=float, default=0.0)
    parser.add_argument("--lidar-yaw-deg", type=float, default=90.0)

    parser.add_argument("--lidar-x-min", type=float, default=0.20)
    parser.add_argument("--lidar-x-max", type=float, default=8.00)
    parser.add_argument("--lidar-y-min", type=float, default=-4.00)
    parser.add_argument("--lidar-y-max", type=float, default=4.00)
    parser.add_argument("--lidar-z-min", type=float, default=-1.20)
    parser.add_argument("--lidar-z-max", type=float, default=2.50)

    parser.add_argument("--max-lidar-points", type=int, default=40000)
    parser.add_argument("--max-compare-points", type=int, default=8000)

    parser.add_argument("--depth-min", type=float, default=0.20)
    parser.add_argument("--depth-max", type=float, default=6.00)
    parser.add_argument("--depth-sample-radius", type=int, default=2)
    parser.add_argument("--good-error-threshold", type=float, default=0.15)

    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init(args=None)

    app = None

    try:
        app = RealSenseLidarDepthCalib(args)
        app.run()

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt caught.", flush=True)

    except Exception as e:
        print(f"[ERROR] Fatal error: {e}", flush=True)

    finally:
        if app is not None:
            app.running = False
            app.stop()
            app.destroy_node()

        try:
            rclpy.shutdown()
        except Exception:
            pass

        print("[INFO] Program finished.", flush=True)


if __name__ == "__main__":
    main()

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
import open3d as o3d

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class RealSenseLidar3DCalib(Node):
    """
    RealSense depth point cloud and Hesai LiDAR point cloud 3D calibration viewer.

    Coordinate convention:
        base x: forward
        base y: left
        base z: up

    RealSense optical frame:
        X: right
        Y: down
        Z: forward

    Camera body frame:
        x: forward
        y: left
        z: up
    """

    def __init__(self, args):
        super().__init__("go2_realsense_lidar_3d_calib")

        self.args = args
        self.running = True
        self.pipeline_started = False

        # -----------------------------
        # RealSense settings
        # -----------------------------
        self.width = args.width
        self.height = args.height
        self.camera_fps = args.camera_fps
        self.depth_min = args.depth_min
        self.depth_max = args.depth_max
        self.rs_stride = args.rs_stride

        self.pipeline = None
        self.config = None
        self.profile = None
        self.align = None
        self.depth_scale = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # -----------------------------
        # LiDAR settings
        # -----------------------------
        self.lidar_topic = args.lidar_topic
        self.lidar_points_frame = args.lidar_points_frame

        self.latest_lidar_points_base = None
        self.lidar_lock = threading.Lock()

        # -----------------------------
        # Default mounting values
        # These are based on your mounting description:
        # RealSense D435i: robot dog head, above front camera
        # Hesai XT16: behind adjoint module, upper body center
        # -----------------------------
        self.default_camera_base_x = args.camera_base_x
        self.default_camera_base_y = args.camera_base_y
        self.default_camera_base_z = args.camera_base_z
        self.default_camera_roll_deg = args.camera_roll_deg
        self.default_camera_pitch_deg = args.camera_pitch_deg
        self.default_camera_yaw_deg = args.camera_yaw_deg

        self.default_lidar_base_x = args.lidar_base_x
        self.default_lidar_base_y = args.lidar_base_y
        self.default_lidar_base_z = args.lidar_base_z
        self.default_lidar_roll_deg = args.lidar_roll_deg
        self.default_lidar_pitch_deg = args.lidar_pitch_deg
        self.default_lidar_yaw_deg = args.lidar_yaw_deg

        # Current extrinsics
        self.camera_base_x = self.default_camera_base_x
        self.camera_base_y = self.default_camera_base_y
        self.camera_base_z = self.default_camera_base_z
        self.camera_roll_deg = self.default_camera_roll_deg
        self.camera_pitch_deg = self.default_camera_pitch_deg
        self.camera_yaw_deg = self.default_camera_yaw_deg

        self.lidar_base_x = self.default_lidar_base_x
        self.lidar_base_y = self.default_lidar_base_y
        self.lidar_base_z = self.default_lidar_base_z
        self.lidar_roll_deg = self.default_lidar_roll_deg
        self.lidar_pitch_deg = self.default_lidar_pitch_deg
        self.lidar_yaw_deg = self.default_lidar_yaw_deg

        self.camera_t_base = None
        self.lidar_t_base = None
        self.R_base_camera_body = None
        self.R_base_lidar = None
        self.update_extrinsics()

        # -----------------------------
        # ROI
        # -----------------------------
        self.lidar_x_min = args.lidar_x_min
        self.lidar_x_max = args.lidar_x_max
        self.lidar_y_min = args.lidar_y_min
        self.lidar_y_max = args.lidar_y_max
        self.lidar_z_min = args.lidar_z_min
        self.lidar_z_max = args.lidar_z_max

        self.max_lidar_points = args.max_lidar_points
        self.max_rs_points = args.max_rs_points

        # -----------------------------
        # Open3D geometry
        # -----------------------------
        self.vis = None
        self.pcd_rs = o3d.geometry.PointCloud()
        self.pcd_lidar = o3d.geometry.PointCloud()
        self.robot_frame = None
        self.camera_frame = None
        self.lidar_frame = None
        self.grid = None

        self.last_status_time = time.time()

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        atexit.register(self.stop)

        self.create_lidar_subscriber()
        self.start_realsense()
        self.create_control_window()
        self.create_open3d_window()

        print("[INFO] 3D calibration viewer started.", flush=True)
        print("[INFO] RealSense depth cloud: green.", flush=True)
        print("[INFO] Hesai LiDAR cloud: red/orange.", flush=True)
        print("[INFO] Adjust trackbars until both clouds overlap in 3D.", flush=True)
        print("[INFO] Keys: s=print params, r=reset, q/ESC=quit.", flush=True)

    # =========================================================
    # Setup
    # =========================================================
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

        self.fx = float(intr.fx)
        self.fy = float(intr.fy)
        self.cx = float(intr.ppx)
        self.cy = float(intr.ppy)

        print(
            f"[INFO] RealSense intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, "
            f"cx={self.cx:.2f}, cy={self.cy:.2f}",
            flush=True
        )
        print(f"[INFO] Depth scale: {self.depth_scale}", flush=True)

        warmup_start = time.time()
        while time.time() - warmup_start < 0.5:
            self.pipeline.poll_for_frames()
            time.sleep(0.01)

    def create_control_window(self):
        cv2.namedWindow("3D Calibration Controls", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("3D Calibration Controls", 760, 620)
        cv2.moveWindow("3D Calibration Controls", 30, 40)

        blank = np.zeros((620, 760, 3), dtype=np.uint8)
        cv2.imshow("3D Calibration Controls", blank)

        # Camera rotation: -45.0 ~ +45.0 deg
        cv2.createTrackbar(
            "cam_roll_x10+450",
            "3D Calibration Controls",
            int(self.default_camera_roll_deg * 10 + 450),
            900,
            self.on_trackbar
        )
        cv2.createTrackbar(
            "cam_pitch_x10+450",
            "3D Calibration Controls",
            int(self.default_camera_pitch_deg * 10 + 450),
            900,
            self.on_trackbar
        )
        cv2.createTrackbar(
            "cam_yaw_x10+450",
            "3D Calibration Controls",
            int(self.default_camera_yaw_deg * 10 + 450),
            900,
            self.on_trackbar
        )

        # Camera position offset: -30cm ~ +30cm from default
        cv2.createTrackbar("cam_dx_cm+30", "3D Calibration Controls", 30, 60, self.on_trackbar)
        cv2.createTrackbar("cam_dy_cm+30", "3D Calibration Controls", 30, 60, self.on_trackbar)
        cv2.createTrackbar("cam_dz_cm+30", "3D Calibration Controls", 30, 60, self.on_trackbar)

        # LiDAR rotation: -180.0 ~ +180.0 deg
        cv2.createTrackbar(
            "lidar_roll_x10+1800",
            "3D Calibration Controls",
            int(self.default_lidar_roll_deg * 10 + 1800),
            3600,
            self.on_trackbar
        )
        cv2.createTrackbar(
            "lidar_pitch_x10+1800",
            "3D Calibration Controls",
            int(self.default_lidar_pitch_deg * 10 + 1800),
            3600,
            self.on_trackbar
        )
        cv2.createTrackbar(
            "lidar_yaw_x10+1800",
            "3D Calibration Controls",
            int(self.default_lidar_yaw_deg * 10 + 1800),
            3600,
            self.on_trackbar
        )

        # LiDAR position offset: -30cm ~ +30cm from default
        cv2.createTrackbar("lidar_dx_cm+30", "3D Calibration Controls", 30, 60, self.on_trackbar)
        cv2.createTrackbar("lidar_dy_cm+30", "3D Calibration Controls", 30, 60, self.on_trackbar)
        cv2.createTrackbar("lidar_dz_cm+30", "3D Calibration Controls", 30, 60, self.on_trackbar)

        cv2.waitKey(1)

    def create_open3d_window(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(
            window_name="RealSense Depth Cloud vs Hesai LiDAR Cloud - 3D Calibration",
            width=1280,
            height=800,
            left=820,
            top=60
        )

        self.pcd_rs.points = o3d.utility.Vector3dVector(np.zeros((1, 3)))
        self.pcd_rs.colors = o3d.utility.Vector3dVector(np.array([[0.0, 0.9, 0.3]]))

        self.pcd_lidar.points = o3d.utility.Vector3dVector(np.zeros((1, 3)))
        self.pcd_lidar.colors = o3d.utility.Vector3dVector(np.array([[1.0, 0.25, 0.0]]))

        self.robot_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.45,
            origin=[0.0, 0.0, 0.0]
        )

        self.camera_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.25,
            origin=self.camera_t_base.tolist()
        )

        self.lidar_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.25,
            origin=self.lidar_t_base.tolist()
        )

        self.grid = self.create_ground_grid(size=8.0, step=0.5)

        self.vis.add_geometry(self.pcd_rs)
        self.vis.add_geometry(self.pcd_lidar)
        self.vis.add_geometry(self.robot_frame)
        self.vis.add_geometry(self.camera_frame)
        self.vis.add_geometry(self.lidar_frame)
        self.vis.add_geometry(self.grid)

        opt = self.vis.get_render_option()
        opt.background_color = np.asarray([0.02, 0.02, 0.02])
        opt.point_size = 2.0

        view = self.vis.get_view_control()
        view.set_front([0.0, -1.0, 0.55])
        view.set_lookat([1.5, 0.0, 0.5])
        view.set_up([0.0, 0.0, 1.0])
        view.set_zoom(0.55)

    # =========================================================
    # Trackbar
    # =========================================================
    def on_trackbar(self, value):
        self.read_trackbars()
        self.update_extrinsics()

    def read_trackbars(self):
        try:
            self.camera_roll_deg = (
                cv2.getTrackbarPos("cam_roll_x10+450", "3D Calibration Controls") - 450
            ) / 10.0

            self.camera_pitch_deg = (
                cv2.getTrackbarPos("cam_pitch_x10+450", "3D Calibration Controls") - 450
            ) / 10.0

            self.camera_yaw_deg = (
                cv2.getTrackbarPos("cam_yaw_x10+450", "3D Calibration Controls") - 450
            ) / 10.0

            cam_dx = (
                cv2.getTrackbarPos("cam_dx_cm+30", "3D Calibration Controls") - 30
            ) / 100.0
            cam_dy = (
                cv2.getTrackbarPos("cam_dy_cm+30", "3D Calibration Controls") - 30
            ) / 100.0
            cam_dz = (
                cv2.getTrackbarPos("cam_dz_cm+30", "3D Calibration Controls") - 30
            ) / 100.0

            self.camera_base_x = self.default_camera_base_x + cam_dx
            self.camera_base_y = self.default_camera_base_y + cam_dy
            self.camera_base_z = self.default_camera_base_z + cam_dz

            self.lidar_roll_deg = (
                cv2.getTrackbarPos("lidar_roll_x10+1800", "3D Calibration Controls") - 1800
            ) / 10.0

            self.lidar_pitch_deg = (
                cv2.getTrackbarPos("lidar_pitch_x10+1800", "3D Calibration Controls") - 1800
            ) / 10.0

            self.lidar_yaw_deg = (
                cv2.getTrackbarPos("lidar_yaw_x10+1800", "3D Calibration Controls") - 1800
            ) / 10.0

            lidar_dx = (
                cv2.getTrackbarPos("lidar_dx_cm+30", "3D Calibration Controls") - 30
            ) / 100.0
            lidar_dy = (
                cv2.getTrackbarPos("lidar_dy_cm+30", "3D Calibration Controls") - 30
            ) / 100.0
            lidar_dz = (
                cv2.getTrackbarPos("lidar_dz_cm+30", "3D Calibration Controls") - 30
            ) / 100.0

            self.lidar_base_x = self.default_lidar_base_x + lidar_dx
            self.lidar_base_y = self.default_lidar_base_y + lidar_dy
            self.lidar_base_z = self.default_lidar_base_z + lidar_dz

        except Exception:
            pass

    def reset_trackbars_to_default(self):
        print("[INFO] Resetting calibration trackbars to default mounting values.", flush=True)

        try:
            cv2.setTrackbarPos(
                "cam_roll_x10+450",
                "3D Calibration Controls",
                int(self.default_camera_roll_deg * 10 + 450)
            )
            cv2.setTrackbarPos(
                "cam_pitch_x10+450",
                "3D Calibration Controls",
                int(self.default_camera_pitch_deg * 10 + 450)
            )
            cv2.setTrackbarPos(
                "cam_yaw_x10+450",
                "3D Calibration Controls",
                int(self.default_camera_yaw_deg * 10 + 450)
            )

            cv2.setTrackbarPos("cam_dx_cm+30", "3D Calibration Controls", 30)
            cv2.setTrackbarPos("cam_dy_cm+30", "3D Calibration Controls", 30)
            cv2.setTrackbarPos("cam_dz_cm+30", "3D Calibration Controls", 30)

            cv2.setTrackbarPos(
                "lidar_roll_x10+1800",
                "3D Calibration Controls",
                int(self.default_lidar_roll_deg * 10 + 1800)
            )
            cv2.setTrackbarPos(
                "lidar_pitch_x10+1800",
                "3D Calibration Controls",
                int(self.default_lidar_pitch_deg * 10 + 1800)
            )
            cv2.setTrackbarPos(
                "lidar_yaw_x10+1800",
                "3D Calibration Controls",
                int(self.default_lidar_yaw_deg * 10 + 1800)
            )

            cv2.setTrackbarPos("lidar_dx_cm+30", "3D Calibration Controls", 30)
            cv2.setTrackbarPos("lidar_dy_cm+30", "3D Calibration Controls", 30)
            cv2.setTrackbarPos("lidar_dz_cm+30", "3D Calibration Controls", 30)

        except Exception as e:
            print(f"[WARN] Failed to reset trackbars: {e}", flush=True)

        self.read_trackbars()
        self.update_extrinsics()

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

    # =========================================================
    # Main loop
    # =========================================================
    def run(self):
        try:
            while self.running:
                rclpy.spin_once(self, timeout_sec=0.0)

                self.read_trackbars()
                self.update_extrinsics()

                rs_points_base = self.get_realsense_depth_cloud_base()
                lidar_points_base = self.get_latest_lidar_points_base_copy()

                if rs_points_base is not None:
                    rs_points_base = self.downsample_points(rs_points_base, self.max_rs_points)

                if lidar_points_base is not None:
                    lidar_points_base = self.downsample_points(lidar_points_base, self.max_lidar_points)

                self.update_open3d(rs_points_base, lidar_points_base)
                self.update_control_panel(rs_points_base, lidar_points_base)

                key = cv2.waitKey(1) & 0xFF

                if key == ord("q") or key == 27:
                    self.running = False

                if key == ord("s"):
                    self.print_current_params()

                if key == ord("r"):
                    self.reset_trackbars_to_default()

                self.print_status(rs_points_base, lidar_points_base)

        except KeyboardInterrupt:
            self.running = False

        finally:
            self.stop()

    # =========================================================
    # RealSense depth cloud
    # =========================================================
    def get_realsense_depth_cloud_base(self):
        if not self.pipeline_started:
            return None

        try:
            frames = self.pipeline.poll_for_frames()

            if not frames:
                return None

            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()

            if not depth_frame:
                return None

            depth_raw = np.asanyarray(depth_frame.get_data()).astype(np.float32)
            depth_m = depth_raw * self.depth_scale

            return self.depth_image_to_base_points(depth_m)

        except Exception as e:
            print(f"[ERROR] RealSense depth cloud error: {e}", flush=True)
            return None

    def depth_image_to_base_points(self, depth_m):
        h, w = depth_m.shape[:2]
        stride = max(1, int(self.rs_stride))

        ys = np.arange(0, h, stride)
        xs = np.arange(0, w, stride)
        grid_x, grid_y = np.meshgrid(xs, ys)

        z = depth_m[grid_y, grid_x]

        valid = (
            (z > self.depth_min) &
            (z < self.depth_max) &
            np.isfinite(z)
        )

        if np.count_nonzero(valid) == 0:
            return None

        u = grid_x[valid].astype(np.float32)
        v = grid_y[valid].astype(np.float32)
        z_opt = z[valid].astype(np.float32)

        # RealSense optical frame
        X_opt = (u - self.cx) * z_opt / self.fx
        Y_opt = (v - self.cy) * z_opt / self.fy
        Z_opt = z_opt

        # optical -> camera body
        x_body = Z_opt
        y_body = -X_opt
        z_body = -Y_opt

        p_body = np.stack((x_body, y_body, z_body), axis=1).astype(np.float32)

        # camera body -> base
        p_base = p_body @ self.R_base_camera_body.T + self.camera_t_base.reshape(1, 3)

        roi = (
            (p_base[:, 0] >= self.lidar_x_min) &
            (p_base[:, 0] <= self.lidar_x_max) &
            (p_base[:, 1] >= self.lidar_y_min) &
            (p_base[:, 1] <= self.lidar_y_max) &
            (p_base[:, 2] >= self.lidar_z_min) &
            (p_base[:, 2] <= self.lidar_z_max)
        )

        p_base = p_base[roi]

        if len(p_base) == 0:
            return None

        return p_base

    # =========================================================
    # LiDAR handling
    # =========================================================
    def lidar_callback(self, msg):
        raw = self.pointcloud2_to_xyz_numpy(msg)

        if raw is None or len(raw) == 0:
            return

        base_points = self.lidar_raw_to_base(raw)
        base_points = self.apply_lidar_roi(base_points)

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

    # =========================================================
    # Open3D update
    # =========================================================
    def update_open3d(self, rs_points_base, lidar_points_base):
        if rs_points_base is not None and len(rs_points_base) > 0:
            rs_colors = np.zeros_like(rs_points_base)
            rs_colors[:, 0] = 0.0
            rs_colors[:, 1] = 0.85
            rs_colors[:, 2] = 0.35

            self.pcd_rs.points = o3d.utility.Vector3dVector(rs_points_base.astype(np.float64))
            self.pcd_rs.colors = o3d.utility.Vector3dVector(rs_colors.astype(np.float64))

        if lidar_points_base is not None and len(lidar_points_base) > 0:
            lidar_colors = np.zeros_like(lidar_points_base)
            lidar_colors[:, 0] = 1.0
            lidar_colors[:, 1] = 0.25
            lidar_colors[:, 2] = 0.0

            self.pcd_lidar.points = o3d.utility.Vector3dVector(lidar_points_base.astype(np.float64))
            self.pcd_lidar.colors = o3d.utility.Vector3dVector(lidar_colors.astype(np.float64))

        self.vis.remove_geometry(self.camera_frame, reset_bounding_box=False)
        self.vis.remove_geometry(self.lidar_frame, reset_bounding_box=False)

        self.camera_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.25,
            origin=self.camera_t_base.tolist()
        )

        self.lidar_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.25,
            origin=self.lidar_t_base.tolist()
        )

        self.vis.add_geometry(self.camera_frame, reset_bounding_box=False)
        self.vis.add_geometry(self.lidar_frame, reset_bounding_box=False)

        self.vis.update_geometry(self.pcd_rs)
        self.vis.update_geometry(self.pcd_lidar)
        self.vis.poll_events()
        self.vis.update_renderer()

    # =========================================================
    # UI / logging
    # =========================================================
    def update_control_panel(self, rs_points_base, lidar_points_base):
        panel = np.zeros((620, 760, 3), dtype=np.uint8)
        panel[:] = (28, 28, 28)

        rs_n = 0 if rs_points_base is None else len(rs_points_base)
        lidar_n = 0 if lidar_points_base is None else len(lidar_points_base)

        lines = [
            "3D Calibration Controls",
            "",
            "Goal:",
            "Overlap RealSense depth cloud and Hesai LiDAR cloud in 3D.",
            "",
            "Colors:",
            "RealSense depth cloud : Green",
            "Hesai XT16 LiDAR cloud: Red/Orange",
            "",
            f"RS points    : {rs_n}",
            f"LiDAR points : {lidar_n}",
            "",
            "Current Extrinsics:",
            f"Cam xyz   : ({self.camera_base_x:.3f}, {self.camera_base_y:.3f}, {self.camera_base_z:.3f}) m",
            f"Cam rpy   : ({self.camera_roll_deg:.1f}, {self.camera_pitch_deg:.1f}, {self.camera_yaw_deg:.1f}) deg",
            f"LiDAR xyz : ({self.lidar_base_x:.3f}, {self.lidar_base_y:.3f}, {self.lidar_base_z:.3f}) m",
            f"LiDAR rpy : ({self.lidar_roll_deg:.1f}, {self.lidar_pitch_deg:.1f}, {self.lidar_yaw_deg:.1f}) deg",
            "",
            "Recommended tuning order:",
            "1. LiDAR yaw/roll/pitch",
            "2. Camera pitch",
            "3. Camera yaw",
            "4. Camera z, x, y offsets",
            "5. LiDAR z, x, y offsets",
            "",
            "Keys:",
            "s: print current parameters",
            "r: reset to default mounting values",
            "q/ESC: quit",
        ]

        y = 26
        for line in lines:
            cv2.putText(
                panel,
                line,
                (16, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (235, 235, 235),
                1,
                cv2.LINE_AA
            )
            y += 21

        cv2.imshow("3D Calibration Controls", panel)

    def print_status(self, rs_points_base, lidar_points_base):
        now = time.time()
        if now - self.last_status_time < 2.0:
            return

        self.last_status_time = now

        rs_n = 0 if rs_points_base is None else len(rs_points_base)
        lidar_n = 0 if lidar_points_base is None else len(lidar_points_base)

        print(
            f"[STATUS] rs_points={rs_n} | lidar_points={lidar_n} | "
            f"cam_xyz=({self.camera_base_x:.3f},{self.camera_base_y:.3f},{self.camera_base_z:.3f}) | "
            f"cam_rpy=({self.camera_roll_deg:.1f},{self.camera_pitch_deg:.1f},{self.camera_yaw_deg:.1f}) | "
            f"lidar_xyz=({self.lidar_base_x:.3f},{self.lidar_base_y:.3f},{self.lidar_base_z:.3f}) | "
            f"lidar_rpy=({self.lidar_roll_deg:.1f},{self.lidar_pitch_deg:.1f},{self.lidar_yaw_deg:.1f})",
            flush=True
        )

    def print_current_params(self):
        print("\n========== CURRENT 3D CALIBRATION PARAMETERS ==========", flush=True)
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
        print("========================================================\n", flush=True)

    # =========================================================
    # Utils
    # =========================================================
    @staticmethod
    def downsample_points(points, max_points):
        if points is None:
            return None

        if len(points) <= max_points:
            return points

        idx = np.random.choice(len(points), max_points, replace=False)
        return points[idx]

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

    @staticmethod
    def create_ground_grid(size=8.0, step=0.5):
        points = []
        lines = []
        colors = []

        idx = 0
        x_min = 0.0
        x_max = size
        y_min = -size / 2.0
        y_max = size / 2.0

        x_values = np.arange(x_min, x_max + 0.001, step)
        y_values = np.arange(y_min, y_max + 0.001, step)

        for x in x_values:
            points.append([x, y_min, 0.0])
            points.append([x, y_max, 0.0])
            lines.append([idx, idx + 1])
            colors.append([0.25, 0.25, 0.25])
            idx += 2

        for y in y_values:
            points.append([x_min, y, 0.0])
            points.append([x_max, y, 0.0])
            lines.append([idx, idx + 1])
            colors.append([0.25, 0.25, 0.25])
            idx += 2

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(np.array(points))
        line_set.lines = o3d.utility.Vector2iVector(np.array(lines))
        line_set.colors = o3d.utility.Vector3dVector(np.array(colors))
        return line_set

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

        try:
            if self.vis is not None:
                self.vis.destroy_window()
        except Exception:
            pass


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)

    parser.add_argument("--lidar-topic", default="/lidar_points")
    parser.add_argument(
        "--lidar-points-frame",
        default="lidar",
        choices=["lidar", "base"]
    )

    # Default mounting estimate
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
    parser.add_argument("--lidar-yaw-deg", type=float, default=-180.0)

    # ROI
    parser.add_argument("--lidar-x-min", type=float, default=0.10)
    parser.add_argument("--lidar-x-max", type=float, default=8.00)
    parser.add_argument("--lidar-y-min", type=float, default=-4.00)
    parser.add_argument("--lidar-y-max", type=float, default=4.00)
    parser.add_argument("--lidar-z-min", type=float, default=-1.50)
    parser.add_argument("--lidar-z-max", type=float, default=3.00)

    # RealSense depth cloud
    parser.add_argument("--depth-min", type=float, default=0.20)
    parser.add_argument("--depth-max", type=float, default=6.00)
    parser.add_argument("--rs-stride", type=int, default=4)

    # Downsampling
    parser.add_argument("--max-rs-points", type=int, default=30000)
    parser.add_argument("--max-lidar-points", type=int, default=30000)

    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init(args=None)

    app = None

    try:
        app = RealSenseLidar3DCalib(args)
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

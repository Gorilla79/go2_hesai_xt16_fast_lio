#!/usr/bin/env python3

import argparse
import math
import time
import signal
import atexit
import threading

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class Go2YoloLidarFusionV2(Node):
    """
    RealSense D435i RGB + YOLO11s-seg + Hesai XT16 /lidar_points fusion.

    Robot base coordinate:
        x: forward
        y: left
        z: up

    RealSense:
        Mounted on robot dog head, above the front camera.

    Hesai XT16:
        Mounted behind the adjoint module on the upper center body.
    """

    def __init__(self, args):
        super().__init__("go2_yolo11s_seg_lidar_topview_v2")

        self.args = args
        self.running = True
        self.pipeline_started = False

        # ----------------------------
        # Model / camera
        # ----------------------------
        self.model_path = args.model
        self.width = args.width
        self.height = args.height
        self.camera_fps = args.camera_fps
        self.image_rotate = args.image_rotate

        self.imgsz = args.imgsz
        self.conf = args.conf
        self.iou = args.iou
        self.device = args.device
        self.use_half = args.half
        self.target_fps = args.target_fps

        # ----------------------------
        # LiDAR
        # ----------------------------
        self.lidar_topic = args.lidar_topic
        self.lidar_points_frame = args.lidar_points_frame

        self.latest_lidar_points_base = None
        self.lidar_lock = threading.Lock()

        # ----------------------------
        # Sensor positions from provided mounting images
        # ----------------------------
        self.camera_t_base = np.array(
            [args.camera_base_x, args.camera_base_y, args.camera_base_z],
            dtype=np.float32
        )

        self.lidar_t_base = np.array(
            [args.lidar_base_x, args.lidar_base_y, args.lidar_base_z],
            dtype=np.float32
        )

        self.R_base_camera_body = self.euler_to_rot(
            math.radians(args.camera_roll_deg),
            math.radians(args.camera_pitch_deg),
            math.radians(args.camera_yaw_deg)
        )

        self.R_base_lidar = self.euler_to_rot(
            math.radians(args.lidar_roll_deg),
            math.radians(args.lidar_pitch_deg),
            math.radians(args.lidar_yaw_deg)
        )

        # ----------------------------
        # LiDAR ROI in base frame
        # ----------------------------
        self.lidar_x_min = args.lidar_x_min
        self.lidar_x_max = args.lidar_x_max
        self.lidar_y_min = args.lidar_y_min
        self.lidar_y_max = args.lidar_y_max
        self.lidar_z_min = args.lidar_z_min
        self.lidar_z_max = args.lidar_z_max
        self.max_lidar_points = args.max_lidar_points

        # ----------------------------
        # Person association
        # ----------------------------
        self.min_lidar_points_in_mask = args.min_lidar_points_in_mask
        self.mask_dilate_px = args.mask_dilate_px
        self.person_outlier_radius = args.person_outlier_radius
        self.fixed_distance = args.fixed_distance

        # ----------------------------
        # Visualization
        # ----------------------------
        self.show_windows = not args.no_gui
        self.draw_lidar_points = args.draw_lidar_points
        self.draw_projected_lidar = args.draw_projected_lidar

        self.topview_size = args.topview_size
        self.meter_to_pixel = args.meter_to_pixel
        self.max_forward_m = args.max_forward_m
        self.max_side_m = args.max_side_m

        # ----------------------------
        # RealSense intrinsics
        # ----------------------------
        self.raw_fx = None
        self.raw_fy = None
        self.raw_cx = None
        self.raw_cy = None
        self.raw_w = None
        self.raw_h = None

        # Runtime state
        self.pipeline = None
        self.config = None
        self.profile = None

        self.last_infer_time = 0.0
        self.infer_times = []
        self.last_log_time = time.time()

        # Safe exit
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        atexit.register(self.stop)

        print(f"[INFO] Loading YOLO model once: {self.model_path}", flush=True)
        self.model = YOLO(self.model_path)
        print("[INFO] YOLO model loaded.", flush=True)

        self.create_lidar_subscriber()
        self.start_realsense()

        if self.show_windows:
            self.create_windows()

        print("[INFO] Modified V2 started.", flush=True)
        print("[INFO] RealSense position: robot dog head, above front camera.", flush=True)
        print("[INFO] Hesai XT16 position: behind adjoint module, upper body center.", flush=True)
        print("[INFO] Base coordinate: x forward, y left, z up.", flush=True)
        print(f"[INFO] image_rotate={self.image_rotate}", flush=True)
        print(f"[INFO] lidar_points_frame={self.lidar_points_frame}", flush=True)
        print("[INFO] Green person = LiDAR-associated, Red person = fixed fallback.", flush=True)
        print("[INFO] Press q, ESC, or Ctrl+C to quit.", flush=True)

    # =========================================================
    # Setup / shutdown
    # =========================================================
    def signal_handler(self, signum, frame):
        print("\n[INFO] Stop signal received. Shutting down safely...", flush=True)
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

        print("[INFO] Starting RealSense pipeline...", flush=True)

        try:
            self.profile = self.pipeline.start(self.config)
            self.pipeline_started = True
        except Exception as e:
            print(f"[ERROR] Failed to start RealSense pipeline: {e}", flush=True)
            print("[HINT] Check camera owner with: sudo fuser -v /dev/video*", flush=True)
            raise

        color_stream = self.profile.get_stream(rs.stream.color)
        intr = color_stream.as_video_stream_profile().get_intrinsics()

        self.raw_fx = float(intr.fx)
        self.raw_fy = float(intr.fy)
        self.raw_cx = float(intr.ppx)
        self.raw_cy = float(intr.ppy)
        self.raw_w = int(intr.width)
        self.raw_h = int(intr.height)

        print(
            f"[INFO] RealSense raw intrinsics: "
            f"fx={self.raw_fx:.2f}, fy={self.raw_fy:.2f}, "
            f"cx={self.raw_cx:.2f}, cy={self.raw_cy:.2f}, "
            f"w={self.raw_w}, h={self.raw_h}",
            flush=True
        )

        warmup_start = time.time()
        while time.time() - warmup_start < 0.5:
            self.pipeline.poll_for_frames()
            time.sleep(0.01)

    def create_windows(self):
        try:
            cv2.namedWindow("Go2 Modified V2 Debug View", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Go2 Modified V2 Top View", cv2.WINDOW_NORMAL)

            cv2.resizeWindow("Go2 Modified V2 Debug View", 960, 720)
            cv2.resizeWindow("Go2 Modified V2 Top View", self.topview_size, self.topview_size)

            cv2.moveWindow("Go2 Modified V2 Debug View", 30, 40)
            cv2.moveWindow("Go2 Modified V2 Top View", 1020, 40)

            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                blank,
                "Waiting for RealSense + YOLO + LiDAR...",
                (30, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
                cv2.LINE_AA
            )

            top = self.draw_top_view([], None)

            cv2.imshow("Go2 Modified V2 Debug View", blank)
            cv2.imshow("Go2 Modified V2 Top View", top)
            cv2.waitKey(1)

        except Exception as e:
            print(f"[WARN] OpenCV window creation failed: {e}", flush=True)
            self.show_windows = False

    # =========================================================
    # Main loop
    # =========================================================
    def run(self):
        try:
            while self.running:
                rclpy.spin_once(self, timeout_sec=0.0)

                now = time.time()
                min_interval = 1.0 / max(self.target_fps, 0.1)

                if now - self.last_infer_time < min_interval:
                    self.handle_keyboard_short()
                    time.sleep(0.001)
                    continue

                self.last_infer_time = now

                raw_frame = self.get_realsense_color_frame_nonblocking()
                if raw_frame is None:
                    self.handle_keyboard_short()
                    time.sleep(0.002)
                    continue

                frame = self.rotate_image(raw_frame)
                lidar_points_base = self.get_latest_lidar_points_base_copy()

                persons, infer_ms, projected = self.run_yolo_and_lidar_fusion(
                    frame,
                    lidar_points_base
                )

                if infer_ms is not None:
                    self.infer_times.append(infer_ms)
                    if len(self.infer_times) > 30:
                        self.infer_times.pop(0)

                if self.show_windows:
                    debug_view = self.draw_debug_view(frame, persons, projected)
                    top_view = self.draw_top_view(persons, lidar_points_base)

                    cv2.imshow("Go2 Modified V2 Debug View", debug_view)
                    cv2.imshow("Go2 Modified V2 Top View", top_view)

                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q") or key == 27:
                        print("[INFO] Quit key pressed.", flush=True)
                        self.running = False

                self.print_status(persons, lidar_points_base)

        except KeyboardInterrupt:
            print("\n[INFO] KeyboardInterrupt caught.", flush=True)
            self.running = False

        except Exception as e:
            print(f"[ERROR] Runtime error: {e}", flush=True)
            self.running = False

        finally:
            self.stop()

    def handle_keyboard_short(self):
        if not self.show_windows:
            return

        try:
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                print("[INFO] Quit key pressed.", flush=True)
                self.running = False
        except Exception:
            pass

    def get_realsense_color_frame_nonblocking(self):
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
            print(f"[ERROR] RealSense polling error: {e}", flush=True)
            return None

    # =========================================================
    # Image rotation
    # =========================================================
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

    # =========================================================
    # LiDAR callback
    # =========================================================
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
            print("[WARN] PointCloud2 has no x/y/z fields.", flush=True)
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
        """
        If /lidar_points is already robot/base frame, use raw directly.
        If /lidar_points is Hesai sensor frame, apply lidar extrinsic.

        Assumed raw LiDAR frame:
            x forward
            y left
            z up
        """

        if self.lidar_points_frame == "base":
            return raw_points

        # row vector:
        # p_base = p_lidar @ R_base_lidar.T + t
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
    # YOLO + LiDAR fusion
    # =========================================================
    def run_yolo_and_lidar_fusion(self, frame, lidar_points_base):
        infer_start = time.time()

        try:
            results = self.model.predict(
                source=frame,
                imgsz=self.imgsz,
                conf=self.conf,
                iou=self.iou,
                classes=[0],
                device=self.device,
                half=self.use_half,
                verbose=False
            )
        except KeyboardInterrupt:
            self.running = False
            return [], None, None
        except Exception as e:
            print(f"[ERROR] YOLO inference failed: {e}", flush=True)
            return [], None, None

        infer_ms = (time.time() - infer_start) * 1000.0

        projected = None
        if lidar_points_base is not None and len(lidar_points_base) > 0:
            projected = self.project_base_points_to_rotated_image(
                lidar_points_base,
                frame.shape
            )

        persons = self.extract_persons_and_attach_lidar(
            results[0],
            frame.shape,
            projected
        )

        return persons, infer_ms, projected

    def extract_persons_and_attach_lidar(self, result, image_shape, projected):
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
                    k = max(1, int(self.mask_dilate_px))
                    kernel = np.ones((k, k), np.uint8)
                    mask_for_lidar = cv2.dilate(mask, kernel, iterations=1)
                else:
                    mask_for_lidar = mask

                u, v = self.get_mask_center(mask, x1, y1, x2, y2)
            else:
                u = 0.5 * (x1 + x2)
                v = 0.5 * (y1 + y2)

            lidar_info = None
            if projected is not None:
                lidar_info = self.compute_lidar_position_for_person(
                    projected,
                    x1,
                    y1,
                    x2,
                    y2,
                    mask_for_lidar
                )

            if lidar_info is not None:
                base_x, base_y, base_z = lidar_info["position"]
                source = "lidar"
                lidar_count = lidar_info["count"]
            else:
                base_x, base_y, base_z = self.pixel_to_fallback_base_position(u, v)
                source = "fixed"
                lidar_count = 0

            persons.append({
                "id": len(persons),
                "bbox": (x1, y1, x2, y2),
                "center_uv": (u, v),
                "confidence": float(confs[i]),
                "mask": mask,
                "base_position": (base_x, base_y, base_z),
                "distance_source": source,
                "lidar_count": lidar_count,
            })

        persons.sort(
            key=lambda p: math.sqrt(
                p["base_position"][0] ** 2 + p["base_position"][1] ** 2
            )
        )

        for idx, p in enumerate(persons):
            p["id"] = idx

        return persons

    def get_mask_center(self, mask, x1, y1, x2, y2):
        roi = mask[y1:y2, x1:x2]
        ys, xs = np.where(roi > 0)

        if len(xs) < 10:
            return 0.5 * (x1 + x2), 0.5 * (y1 + y2)

        return float(np.mean(xs + x1)), float(np.mean(ys + y1))

    def compute_lidar_position_for_person(self, projected, x1, y1, x2, y2, mask):
        """
        Robust LiDAR association for a YOLO person mask.

        Previous problem:
            Using median of all LiDAR points inside the person mask can select
            background wall points when the person/hand is very close or sparse.

        New strategy:
            1. Filter projected LiDAR points by bbox.
            2. Filter by person mask.
            3. Prefer the central body area, not the whole arm/hand area.
            4. Select the nearest valid depth cluster.
            5. Reject far background points.
            6. Return median of the nearest cluster.
        """

        if projected is None:
            return None

        u = projected["u"]
        v = projected["v"]
        points_base = projected["points_base"]

        # -----------------------------
        # 1. bbox filter
        # -----------------------------
        bbox_mask = (
            (u >= x1) & (u < x2) &
            (v >= y1) & (v < y2)
        )

        if np.count_nonzero(bbox_mask) == 0:
            return None

        candidate_idx = np.where(bbox_mask)[0]

        # -----------------------------
        # 2. mask filter
        # -----------------------------
        if mask is not None:
            uu = u[candidate_idx].astype(np.int32)
            vv = v[candidate_idx].astype(np.int32)

            inside = mask[vv, uu] > 0
            candidate_idx = candidate_idx[inside]

        if len(candidate_idx) < self.min_lidar_points_in_mask:
            return None

        # -----------------------------
        # 3. central region preference
        #    손/팔 끝부분보다 사람 몸통 중심부에 가까운 LiDAR를 우선 사용
        # -----------------------------
        bx_w = max(1, x2 - x1)
        bx_h = max(1, y2 - y1)

        uu = u[candidate_idx]
        vv = v[candidate_idx]

        # bbox 중심부 영역
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

        # -----------------------------
        # 4. distance-based nearest cluster
        #    base_x가 전방 거리이므로 우선 base_x 기준 가장 가까운 cluster 선택
        # -----------------------------
        forward_x = person_points[:, 0]

        # 로봇 앞쪽 유효 거리만 사용
        valid_front = (
            (forward_x > 0.15) &
            (forward_x < self.lidar_x_max)
        )

        person_points = person_points[valid_front]

        if len(person_points) < self.min_lidar_points_in_mask:
            return None

        forward_x = person_points[:, 0]

        # 가장 가까운 쪽 percentile 사용
        near_x = np.percentile(forward_x, 10)

        # 가까운 cluster만 남김
        # 사람 몸 두께/라이다 오차를 고려해서 0.45m 이내만 유지
        near_cluster = person_points[
            forward_x <= near_x + 0.45
        ]

        if len(near_cluster) < self.min_lidar_points_in_mask:
            return None

        # -----------------------------
        # 5. lateral outlier rejection
        # -----------------------------
        median_xyz = np.median(near_cluster, axis=0)

        diff_xy = near_cluster[:, :2] - median_xyz[:2]
        d_xy = np.linalg.norm(diff_xy, axis=1)

        keep = d_xy < self.person_outlier_radius

        if np.count_nonzero(keep) >= self.min_lidar_points_in_mask:
            near_cluster = near_cluster[keep]
            median_xyz = np.median(near_cluster, axis=0)

        return {
            "position": (
                float(median_xyz[0]),
                float(median_xyz[1]),
                float(median_xyz[2])
            ),
            "count": int(len(near_cluster))
        }

    # =========================================================
    # Projection
    # =========================================================
    def project_base_points_to_rotated_image(self, points_base, image_shape):
        """
        base points -> camera body -> camera optical -> raw image -> rotated image

        base frame:
            x forward, y left, z up

        camera body frame:
            x forward, y left, z up

        camera optical frame:
            X right, Y down, Z forward

        body to optical:
            X_opt = -y_body
            Y_opt = -z_body
            Z_opt =  x_body
        """

        if points_base is None or len(points_base) == 0:
            return None

        h_rot, w_rot = image_shape[:2]

        p_rel_base = points_base - self.camera_t_base.reshape(1, 3)

        # row vector inverse transform:
        # p_body = R_base_camera_body.T @ p_rel_base
        # row form = p_rel_base @ R_base_camera_body
        p_body = p_rel_base @ self.R_base_camera_body

        x_body = p_body[:, 0]
        y_body = p_body[:, 1]
        z_body = p_body[:, 2]

        X_opt = -y_body
        Y_opt = -z_body
        Z_opt = x_body

        valid = Z_opt > 0.05

        if np.count_nonzero(valid) == 0:
            return None

        X_opt = X_opt[valid]
        Y_opt = Y_opt[valid]
        Z_opt = Z_opt[valid]
        points_base_valid = points_base[valid]

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
        points_base_valid = points_base_valid[raw_valid]
        Z_opt = Z_opt[raw_valid]

        u_rot, v_rot = self.original_uv_to_rotated_uv(u0, v0)

        rot_valid = (
            (u_rot >= 0) & (u_rot < w_rot) &
            (v_rot >= 0) & (v_rot < h_rot)
        )

        if np.count_nonzero(rot_valid) == 0:
            return None

        u_rot = u_rot[rot_valid].astype(np.int32)
        v_rot = v_rot[rot_valid].astype(np.int32)
        points_base_valid = points_base_valid[rot_valid]
        Z_opt = Z_opt[rot_valid]

        return {
            "u": u_rot,
            "v": v_rot,
            "points_base": points_base_valid,
            "z_opt": Z_opt
        }

    def pixel_to_fallback_base_position(self, u_rot, v_rot):
        u0, v0 = self.rotated_uv_to_original_uv(u_rot, v_rot)

        z_opt = self.fixed_distance
        X_opt = (u0 - self.raw_cx) * z_opt / self.raw_fx
        Y_opt = (v0 - self.raw_cy) * z_opt / self.raw_fy

        # optical -> camera body
        x_body = z_opt
        y_body = -X_opt
        z_body = -Y_opt

        p_body = np.array([x_body, y_body, z_body], dtype=np.float32)
        p_base = self.R_base_camera_body @ p_body + self.camera_t_base

        return float(p_base[0]), float(p_base[1]), float(p_base[2])

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

    # =========================================================
    # Visualization
    # =========================================================
    def draw_debug_view(self, frame, persons, projected):
        out = frame.copy()

        if self.draw_projected_lidar and projected is not None:
            u = projected["u"]
            v = projected["v"]

            max_draw = 4000
            if len(u) > max_draw:
                idx = np.random.choice(len(u), max_draw, replace=False)
                uu = u[idx]
                vv = v[idx]
            else:
                uu = u
                vv = v

            for px, py in zip(uu, vv):
                cv2.circle(out, (int(px), int(py)), 1, (255, 120, 0), -1)

        for person in persons:
            x1, y1, x2, y2 = person["bbox"]
            mask = person["mask"]
            source = person["distance_source"]

            color = (0, 255, 0) if source == "lidar" else (0, 0, 255)

            if mask is not None:
                overlay = np.zeros_like(out)
                if source == "lidar":
                    overlay[:, :, 1] = mask * 255
                else:
                    overlay[:, :, 2] = mask * 255
                out = cv2.addWeighted(out, 1.0, overlay, 0.35, 0)

            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)

            u, v = person["center_uv"]
            cv2.circle(out, (int(u), int(v)), 5, (0, 255, 255), -1)

            x, y, z = person["base_position"]
            label = (
                f"id:{person['id']} {source} n={person['lidar_count']} "
                f"c={person['confidence']:.2f} "
                f"base=({x:.2f},{y:.2f},{z:.2f})"
            )

            cv2.putText(
                out,
                label,
                (x1, max(25, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                color,
                2,
                cv2.LINE_AA
            )

        avg_ms = sum(self.infer_times) / len(self.infer_times) if self.infer_times else 0.0
        lidar_state = "OK" if projected is not None else "NO_PROJECTED_POINTS"

        status = (
            f"Modified V2 | rot={self.image_rotate} | persons={len(persons)} | "
            f"avg={avg_ms:.1f}ms | LiDAR={lidar_state} | "
            f"green=lidar red=fallback"
        )

        cv2.putText(
            out,
            status,
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2,
            cv2.LINE_AA
        )

        return out

    def draw_top_view(self, persons, lidar_points_base):
        size = self.topview_size
        canvas = np.zeros((size, size, 3), dtype=np.uint8)
        canvas[:] = (20, 20, 20)

        origin_x = size // 2
        origin_y = int(size * 0.82)

        self.draw_grid(canvas, origin_x, origin_y)

        if self.draw_lidar_points and lidar_points_base is not None:
            self.draw_lidar_points_on_topview(canvas, lidar_points_base, origin_x, origin_y)

        self.draw_robot(canvas, origin_x, origin_y)
        self.draw_sensor_positions(canvas, origin_x, origin_y)

        for person in persons:
            x, y, _ = person["base_position"]
            px, py = self.base_to_canvas(x, y, origin_x, origin_y)

            if not self.is_inside_canvas(px, py, size):
                continue

            source = person["distance_source"]
            color = (0, 255, 0) if source == "lidar" else (0, 0, 255)

            cv2.circle(canvas, (px, py), 13, color, -1)
            cv2.circle(canvas, (px, py), 17, (255, 255, 255), 2)
            cv2.line(canvas, (origin_x, origin_y), (px, py), color, 2)

            dist = math.sqrt(x * x + y * y)
            text1 = f"P{person['id']} {dist:.2f}m {source}"
            text2 = f"x={x:.2f}, y={y:.2f}, n={person['lidar_count']}"

            cv2.putText(canvas, text1, (px + 18, py - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(canvas, text2, (px + 18, py + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 255, 180), 1, cv2.LINE_AA)

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
        body_w = 48
        body_h = 70

        x1 = origin_x - body_w // 2
        y1 = origin_y - body_h // 2
        x2 = origin_x + body_w // 2
        y2 = origin_y + body_h // 2

        cv2.rectangle(canvas, (x1, y1), (x2, y2), (80, 160, 255), -1)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (255, 255, 255), 2)

        cv2.arrowedLine(
            canvas,
            (origin_x, origin_y),
            (origin_x, origin_y - 70),
            (0, 255, 255),
            3,
            tipLength=0.25
        )

        cv2.putText(canvas, "ROBOT", (origin_x - 32, origin_y + 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1, cv2.LINE_AA)

    def draw_sensor_positions(self, canvas, origin_x, origin_y):
        lx, ly, _ = self.lidar_t_base
        lpx, lpy = self.base_to_canvas(lx, ly, origin_x, origin_y)
        if self.is_inside_canvas(lpx, lpy, canvas.shape[0]):
            cv2.circle(canvas, (lpx, lpy), 7, (255, 120, 0), -1)
            cv2.putText(canvas, "Hesai XT16", (lpx + 8, lpy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                        (255, 160, 80), 1, cv2.LINE_AA)

        cx, cy, _ = self.camera_t_base
        cpx, cpy = self.base_to_canvas(cx, cy, origin_x, origin_y)
        if self.is_inside_canvas(cpx, cpy, canvas.shape[0]):
            cv2.circle(canvas, (cpx, cpy), 7, (0, 180, 255), -1)
            cv2.putText(canvas, "D435i", (cpx + 8, cpy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                        (0, 220, 255), 1, cv2.LINE_AA)

    def draw_lidar_points_on_topview(self, canvas, lidar_points, origin_x, origin_y):
        max_draw = 5000
        if len(lidar_points) > max_draw:
            idx = np.random.choice(len(lidar_points), max_draw, replace=False)
            pts = lidar_points[idx]
        else:
            pts = lidar_points

        for p in pts:
            px, py = self.base_to_canvas(float(p[0]), float(p[1]), origin_x, origin_y)
            if self.is_inside_canvas(px, py, canvas.shape[0]):
                cv2.circle(canvas, (px, py), 1, (90, 90, 90), -1)

    def draw_topview_labels(self, canvas, origin_x, origin_y):
        cv2.putText(canvas, "Modified V2 Robot-Centered Top View",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.72, (255, 255, 255), 2, cv2.LINE_AA)

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
                    "Gray: LiDAR | Green: LiDAR-associated person | Red: fallback",
                    (20, canvas.shape[0] - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.43,
                    (180, 180, 180), 1, cv2.LINE_AA)

    def base_to_canvas(self, base_x, base_y, origin_x, origin_y):
        px = int(origin_x - base_y * self.meter_to_pixel)
        py = int(origin_y - base_x * self.meter_to_pixel)
        return px, py

    @staticmethod
    def is_inside_canvas(px, py, size):
        return 0 <= px < size and 0 <= py < size

    # =========================================================
    # Logging / cleanup
    # =========================================================
    def print_status(self, persons, lidar_points):
        now = time.time()
        if now - self.last_log_time < 1.0:
            return

        avg_ms = sum(self.infer_times) / len(self.infer_times) if self.infer_times else 0.0
        lidar_count = 0 if lidar_points is None else len(lidar_points)

        print(
            f"[STATUS] persons={len(persons)} | "
            f"avg_infer={avg_ms:.1f}ms | "
            f"lidar_points={lidar_count} | "
            f"rot={self.image_rotate}",
            flush=True
        )

        for p in persons:
            x, y, z = p["base_position"]
            print(
                f"  - person_{p['id']}: "
                f"source={p['distance_source']}, "
                f"lidar_n={p['lidar_count']}, "
                f"conf={p['confidence']:.2f}, "
                f"x={x:.2f}, y={y:.2f}, z={z:.2f}",
                flush=True
            )

        self.last_log_time = now

    def stop(self):
        if getattr(self, "pipeline_started", False):
            print("[INFO] Releasing RealSense pipeline...", flush=True)
            self.pipeline_started = False
            try:
                self.pipeline.stop()
                print("[INFO] RealSense pipeline stopped.", flush=True)
            except Exception as e:
                print(f"[WARN] pipeline.stop() failed: {e}", flush=True)

        try:
            cv2.destroyAllWindows()
            for _ in range(5):
                cv2.waitKey(1)
        except Exception:
            pass


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--model", default="yolo11s-seg.pt")

    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)

    parser.add_argument(
        "--image-rotate",
        default="none",
        choices=["none", "cw90", "ccw90", "180"],
        help="Rotate RealSense image before YOLO. Default ccw90 based on current mounting."
    )

    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--iou", type=float, default=0.55)
    parser.add_argument("--device", default="0")
    parser.add_argument("--half", action="store_true")
    parser.add_argument("--target-fps", type=float, default=10.0)

    parser.add_argument("--lidar-topic", default="/lidar_points")
    parser.add_argument(
        "--lidar-points-frame",
        default="lidar",
        choices=["lidar", "base"],
        help="Use 'lidar' if /lidar_points is Hesai sensor frame. Use 'base' if already robot base frame."
    )

    # Updated sensor positions from provided mounting images
    parser.add_argument("--camera-base-x", type=float, default=0.38)
    parser.add_argument("--camera-base-y", type=float, default=0.00)
    parser.add_argument("--camera-base-z", type=float, default=0.12)

    parser.add_argument("--camera-roll-deg", type=float, default=0.0)
    parser.add_argument("--camera-pitch-deg", type=float, default=0.0)
    parser.add_argument("--camera-yaw-deg", type=float, default=0.0)

    parser.add_argument("--lidar-base-x", type=float, default=0.15)
    parser.add_argument("--lidar-base-y", type=float, default=0.00)
    parser.add_argument("--lidar-base-z", type=float, default=0.25)

    # Default 90 deg because current projection appears rotated.
    parser.add_argument("--lidar-roll-deg", type=float, default=0.0)
    parser.add_argument("--lidar-pitch-deg", type=float, default=0.0)
    parser.add_argument("--lidar-yaw-deg", type=float, default=90.0)

    parser.add_argument("--lidar-x-min", type=float, default=0.20)
    parser.add_argument("--lidar-x-max", type=float, default=8.00)
    parser.add_argument("--lidar-y-min", type=float, default=-4.00)
    parser.add_argument("--lidar-y-max", type=float, default=4.00)
    parser.add_argument("--lidar-z-min", type=float, default=-1.20)
    parser.add_argument("--lidar-z-max", type=float, default=2.50)
    parser.add_argument("--max-lidar-points", type=int, default=30000)

    parser.add_argument("--min-lidar-points-in-mask", type=int, default=3)
    parser.add_argument("--mask-dilate-px", type=int, default=15)
    parser.add_argument("--person-outlier-radius", type=float, default=0.8)

    parser.add_argument("--fixed-distance", type=float, default=2.0)

    parser.add_argument("--topview-size", type=int, default=800)
    parser.add_argument("--meter-to-pixel", type=float, default=90.0)
    parser.add_argument("--max-forward-m", type=float, default=6.0)
    parser.add_argument("--max-side-m", type=float, default=3.0)

    parser.add_argument("--draw-lidar-points", action="store_true")
    parser.add_argument("--draw-projected-lidar", action="store_true")
    parser.add_argument("--no-gui", action="store_true")

    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init(args=None)

    app = None

    try:
        app = Go2YoloLidarFusionV2(args)
        app.run()

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt caught in main.", flush=True)

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

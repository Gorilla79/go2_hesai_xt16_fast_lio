#!/usr/bin/env python3

import argparse
import math
import time
import signal
import atexit
import sys

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO


class RealSenseYoloTopView:
    def __init__(self, args):
        self.args = args
        self.running = True
        self.pipeline_started = False

        self.model_path = args.model
        self.width = args.width
        self.height = args.height
        self.camera_fps = args.camera_fps

        self.imgsz = args.imgsz
        self.conf = args.conf
        self.iou = args.iou
        self.device = args.device
        self.use_half = args.half
        self.target_fps = args.target_fps

        self.fixed_distance = args.fixed_distance

        # Approximate RealSense pose relative to robot base center.
        # base_link convention:
        #   x: forward
        #   y: left
        #   z: up
        self.camera_offset_x = args.camera_offset_x
        self.camera_offset_y = args.camera_offset_y
        self.camera_offset_z = args.camera_offset_z

        self.topview_size = args.topview_size
        self.meter_to_pixel = args.meter_to_pixel
        self.max_forward_m = args.max_forward_m
        self.max_side_m = args.max_side_m

        self.show_windows = not args.no_gui

        self.last_infer_time = 0.0
        self.infer_times = []
        self.last_log_time = time.time()

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.pipeline = None
        self.config = None
        self.profile = None

        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        atexit.register(self.stop)

        print(f"[INFO] Loading YOLO model once: {self.model_path}", flush=True)
        self.model = YOLO(self.model_path)
        print("[INFO] YOLO model loaded.", flush=True)

        self.start_realsense()

        if self.show_windows:
            self.create_windows()

        print("[INFO] System started.", flush=True)
        print("[INFO] Press 'q' or ESC in the OpenCV window to quit.", flush=True)
        print("[INFO] Ctrl+C is also supported.", flush=True)
        print("[INFO] Current mode: temporary fixed-distance visualization.", flush=True)
        print("[INFO] LiDAR projection is not connected yet.", flush=True)

    def signal_handler(self, signum, frame):
        print("\n[INFO] Stop signal received. Shutting down safely...", flush=True)
        self.running = False

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
            print("[HINT] The camera may be busy. Check with:", flush=True)
            print("       sudo fuser -v /dev/video*", flush=True)
            raise

        color_stream = self.profile.get_stream(rs.stream.color)
        intr = color_stream.as_video_stream_profile().get_intrinsics()

        self.fx = float(intr.fx)
        self.fy = float(intr.fy)
        self.cx = float(intr.ppx)
        self.cy = float(intr.ppy)

        print(
            f"[INFO] RealSense intrinsics: "
            f"fx={self.fx:.2f}, fy={self.fy:.2f}, "
            f"cx={self.cx:.2f}, cy={self.cy:.2f}",
            flush=True
        )

        # Warm-up non-blocking reads.
        warmup_start = time.time()
        while time.time() - warmup_start < 0.5:
            self.pipeline.poll_for_frames()
            time.sleep(0.01)

    def create_windows(self):
        try:
            cv2.namedWindow("Go2 RealSense YOLO11s-seg Debug View", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Go2 Robot-Centered Top View", cv2.WINDOW_NORMAL)

            cv2.resizeWindow("Go2 RealSense YOLO11s-seg Debug View", 960, 720)
            cv2.resizeWindow("Go2 Robot-Centered Top View", self.topview_size, self.topview_size)

            cv2.moveWindow("Go2 RealSense YOLO11s-seg Debug View", 30, 40)
            cv2.moveWindow("Go2 Robot-Centered Top View", 1020, 40)

            blank_debug = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                blank_debug,
                "Waiting for RealSense + YOLO inference...",
                (30, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
                cv2.LINE_AA
            )

            blank_top = self.draw_top_view([])

            cv2.imshow("Go2 RealSense YOLO11s-seg Debug View", blank_debug)
            cv2.imshow("Go2 Robot-Centered Top View", blank_top)
            cv2.waitKey(1)

        except Exception as e:
            print(f"[WARN] Failed to create OpenCV windows: {e}", flush=True)
            print("[WARN] Continuing in no-gui mode.", flush=True)
            self.show_windows = False

    def run(self):
        try:
            while self.running:
                now = time.time()
                min_interval = 1.0 / max(self.target_fps, 0.1)

                if now - self.last_infer_time < min_interval:
                    self.handle_keyboard_short()
                    time.sleep(0.001)
                    continue

                self.last_infer_time = now

                frame = self.get_realsense_color_frame_nonblocking()

                if frame is None:
                    self.handle_keyboard_short()
                    time.sleep(0.002)
                    continue

                persons, infer_ms = self.run_yolo(frame)

                if infer_ms is not None:
                    self.infer_times.append(infer_ms)
                    if len(self.infer_times) > 30:
                        self.infer_times.pop(0)

                if self.show_windows:
                    debug_view = self.draw_debug_view(frame, persons)
                    top_view = self.draw_top_view(persons)

                    cv2.imshow("Go2 RealSense YOLO11s-seg Debug View", debug_view)
                    cv2.imshow("Go2 Robot-Centered Top View", top_view)

                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q") or key == 27:
                        print("[INFO] Quit key pressed.", flush=True)
                        self.running = False

                self.print_status(persons)

        except KeyboardInterrupt:
            print("\n[INFO] KeyboardInterrupt caught in run loop.", flush=True)
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
        """
        Non-blocking RealSense frame acquisition.
        This avoids being stuck inside wait_for_frames() when Ctrl+C is pressed.
        """
        if not self.pipeline_started:
            return None

        try:
            frames = self.pipeline.poll_for_frames()

            if not frames:
                return None

            color_frame = frames.get_color_frame()

            if not color_frame:
                return None

            frame = np.asanyarray(color_frame.get_data())
            return frame

        except Exception as e:
            print(f"[ERROR] RealSense frame polling error: {e}", flush=True)
            return None

    def run_yolo(self, frame):
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
            print("\n[INFO] KeyboardInterrupt during YOLO inference.", flush=True)
            self.running = False
            return [], None

        except Exception as e:
            print(f"[ERROR] YOLO inference failed: {e}", flush=True)
            return [], None

        infer_ms = (time.time() - infer_start) * 1000.0
        persons = self.extract_persons(results[0], frame.shape)

        return persons, infer_ms

    def extract_persons(self, result, image_shape):
        persons = []
        h, w = image_shape[:2]

        if result.boxes is None or len(result.boxes) == 0:
            return persons

        boxes = result.boxes.xyxy.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()
        clss = result.boxes.cls.cpu().numpy().astype(np.int32)

        has_masks = result.masks is not None
        masks = None

        if has_masks:
            masks = result.masks.data.cpu().numpy()

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

            if has_masks and i < len(masks):
                mask_small = masks[i]
                mask = cv2.resize(mask_small, (w, h), interpolation=cv2.INTER_NEAREST)
                mask = (mask > 0.5).astype(np.uint8)

                u, v = self.get_mask_center(mask, x1, y1, x2, y2)
                area = int(np.sum(mask > 0))
            else:
                u = 0.5 * (x1 + x2)
                v = 0.5 * (y1 + y2)
                area = int((x2 - x1) * (y2 - y1))

            base_x, base_y, base_z = self.pixel_to_temporary_base_position(u, v)

            persons.append({
                "id": len(persons),
                "bbox": (x1, y1, x2, y2),
                "center_uv": (u, v),
                "confidence": float(confs[i]),
                "mask": mask,
                "mask_area": area,
                "base_position": (base_x, base_y, base_z),
            })

        persons.sort(
            key=lambda p: math.sqrt(
                p["base_position"][0] ** 2 + p["base_position"][1] ** 2
            )
        )

        for idx, person in enumerate(persons):
            person["id"] = idx

        return persons

    def get_mask_center(self, mask, x1, y1, x2, y2):
        roi = mask[y1:y2, x1:x2]
        ys, xs = np.where(roi > 0)

        if len(xs) < 10:
            return 0.5 * (x1 + x2), 0.5 * (y1 + y2)

        u = float(np.mean(xs + x1))
        v = float(np.mean(ys + y1))

        return u, v

    def pixel_to_temporary_base_position(self, u, v):
        """
        Temporary camera pixel to robot base coordinate conversion.

        RealSense optical frame:
            x: right
            y: down
            z: forward

        Robot base frame approximation:
            x: forward
            y: left
            z: up

        This function uses fixed_distance for now.
        Later:
            fixed_distance -> LiDAR projected distance inside person mask.
        """
        z_cam = self.fixed_distance
        x_cam = (u - self.cx) * z_cam / self.fx
        y_cam = (v - self.cy) * z_cam / self.fy

        base_x = z_cam + self.camera_offset_x
        base_y = -x_cam + self.camera_offset_y
        base_z = -y_cam + self.camera_offset_z

        return base_x, base_y, base_z

    def draw_debug_view(self, frame, persons):
        out = frame.copy()

        for person in persons:
            x1, y1, x2, y2 = person["bbox"]
            mask = person["mask"]
            person_id = person["id"]
            conf = person["confidence"]
            u, v = person["center_uv"]
            base_x, base_y, base_z = person["base_position"]

            if mask is not None:
                overlay = np.zeros_like(out)
                overlay[:, :, 1] = mask * 255
                out = cv2.addWeighted(out, 1.0, overlay, 0.35, 0)

            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(out, (int(u), int(v)), 5, (0, 0, 255), -1)

            label = (
                f"id:{person_id} conf:{conf:.2f} "
                f"tmp_base=({base_x:.2f},{base_y:.2f},{base_z:.2f})"
            )

            cv2.putText(
                out,
                label,
                (x1, max(25, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )

        avg_ms = 0.0
        if len(self.infer_times) > 0:
            avg_ms = sum(self.infer_times) / len(self.infer_times)

        status = (
            f"YOLO11s-seg | persons={len(persons)} | "
            f"avg={avg_ms:.1f}ms | "
            f"fixed_distance={self.fixed_distance:.2f}m | "
            f"LiDAR: NOT connected | q/ESC/Ctrl+C to quit"
        )

        cv2.putText(
            out,
            status,
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 0, 255),
            2,
            cv2.LINE_AA
        )

        return out

    def draw_top_view(self, persons):
        size = self.topview_size
        canvas = np.zeros((size, size, 3), dtype=np.uint8)
        canvas[:] = (20, 20, 20)

        origin_x = size // 2
        origin_y = int(size * 0.82)

        self.draw_grid(canvas, origin_x, origin_y)
        self.draw_robot(canvas, origin_x, origin_y)

        for person in persons:
            base_x, base_y, _ = person["base_position"]
            person_id = person["id"]
            conf = person["confidence"]

            px, py = self.base_to_canvas(base_x, base_y, origin_x, origin_y)

            if not self.is_inside_canvas(px, py, size):
                continue

            cv2.circle(canvas, (px, py), 12, (0, 255, 0), -1)
            cv2.circle(canvas, (px, py), 16, (255, 255, 255), 2)

            cv2.line(canvas, (origin_x, origin_y), (px, py), (0, 180, 0), 2)

            distance = math.sqrt(base_x ** 2 + base_y ** 2)

            text = f"P{person_id} {distance:.2f}m"
            text2 = f"x={base_x:.2f}, y={base_y:.2f}, c={conf:.2f}"

            cv2.putText(
                canvas,
                text,
                (px + 18, py - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA
            )

            cv2.putText(
                canvas,
                text2,
                (px + 18, py + 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (180, 255, 180),
                1,
                cv2.LINE_AA
            )

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
                cv2.line(
                    canvas,
                    (origin_x - max_side_px, y),
                    (origin_x + max_side_px, y),
                    grid_color,
                    1
                )

                cv2.putText(
                    canvas,
                    f"{m:.1f}m",
                    (origin_x + max_side_px + 8, y + 4),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.35,
                    (120, 120, 120),
                    1,
                    cv2.LINE_AA
                )

        for m in np.arange(-self.max_side_m, self.max_side_m + 0.001, 0.5):
            x = int(origin_x - m * self.meter_to_pixel)
            if 0 <= x < size:
                cv2.line(
                    canvas,
                    (x, origin_y),
                    (x, origin_y - max_forward_px),
                    grid_color,
                    1
                )

        cv2.line(
            canvas,
            (origin_x, origin_y),
            (origin_x, max(0, origin_y - max_forward_px)),
            axis_color,
            2
        )

        cv2.line(
            canvas,
            (origin_x - max_side_px, origin_y),
            (origin_x + max_side_px, origin_y),
            axis_color,
            2
        )

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

        cv2.putText(
            canvas,
            "ROBOT",
            (origin_x - 32, origin_y + 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )

    def draw_topview_labels(self, canvas, origin_x, origin_y):
        cv2.putText(
            canvas,
            "Robot-Centered Top View",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

        cv2.putText(
            canvas,
            "+X Forward",
            (origin_x + 15, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            1,
            cv2.LINE_AA
        )

        cv2.putText(
            canvas,
            "+Y Left",
            (30, origin_y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            1,
            cv2.LINE_AA
        )

        cv2.putText(
            canvas,
            "-Y Right",
            (canvas.shape[1] - 130, origin_y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            1,
            cv2.LINE_AA
        )

        cv2.putText(
            canvas,
            "Temporary distance mode. LiDAR projection will replace fixed distance.",
            (20, canvas.shape[0] - 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 255),
            1,
            cv2.LINE_AA
        )

    def base_to_canvas(self, base_x, base_y, origin_x, origin_y):
        px = int(origin_x - base_y * self.meter_to_pixel)
        py = int(origin_y - base_x * self.meter_to_pixel)
        return px, py

    @staticmethod
    def is_inside_canvas(px, py, size):
        return 0 <= px < size and 0 <= py < size

    def print_status(self, persons):
        now = time.time()

        if now - self.last_log_time < 1.0:
            return

        avg_ms = 0.0
        if len(self.infer_times) > 0:
            avg_ms = sum(self.infer_times) / len(self.infer_times)

        print(
            f"[STATUS] persons={len(persons)} | "
            f"avg_infer={avg_ms:.1f} ms | "
            f"target_fps={self.target_fps:.1f} | "
            f"fixed_distance={self.fixed_distance:.2f} m",
            flush=True
        )

        for person in persons:
            x, y, z = person["base_position"]
            print(
                f"  - person_{person['id']}: "
                f"conf={person['confidence']:.2f}, "
                f"base_x={x:.2f}, base_y={y:.2f}, base_z={z:.2f}",
                flush=True
            )

        self.last_log_time = now

    def stop(self):
        if not getattr(self, "pipeline_started", False):
            return

        print("[INFO] Releasing RealSense and OpenCV resources...", flush=True)

        self.pipeline_started = False

        try:
            self.pipeline.stop()
            print("[INFO] RealSense pipeline stopped.", flush=True)
        except Exception as e:
            print(f"[WARN] pipeline.stop() failed or already stopped: {e}", flush=True)

        try:
            cv2.destroyAllWindows()
            for _ in range(5):
                cv2.waitKey(1)
            print("[INFO] OpenCV windows destroyed.", flush=True)
        except Exception as e:
            print(f"[WARN] cv2.destroyAllWindows() failed: {e}", flush=True)


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--model", default="yolo11s-seg.pt")

    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)

    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--iou", type=float, default=0.55)
    parser.add_argument("--device", default="0")
    parser.add_argument("--half", action="store_true")
    parser.add_argument("--target-fps", type=float, default=10.0)

    parser.add_argument("--fixed-distance", type=float, default=2.0)

    parser.add_argument("--camera-offset-x", type=float, default=0.28)
    parser.add_argument("--camera-offset-y", type=float, default=0.00)
    parser.add_argument("--camera-offset-z", type=float, default=0.35)

    parser.add_argument("--topview-size", type=int, default=800)
    parser.add_argument("--meter-to-pixel", type=float, default=90.0)
    parser.add_argument("--max-forward-m", type=float, default=6.0)
    parser.add_argument("--max-side-m", type=float, default=3.0)

    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="Run without OpenCV windows. Useful for SSH/headless test."
    )

    return parser.parse_args()


def main():
    args = parse_args()

    app = None

    try:
        app = RealSenseYoloTopView(args)
        app.run()

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt caught in main.", flush=True)

    except Exception as e:
        print(f"[ERROR] Fatal error: {e}", flush=True)

    finally:
        if app is not None:
            app.running = False
            app.stop()

        print("[INFO] Program finished.", flush=True)


if __name__ == "__main__":
    main()

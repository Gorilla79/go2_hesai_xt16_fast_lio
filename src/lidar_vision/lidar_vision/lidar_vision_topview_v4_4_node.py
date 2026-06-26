#!/usr/bin/env python3

import math
import time
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2


class LiDARVisionTopViewV44Node(Node):
    """
    V4.4 diagnostic node for Hesai XT16 range-image construction.

    Input:
        /lidar_points

    Expected PointCloud2 fields:
        x, y, z, intensity, ring, timestamp

    Output:
        OpenCV dashboard containing:
        - 16 x 4000 range image
        - intensity image
        - validity mask
        - ring/timestamp/azimuth diagnostics

    This version does not perform:
        - person detection
        - motion segmentation
        - connected-component segmentation
        - tracking
    """

    def __init__(self):
        super().__init__("lidar_vision_topview_v4_4_node")

        # Topics and sensor layout
        self.declare_parameter("lidar_topic", "/lidar_points")
        self.declare_parameter("expected_rings", 16)
        self.declare_parameter("expected_columns", 4000)

        # Range image construction
        self.declare_parameter("min_range_m", 0.30)
        self.declare_parameter("max_range_m", 20.0)
        self.declare_parameter("column_mode", "azimuth")
        self.declare_parameter("azimuth_zero_deg", 0.0)
        self.declare_parameter("azimuth_clockwise", False)
        self.declare_parameter("keep_nearest_on_collision", True)

        # Visualization
        self.declare_parameter("display_scale_x", 0.25)
        self.declare_parameter("display_scale_y", 18.0)
        self.declare_parameter("display_fps", 10.0)
        self.declare_parameter("window_name", "LiDAR V4.4 XT16 Range Image")
        self.declare_parameter("show_intensity", True)
        self.declare_parameter("show_validity", True)
        self.declare_parameter("print_interval_sec", 2.0)

        self.lidar_topic = str(self.get_parameter("lidar_topic").value)
        self.expected_rings = int(self.get_parameter("expected_rings").value)
        self.expected_columns = int(
            self.get_parameter("expected_columns").value
        )

        self.min_range_m = float(self.get_parameter("min_range_m").value)
        self.max_range_m = float(self.get_parameter("max_range_m").value)
        self.column_mode = str(
            self.get_parameter("column_mode").value
        ).lower()
        self.azimuth_zero_deg = float(
            self.get_parameter("azimuth_zero_deg").value
        )
        self.azimuth_clockwise = bool(
            self.get_parameter("azimuth_clockwise").value
        )
        self.keep_nearest_on_collision = bool(
            self.get_parameter("keep_nearest_on_collision").value
        )

        self.display_scale_x = float(
            self.get_parameter("display_scale_x").value
        )
        self.display_scale_y = float(
            self.get_parameter("display_scale_y").value
        )
        self.display_fps = float(self.get_parameter("display_fps").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.show_intensity = bool(
            self.get_parameter("show_intensity").value
        )
        self.show_validity = bool(
            self.get_parameter("show_validity").value
        )
        self.print_interval_sec = float(
            self.get_parameter("print_interval_sec").value
        )

        self.latest_range_image: Optional[np.ndarray] = None
        self.latest_intensity_image: Optional[np.ndarray] = None
        self.latest_valid_mask: Optional[np.ndarray] = None
        self.latest_stats: Dict[str, float] = {}
        self.last_print_time = 0.0
        self.last_receive_time = 0.0

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.cloud_callback,
            qos,
        )

        self.create_timer(
            1.0 / max(1.0, self.display_fps),
            self.display_callback,
        )

        self.get_logger().info(
            "V4.4 XT16 range-image diagnostic node started."
        )
        self.get_logger().info(
            f"topic={self.lidar_topic}, "
            f"rings={self.expected_rings}, "
            f"columns={self.expected_columns}"
        )

    def cloud_callback(self, msg: PointCloud2):
        start = time.perf_counter()
        receive_time = self.local_time_sec()

        try:
            arrays = self.parse_cloud(msg)
        except Exception as exc:
            self.get_logger().error(f"PointCloud2 parse failed: {exc}")
            return

        xyz = arrays["xyz"]
        intensity = arrays["intensity"]
        ring = arrays["ring"]
        timestamp = arrays["timestamp"]

        valid_xyz = np.isfinite(xyz).all(axis=1)
        valid_ring = (
            (ring >= 0)
            & (ring < self.expected_rings)
        )
        valid_time = np.isfinite(timestamp)

        ranges = np.linalg.norm(xyz, axis=1)
        valid_range = (
            np.isfinite(ranges)
            & (ranges >= self.min_range_m)
            & (ranges <= self.max_range_m)
        )

        valid = valid_xyz & valid_ring & valid_time & valid_range

        xyz = xyz[valid]
        intensity = intensity[valid]
        ring = ring[valid]
        timestamp = timestamp[valid]
        ranges = ranges[valid]

        if len(xyz) == 0:
            return

        if self.column_mode == "sequence":
            columns = self.sequence_columns(ring)
        else:
            columns = self.azimuth_columns(xyz)

        range_image = np.full(
            (self.expected_rings, self.expected_columns),
            np.nan,
            dtype=np.float32,
        )
        intensity_image = np.full(
            (self.expected_rings, self.expected_columns),
            np.nan,
            dtype=np.float32,
        )

        if self.keep_nearest_on_collision:
            flat_index = (
                ring.astype(np.int64) * self.expected_columns
                + columns.astype(np.int64)
            )
            order = np.lexsort((ranges, flat_index))
            sorted_flat = flat_index[order]

            keep = np.ones(len(order), dtype=bool)
            if len(order) > 1:
                keep[1:] = (
                    sorted_flat[1:] != sorted_flat[:-1]
                )

            selected = order[keep]
        else:
            selected = np.arange(len(ranges))

        range_image[ring[selected], columns[selected]] = ranges[selected]
        intensity_image[ring[selected], columns[selected]] = (
            intensity[selected]
        )

        valid_mask = np.isfinite(range_image)

        ring_counts = np.bincount(
            ring,
            minlength=self.expected_rings,
        )

        timestamp_min = float(np.min(timestamp))
        timestamp_max = float(np.max(timestamp))
        timestamp_span = timestamp_max - timestamp_min

        azimuth = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))
        azimuth = (azimuth + 360.0) % 360.0

        occupied_cells = int(np.count_nonzero(valid_mask))
        collision_count = int(len(ranges) - occupied_cells)

        stats = {
            "raw_points": float(msg.width * msg.height),
            "valid_points": float(len(ranges)),
            "occupied_cells": float(occupied_cells),
            "coverage_percent": (
                100.0
                * occupied_cells
                / float(self.expected_rings * self.expected_columns)
            ),
            "collision_count": float(max(0, collision_count)),
            "timestamp_min": timestamp_min,
            "timestamp_max": timestamp_max,
            "timestamp_span": timestamp_span,
            "unique_timestamps": float(len(np.unique(timestamp))),
            "ring_min": float(np.min(ring)),
            "ring_max": float(np.max(ring)),
            "ring_count": float(len(np.unique(ring))),
            "azimuth_min": float(np.min(azimuth)),
            "azimuth_max": float(np.max(azimuth)),
            "range_min": float(np.nanmin(range_image)),
            "range_max": float(np.nanmax(range_image)),
            "build_ms": (
                time.perf_counter() - start
            ) * 1000.0,
            "cloud_age_ms": 0.0,
        }

        for index, count in enumerate(ring_counts):
            stats[f"ring_{index}_count"] = float(count)

        self.latest_range_image = range_image
        self.latest_intensity_image = intensity_image
        self.latest_valid_mask = valid_mask
        self.latest_stats = stats
        self.last_receive_time = receive_time

    def parse_cloud(self, msg: PointCloud2) -> Dict[str, np.ndarray]:
        fields = {field.name: field for field in msg.fields}

        required = [
            "x",
            "y",
            "z",
            "intensity",
            "ring",
            "timestamp",
        ]
        missing = [name for name in required if name not in fields]
        if missing:
            raise ValueError(f"Missing fields: {missing}")

        count = int(msg.width * msg.height)
        step = int(msg.point_step)
        endian = ">" if msg.is_bigendian else "<"

        dtype = np.dtype(
            {
                "names": required,
                "formats": [
                    endian + "f4",
                    endian + "f4",
                    endian + "f4",
                    endian + "f4",
                    endian + "u2",
                    endian + "f8",
                ],
                "offsets": [
                    int(fields["x"].offset),
                    int(fields["y"].offset),
                    int(fields["z"].offset),
                    int(fields["intensity"].offset),
                    int(fields["ring"].offset),
                    int(fields["timestamp"].offset),
                ],
                "itemsize": step,
            }
        )

        data = np.frombuffer(
            msg.data,
            dtype=dtype,
            count=count,
        )

        xyz = np.empty((count, 3), dtype=np.float32)
        xyz[:, 0] = data["x"]
        xyz[:, 1] = data["y"]
        xyz[:, 2] = data["z"]

        return {
            "xyz": xyz,
            "intensity": data["intensity"].astype(
                np.float32,
                copy=False,
            ),
            "ring": data["ring"].astype(
                np.int32,
                copy=False,
            ),
            "timestamp": data["timestamp"].astype(
                np.float64,
                copy=False,
            ),
        }

    def azimuth_columns(self, xyz: np.ndarray) -> np.ndarray:
        azimuth_deg = np.degrees(
            np.arctan2(xyz[:, 1], xyz[:, 0])
        )

        azimuth_deg = (
            azimuth_deg - self.azimuth_zero_deg
        ) % 360.0

        if self.azimuth_clockwise:
            azimuth_deg = (-azimuth_deg) % 360.0

        columns = np.floor(
            azimuth_deg
            / 360.0
            * self.expected_columns
        ).astype(np.int32)

        return np.clip(
            columns,
            0,
            self.expected_columns - 1,
        )

    def sequence_columns(self, ring: np.ndarray) -> np.ndarray:
        columns = np.zeros(len(ring), dtype=np.int32)

        for ring_index in range(self.expected_rings):
            indices = np.where(ring == ring_index)[0]
            if len(indices) == 0:
                continue

            sequence = np.linspace(
                0,
                self.expected_columns - 1,
                len(indices),
            ).astype(np.int32)

            columns[indices] = sequence

        return columns

    def display_callback(self):
        if self.latest_range_image is None:
            return

        now = self.local_time_sec()
        stats = dict(self.latest_stats)
        stats["cloud_age_ms"] = (
            now - self.last_receive_time
        ) * 1000.0

        range_panel = self.render_range_image(
            self.latest_range_image
        )
        panels = [range_panel]

        if self.show_intensity:
            panels.append(
                self.render_intensity_image(
                    self.latest_intensity_image
                )
            )

        if self.show_validity:
            panels.append(
                self.render_validity_image(
                    self.latest_valid_mask
                )
            )

        image_stack = np.vstack(panels)
        info_panel = self.render_info_panel(
            stats,
            image_stack.shape[0],
        )
        dashboard = np.hstack([image_stack, info_panel])

        cv2.imshow(self.window_name, dashboard)
        key = cv2.waitKey(1) & 0xFF

        if key in (27, ord("q")):
            rclpy.shutdown()

        if now - self.last_print_time >= self.print_interval_sec:
            self.last_print_time = now
            self.print_stats(stats)

    def render_range_image(
        self,
        range_image: np.ndarray,
    ) -> np.ndarray:
        normalized = np.zeros_like(range_image, dtype=np.uint8)
        valid = np.isfinite(range_image)

        clipped = np.clip(
            range_image,
            self.min_range_m,
            self.max_range_m,
        )
        normalized[valid] = (
            255.0
            * (
                1.0
                - (
                    clipped[valid] - self.min_range_m
                )
                / (
                    self.max_range_m - self.min_range_m
                )
            )
        ).astype(np.uint8)

        colored = cv2.applyColorMap(
            normalized,
            cv2.COLORMAP_TURBO,
        )
        colored[~valid] = (0, 0, 0)

        return self.resize_image(
            colored,
            "RANGE IMAGE",
        )

    def render_intensity_image(
        self,
        intensity_image: np.ndarray,
    ) -> np.ndarray:
        valid = np.isfinite(intensity_image)
        output = np.zeros_like(intensity_image, dtype=np.uint8)

        if np.any(valid):
            values = intensity_image[valid]
            low = float(np.percentile(values, 2.0))
            high = float(np.percentile(values, 98.0))

            if high <= low:
                high = low + 1.0

            output[valid] = np.clip(
                255.0
                * (
                    intensity_image[valid] - low
                )
                / (high - low),
                0.0,
                255.0,
            ).astype(np.uint8)

        colored = cv2.applyColorMap(
            output,
            cv2.COLORMAP_VIRIDIS,
        )
        colored[~valid] = (0, 0, 0)

        return self.resize_image(
            colored,
            "INTENSITY IMAGE",
        )

    def render_validity_image(
        self,
        valid_mask: np.ndarray,
    ) -> np.ndarray:
        image = np.zeros(
            valid_mask.shape,
            dtype=np.uint8,
        )
        image[valid_mask] = 255

        colored = cv2.cvtColor(
            image,
            cv2.COLOR_GRAY2BGR,
        )

        return self.resize_image(
            colored,
            "VALIDITY MASK",
        )

    def resize_image(
        self,
        image: np.ndarray,
        title: str,
    ) -> np.ndarray:
        width = max(
            640,
            int(round(
                self.expected_columns * self.display_scale_x
            )),
        )
        height = max(
            220,
            int(round(
                self.expected_rings * self.display_scale_y
            )),
        )

        resized = cv2.resize(
            image,
            (width, height),
            interpolation=cv2.INTER_NEAREST,
        )

        cv2.rectangle(
            resized,
            (0, 0),
            (width - 1, 34),
            (245, 245, 245),
            -1,
        )
        cv2.putText(
            resized,
            title,
            (12, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (25, 25, 25),
            2,
            cv2.LINE_AA,
        )

        for ring in range(self.expected_rings):
            y = int(
                34
                + (
                    height - 35
                )
                * (
                    ring + 0.5
                )
                / self.expected_rings
            )
            cv2.putText(
                resized,
                f"{ring}",
                (4, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.30,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        for degree in (0, 90, 180, 270, 360):
            x = int(
                degree / 360.0 * (width - 1)
            )
            cv2.line(
                resized,
                (x, 34),
                (x, height - 1),
                (110, 110, 110),
                1,
            )
            cv2.putText(
                resized,
                f"{degree}deg",
                (min(width - 52, x + 3), 48),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.32,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        return resized

    def render_info_panel(
        self,
        stats: Dict[str, float],
        height: int,
    ) -> np.ndarray:
        width = 430
        panel = np.full(
            (height, width, 3),
            248,
            dtype=np.uint8,
        )

        lines = [
            "V4.4 XT16 RANGE IMAGE",
            "",
            f"Raw points       : {int(stats['raw_points'])}",
            f"Valid points     : {int(stats['valid_points'])}",
            f"Occupied cells   : {int(stats['occupied_cells'])}",
            f"Coverage         : {stats['coverage_percent']:.2f} %",
            f"Cell collisions  : {int(stats['collision_count'])}",
            "",
            f"Ring min/max     : {int(stats['ring_min'])} / {int(stats['ring_max'])}",
            f"Ring count       : {int(stats['ring_count'])}",
            f"Columns          : {self.expected_columns}",
            "",
            f"Timestamp span   : {stats['timestamp_span']:.6f} s",
            f"Unique timestamps: {int(stats['unique_timestamps'])}",
            "",
            f"Azimuth min/max  : {stats['azimuth_min']:.2f} / {stats['azimuth_max']:.2f}",
            f"Range min/max    : {stats['range_min']:.2f} / {stats['range_max']:.2f} m",
            "",
            f"Build time       : {stats['build_ms']:.2f} ms",
            f"Cloud age        : {stats['cloud_age_ms']:.2f} ms",
            "",
            "Ring distribution:",
        ]

        y = 30
        for index, line in enumerate(lines):
            scale = 0.64 if index == 0 else 0.46
            thickness = 2 if index == 0 else 1

            cv2.putText(
                panel,
                line,
                (18, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                scale,
                (35, 35, 35),
                thickness,
                cv2.LINE_AA,
            )
            y += 25 if index == 0 else 22

        for ring_index in range(self.expected_rings):
            count = int(
                stats.get(
                    f"ring_{ring_index}_count",
                    0.0,
                )
            )
            cv2.putText(
                panel,
                f"Ring {ring_index:02d}: {count:5d}",
                (
                    18 + (ring_index // 8) * 180,
                    y + (ring_index % 8) * 21,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.43,
                (55, 55, 55),
                1,
                cv2.LINE_AA,
            )

        return panel

    def print_stats(self, stats: Dict[str, float]):
        self.get_logger().info(
            "V4.4 | "
            f"valid={int(stats['valid_points'])} | "
            f"cells={int(stats['occupied_cells'])} | "
            f"coverage={stats['coverage_percent']:.2f}% | "
            f"collisions={int(stats['collision_count'])} | "
            f"rings={int(stats['ring_count'])} | "
            f"span={stats['timestamp_span']:.6f}s | "
            f"build={stats['build_ms']:.2f}ms | "
            f"age={stats['cloud_age_ms']:.2f}ms"
        )

    def local_time_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiDARVisionTopViewV44Node()

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

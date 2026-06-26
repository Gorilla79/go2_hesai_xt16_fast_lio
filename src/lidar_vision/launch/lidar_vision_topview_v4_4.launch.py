#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="lidar_vision",
                executable="lidar_vision_topview_v4_4_node",
                name="lidar_vision_topview_v4_4_node",
                output="screen",
                parameters=[
                    {
                        "lidar_topic": "/lidar_points",
                        "expected_rings": 16,
                        "expected_columns": 4000,

                        "min_range_m": 0.30,
                        "max_range_m": 20.0,
                        "column_mode": "azimuth",
                        "azimuth_zero_deg": 0.0,
                        "azimuth_clockwise": False,
                        "keep_nearest_on_collision": True,

                        "display_scale_x": 0.25,
                        "display_scale_y": 18.0,
                        "display_fps": 10.0,
                        "window_name": "LiDAR V4.4 XT16 Range Image",
                        "show_intensity": True,
                        "show_validity": True,
                        "print_interval_sec": 2.0,
                    }
                ],
            )
        ]
    )

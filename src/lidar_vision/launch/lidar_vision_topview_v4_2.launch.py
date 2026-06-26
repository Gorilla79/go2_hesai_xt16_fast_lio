#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="lidar_vision",
                executable="lidar_vision_topview_v4_2_node",
                name="lidar_vision_topview_v4_2_node",
                output="screen",
                parameters=[
                    {
                        # Topics
                        "lidar_topic": "/lidar_points",
                        "sportmode_topic": "/sportmodestate",

                        # Fixed Hesai XT16 -> Go2 body extrinsic
                        "lidar_base_x": 0.150,
                        "lidar_base_y": 0.000,
                        "lidar_base_z": 0.250,
                        "lidar_roll_deg": -6.0,
                        "lidar_pitch_deg": 0.0,
                        "lidar_yaw_deg": 90.0,

                        # Gravity alignment
                        "enable_gravity_alignment": True,
                        "imu_roll_sign": 1.0,
                        "imu_pitch_sign": 1.0,
                        "imu_roll_offset_deg": 0.0,
                        "imu_pitch_offset_deg": 0.0,
                        "imu_timeout_sec": 0.20,
                        "imu_lowpass_alpha": 0.15,

                        # Point processing
                        "min_range": 0.35,
                        "max_range": 10.0,
                        "min_z": -1.50,
                        "max_z": 2.50,
                        "self_x_min": -0.70,
                        "self_x_max": 0.90,
                        "self_y_min": -0.60,
                        "self_y_max": 0.60,
                        "self_z_min": -0.80,
                        "self_z_max": 0.80,
                        "voxel_size": 0.05,
                        "max_display_points": 18000,

                        # Ground-plane estimation
                        "ground_fit_min_range": 0.80,
                        "ground_fit_max_range": 5.00,
                        "ground_fit_z_min": -1.20,
                        "ground_fit_z_max": 0.25,
                        "ground_fit_max_points": 5000,
                        "ground_inlier_threshold": 0.08,
                        "ground_refine_iterations": 3,
                        "obstacle_ground_clearance": 0.12,

                        # BEV and diagnostics
                        "display_range_m": 6.0,
                        "panel_size": 640,
                        "display_fps": 5.0,
                        "sector_half_angle_deg": 18.0,
                        "sector_min_height": 0.10,
                        "sector_max_height": 2.00,
                        "print_interval_sec": 1.0,
                        "window_name": "LiDAR V4.2 Coordinate Validation",
                        "draw_sector_nearest_markers": False,
                        "enable_click_coordinate_probe": True,
                        "click_probe_radius_m": 0.20,
                        "click_probe_min_points": 3,
                    }
                ],
            )
        ]
    )

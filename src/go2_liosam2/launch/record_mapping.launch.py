#!/usr/bin/env python3
"""
record_mapping.launch.py
========================
Go2 로봇에서 LIO-SAM2용 rosbag을 녹화하는 launch 파일.

녹화 토픽:
  /lidar_points          — Hesai XT16 포인트클라우드
  /utlidar/imu_corrected — Go2 내장 IMU (보정값)
  /utlidar/imu           — Go2 내장 IMU (raw, 백업)
  /tf                    — 동적 TF
  /tf_static             — 정적 TF

사용법:
  ros2 launch go2_liosam2 record_mapping.launch.py
  ros2 launch go2_liosam2 record_mapping.launch.py bag_name:=my_run save_dir:=/data/bags
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration


def _bag_record_action(context, *args, **kwargs):
    save_dir = LaunchConfiguration("save_dir").perform(context)
    bag_name = LaunchConfiguration("bag_name").perform(context)

    if not bag_name:
        bag_name = "mapping_" + datetime.now().strftime("%Y%m%d_%H%M%S")

    bag_path = os.path.join(save_dir, bag_name)

    return [
        LogInfo(msg=f"[go2_liosam2] Saving bag → {bag_path}"),
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "/lidar_points",
                "/utlidar/imu_corrected",
                "/utlidar/imu",
                "/tf",
                "/tf_static",
                "--output",
                bag_path,
                "--compression-mode",
                "file",
                "--compression-format",
                "zstd",
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            # ------------------------------------------------------------------
            # Arguments
            # ------------------------------------------------------------------
            DeclareLaunchArgument(
                "save_dir",
                default_value=os.path.expanduser("~/mapping_bags"),
                description="Directory where the bag will be saved",
            ),
            DeclareLaunchArgument(
                "bag_name",
                default_value="",
                description=(
                    "Bag folder name. "
                    "Defaults to mapping_YYYYMMDD_HHMMSS when empty."
                ),
            ),
            # ------------------------------------------------------------------
            # Bag recorder
            # ------------------------------------------------------------------
            OpaqueFunction(function=_bag_record_action),
        ]
    )

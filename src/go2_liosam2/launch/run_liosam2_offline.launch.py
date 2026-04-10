#!/usr/bin/env python3
"""
run_liosam2_offline.launch.py
==============================
녹화된 rosbag을 재생하며 LIO-SAM2 SLAM을 오프라인으로 실행하는 launch 파일.

노드 구성:
  1. lio_sam::imageProjection         — 포인트클라우드 전처리 + ring/time 검증
  2. lio_sam::featureExtraction       — Edge / Planar feature 추출
  3. lio_sam::mapOptimization         — 인자 그래프 최적화 + Loop closure
  4. lio_sam::imuPreintegration       — IMU 전처리 + 초기화
  5. ros2 bag play (선택)             — bag_path 인자 제공 시 자동 재생

토픽 리맵:
  /lidar_points          → LIO-SAM이 기대하는 포인트클라우드
  /utlidar/imu_corrected → LIO-SAM이 기대하는 IMU 입력

사용법:
  # bag 없이 LIO-SAM만 실행 (별도 터미널에서 직접 bag play):
  ros2 launch go2_liosam2 run_liosam2_offline.launch.py

  # bag 자동 재생 포함:
  ros2 launch go2_liosam2 run_liosam2_offline.launch.py \\
      bag_path:=/home/unitree/mapping_bags/mapping_20250101_120000

  # 지도를 PCD로 저장:
  ros2 launch go2_liosam2 run_liosam2_offline.launch.py \\
      bag_path:=/home/unitree/mapping_bags/mapping_20250101_120000 \\
      save_pcd:=true  save_pcd_dir:=/home/unitree/maps
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _maybe_play_bag(context, *args, **kwargs):
    bag_path = LaunchConfiguration("bag_path").perform(context)
    if not bag_path:
        return []

    return [
        LogInfo(msg=f"[go2_liosam2] Playing bag: {bag_path}"),
        ExecuteProcess(
            cmd=["ros2", "bag", "play", bag_path, "--clock", "-r", "1.0"],
            output="screen",
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("go2_liosam2")
    params_file = os.path.join(pkg_share, "config", "params_go2_hesai_xt16.yaml")

    # ------------------------------------------------------------------
    # Arguments
    # ------------------------------------------------------------------
    args = [
        DeclareLaunchArgument(
            "bag_path",
            default_value="",
            description=(
                "Path to the rosbag directory to replay. "
                "Leave empty to skip automatic bag playback."
            ),
        ),
        DeclareLaunchArgument(
            "save_pcd",
            default_value="false",
            description="Set to true to save the final map as PCD files.",
        ),
        DeclareLaunchArgument(
            "save_pcd_dir",
            default_value="/tmp/go2_liosam2_map",
            description="Directory where PCD files will be saved when save_pcd=true.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description=(
                "Use simulation (bag) time. "
                "Should be true when replaying a bag with --clock."
            ),
        ),
    ]

    # ------------------------------------------------------------------
    # LIO-SAM nodes
    # All four nodes load the same YAML and apply topic remaps so they
    # receive the Go2/Hesai topics without patching the upstream package.
    # ------------------------------------------------------------------
    common_params = [
        params_file,
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
        {"savePCD": LaunchConfiguration("save_pcd")},
        {"savePCDDirectory": LaunchConfiguration("save_pcd_dir")},
    ]

    # Topic remaps: map Hesai/Go2 topics to LIO-SAM's expected names
    common_remaps = [
        ("/lidar_points",          "/lidar_points"),           # already correct
        ("/utlidar/imu_corrected", "/utlidar/imu_corrected"),  # already correct
    ]

    imu_preintegration_node = Node(
        package="lio_sam",
        executable="lio_sam_imuPreintegration",
        name="lio_sam_imuPreintegration",
        parameters=common_params,
        remappings=common_remaps,
        output="screen",
    )

    image_projection_node = Node(
        package="lio_sam",
        executable="lio_sam_imageProjection",
        name="lio_sam_imageProjection",
        parameters=common_params,
        remappings=common_remaps,
        output="screen",
    )

    feature_extraction_node = Node(
        package="lio_sam",
        executable="lio_sam_featureExtraction",
        name="lio_sam_featureExtraction",
        parameters=common_params,
        remappings=common_remaps,
        output="screen",
    )

    map_optimization_node = Node(
        package="lio_sam",
        executable="lio_sam_mapOptimization",
        name="lio_sam_mapOptimization",
        parameters=common_params,
        remappings=common_remaps,
        output="screen",
    )

    # ------------------------------------------------------------------
    # Bag playback (conditional on bag_path being set)
    # ------------------------------------------------------------------
    bag_action = OpaqueFunction(function=_maybe_play_bag)

    return LaunchDescription(
        args
        + [
            imu_preintegration_node,
            image_projection_node,
            feature_extraction_node,
            map_optimization_node,
            bag_action,
        ]
    )

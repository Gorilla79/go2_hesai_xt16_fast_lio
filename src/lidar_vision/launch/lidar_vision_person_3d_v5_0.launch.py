#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("lidar_vision")

    calibration_config = os.path.join(
        package_share,
        "config",
        "calibration_current.yaml",
    )
    perception_config = os.path.join(
        package_share,
        "config",
        "person_3d_v5.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value="yolo11s-seg.pt",
        ),
        DeclareLaunchArgument(
            "show_gui",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "enable_front_semantics",
            default_value="true",
        ),
        Node(
            package="lidar_vision",
            executable="lidar_vision_person_3d_v5_0_node",
            name="lidar_vision_person_3d_v5_0",
            output="screen",
            parameters=[
                calibration_config,
                perception_config,
                {
                    "model_path": LaunchConfiguration("model_path"),
                    "show_gui": LaunchConfiguration("show_gui"),
                    "enable_front_semantics": LaunchConfiguration(
                        "enable_front_semantics"
                    ),
                },
            ],
        ),
    ])

#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lidar_vision",
            executable="go2_calibration_verify_adjust",
            name="go2_calibration_verify_adjust",
            output="screen",
            arguments=[
                "--width", "640",
                "--height", "480",
                "--camera-fps", "30",

                "--lidar-topic", "/lidar_points",
                "--lidar-points-frame", "lidar",

                "--camera-base-x", "0.340",
                "--camera-base-y", "0.040",
                "--camera-base-z", "0.390",
                "--camera-roll-deg", "0.0",
                "--camera-pitch-deg", "0.0",
                "--camera-yaw-deg", "-0.5",

                "--lidar-base-x", "0.150",
                "--lidar-base-y", "0.000",
                "--lidar-base-z", "0.250",
                "--lidar-roll-deg", "-6.0",
                "--lidar-pitch-deg", "0.0",
                "--lidar-yaw-deg", "90.0",

                "--lidar-x-min", "0.10",
                "--lidar-x-max", "8.00",
                "--lidar-y-min", "-4.00",
                "--lidar-y-max", "4.00",
                "--lidar-z-min", "-1.50",
                "--lidar-z-max", "3.00",

                "--depth-min", "0.20",
                "--depth-max", "6.00",
                "--rs-stride", "4",

                "--max-rs-points", "30000",
                "--max-lidar-points", "30000",

                "--save-path",
                "/home/unitree/go2_ws/src/lidar_vision/config/calibration_current.json",
            ],
        )
    ])

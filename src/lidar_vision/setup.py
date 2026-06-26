from setuptools import setup
from glob import glob
import os

package_name = "lidar_vision"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "docs"), glob("docs/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="unitree",
    maintainer_email="leedongsup9149@gmail.com",
    description="Recovered LiDAR_Vision package for Go2, RealSense, YOLO, Hesai XT16, BEV, motion and range-image diagnostics.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lidar_vision_topview_node = lidar_vision.lidar_vision_topview_v2_node:main",
            "lidar_vision_topview_v1_node = lidar_vision.lidar_vision_topview_v1_node:main",
            "lidar_vision_topview_v2_node = lidar_vision.lidar_vision_topview_v2_node:main",
            "lidar_vision_topview_v2_node_final = lidar_vision.lidar_vision_topview_v2_node_final:main",
            "lidar_vision_topview_v3_node = lidar_vision.lidar_vision_topview_v3_node:main",
            "lidar_vision_topview_v3_1_node = lidar_vision.lidar_vision_topview_v3_1_node:main",
            "lidar_vision_topview_v3_2_node = lidar_vision.lidar_vision_topview_v3_2_node:main",
            "lidar_vision_topview_v3_3_node = lidar_vision.lidar_vision_topview_v3_3_node:main",
            "lidar_vision_topview_v3_4_node = lidar_vision.lidar_vision_topview_v3_4_node:main",
            "lidar_vision_topview_v3_5_node = lidar_vision.lidar_vision_topview_v3_5_node:main",
            "lidar_vision_topview_v3_5_node_xt16_optimized = lidar_vision.lidar_vision_topview_v3_5_node_xt16_optimized:main",
            "lidar_vision_topview_v3_6_node = lidar_vision.lidar_vision_topview_v3_6_node:main",
            "lidar_vision_topview_v4_node = lidar_vision.lidar_vision_topview_v4_node:main",
            "lidar_vision_topview_v4_node_corrected = lidar_vision.lidar_vision_topview_v4_node_corrected:main",
            "lidar_vision_topview_v4_1_node = lidar_vision.lidar_vision_topview_v4_1_node:main",
            "lidar_vision_topview_v4_1_1_node = lidar_vision.lidar_vision_topview_v4_1_1_node:main",
            "lidar_vision_topview_v4_2_node = lidar_vision.lidar_vision_topview_v4_2_node:main",
            "lidar_vision_topview_v4_3_node = lidar_vision.lidar_vision_topview_v4_3_node:main",
            "lidar_vision_topview_v4_3_optimized_node = lidar_vision.lidar_vision_topview_v4_3_optimized_node:main",
            "lidar_vision_topview_v4_4_node = lidar_vision.lidar_vision_topview_v4_4_node:main",
            "go2_calibration_verify_adjust = lidar_vision.go2_calibration_verify_adjust:main",
            "go2_realsense_lidar_3d_calib = lidar_vision.go2_realsense_lidar_3d_calib:main",
            "go2_realsense_lidar_depth_calib = lidar_vision.go2_realsense_lidar_depth_calib:main",
            "go2_realsense_yolo11s_seg_topview = lidar_vision.go2_realsense_yolo11s_seg_topview:main",
            "go2_realsense_yolo11s_seg_lidar_topview_v2 = lidar_vision.go2_realsense_yolo11s_seg_lidar_topview_v2:main",
            "lidar_vision_person_3d_v5_0_node = lidar_vision.lidar_vision_person_3d_v5_0_node:main",
        ],
    },
)

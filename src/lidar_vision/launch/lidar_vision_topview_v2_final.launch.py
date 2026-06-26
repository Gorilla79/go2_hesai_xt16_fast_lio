from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    model = LaunchConfiguration("model")
    target_fps = LaunchConfiguration("target_fps")
    device = LaunchConfiguration("device")
    calibration_config = LaunchConfiguration("calibration_config")

    default_calibration_config = PathJoinSubstitution([
        FindPackageShare("lidar_vision"),
        "config",
        "calibration_current.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value="yolo11s-seg.pt",
            description="YOLO segmentation model path"
        ),
        DeclareLaunchArgument(
            "target_fps",
            default_value="10.0",
            description="Target processing FPS"
        ),
        DeclareLaunchArgument(
            "device",
            default_value="0",
            description="YOLO device, e.g. 0 or cpu"
        ),
        DeclareLaunchArgument(
            "calibration_config",
            default_value=default_calibration_config,
            description="Calibration YAML file path"
        ),

        Node(
            package="lidar_vision",
            executable="lidar_vision_topview_v2_node",
            name="lidar_vision_topview_v2_node",
            output="screen",
            parameters=[
                calibration_config,
                {
                    "model": model,
                    "device": device,
                    "half": True,
                    "imgsz": 640,
                    "conf": 0.35,
                    "iou": 0.55,
                    "target_fps": target_fps,

                    "camera_width": 640,
                    "camera_height": 480,
                    "camera_fps": 30,
                    "image_rotate": "none",

                    "lidar_topic": "/lidar_points",
                    "lidar_points_frame": "lidar",

                    "lidar_x_min": 0.20,
                    "lidar_x_max": 8.00,
                    "lidar_y_min": -4.00,
                    "lidar_y_max": 4.00,
                    "lidar_z_min": -1.20,
                    "lidar_z_max": 2.50,

                    "max_lidar_points_process": 12000,
                    "max_lidar_points_topview": 1800,
                    "skip_lidar_projection_if_points_over": 20000,

                    "min_lidar_points_in_mask": 3,
                    "mask_dilate_px": 3,
                    "person_outlier_radius": 0.45,
                    "near_cluster_depth_m": 0.45,
                    "fixed_distance": 2.0,

                    "topview_size": 800,
                    "meter_to_pixel": 90.0,
                    "max_forward_m": 6.0,
                    "max_side_m": 3.0,

                    "draw_lidar_topview": True,
                    "draw_camera_window": True,
                    "draw_topview_window": True,
                    "show_mask_overlay": True,

                    # V2: LiDAR association confidence thresholds
                    "confidence_high_count": 30,
                    "confidence_medium_count": 10,
                    "confidence_high_spread": 0.45,
                    "confidence_medium_spread": 0.70,
                    "confidence_high_depth_std": 0.25,
                    "confidence_medium_depth_std": 0.50,
                }
            ]
        ),
    ])

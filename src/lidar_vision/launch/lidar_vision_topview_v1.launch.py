from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model = LaunchConfiguration("model")
    target_fps = LaunchConfiguration("target_fps")
    device = LaunchConfiguration("device")

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

        Node(
            package="lidar_vision",
            executable="lidar_vision_topview_node",
            name="lidar_vision_topview_node",
            output="screen",
            parameters=[{
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

                # Latest calibration
                "camera_base_x": 0.340,
                "camera_base_y": 0.040,
                "camera_base_z": 0.390,
                "camera_roll_deg": 0.0,
                "camera_pitch_deg": 0.0,
                "camera_yaw_deg": -0.5,

                "lidar_base_x": 0.150,
                "lidar_base_y": 0.000,
                "lidar_base_z": 0.250,
                "lidar_roll_deg": -6.0,
                "lidar_pitch_deg": 0.0,
                "lidar_yaw_deg": 90.0,

                "lidar_x_min": 0.20,
                "lidar_x_max": 8.00,
                "lidar_y_min": -4.00,
                "lidar_y_max": 4.00,
                "lidar_z_min": -1.20,
                "lidar_z_max": 2.50,

                # Important for latency reduction
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
            }]
        ),
    ])

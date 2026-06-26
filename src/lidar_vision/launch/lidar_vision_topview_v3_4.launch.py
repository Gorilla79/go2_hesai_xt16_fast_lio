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
            executable="lidar_vision_topview_v3_4_node",
            name="lidar_vision_topview_v3_4_node",
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
                    "lidar_x_max": 12.00,
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
                    "meter_to_pixel": 60.0,
                    "max_forward_m": 10.0,
                    "max_side_m": 3.5,

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
                    # V3: tracking memory, smoothing, and short-term persistence
                    "enable_tracking": True,
                    "track_match_distance": 0.80,
                    "smoothing_alpha": 0.35,
                    "max_missing_frames": 5,
                    "stale_track_color_b": 160,
                    "stale_track_color_g": 160,
                    "stale_track_color_r": 160,

                    # V3.1: duplicate detection and duplicate track suppression
                    "enable_duplicate_filter": True,
                    "duplicate_detection_distance": 0.45,
                    "duplicate_track_distance": 0.55,
                    "duplicate_bbox_iou": 0.45,
                    "stale_active_suppression_distance": 0.80,
                    "stale_control_max_missing": 1,

                    # V3.2: false-positive suppression and control-count filtering
                    "enable_false_positive_filter": True,
                    "exclude_fallback_from_control": True,
                    "allow_low_confidence_close_control": True,
                    "low_control_max_distance": 2.00,
                    "low_control_min_hits": 3,
                    "fallback_control_max_distance": 1.20,
                    "fallback_control_min_hits": 6,
                    "reflection_min_distance": 1.50,
                    "min_control_bbox_height_px": 18,
                    "min_control_bbox_area_px": 180,
                    "far_distance_threshold": 3.00,
                    "very_far_distance_threshold": 5.00,

                    # V3.3: LiDAR-only obstacle and free-space layer
                    "enable_lidar_obstacle_layer": True,
                    "draw_lidar_obstacle_layer": True,
                    "obstacle_x_min": 0.20,
                    "obstacle_x_max": 10.00,
                    "obstacle_y_min": -3.00,
                    "obstacle_y_max": 3.00,
                    "obstacle_z_min": -0.20,
                    "obstacle_z_max": 1.80,
                    "front_corridor_half_width": 0.80,
                    "side_sector_x_min": 0.50,
                    "side_sector_x_max": 5.00,
                    "side_sector_y_min_abs": 0.60,
                    "side_sector_y_max_abs": 3.00,
                    "front_stop_distance": 1.20,
                    "front_slow_distance": 4.00,
                    "front_caution_distance": 8.00,
                    "min_obstacle_points_for_risk": 5,
                    "obstacle_draw_max_points": 2500,

                    # V3.4: LiDAR cluster candidate layer
                    "enable_lidar_cluster_layer": True,
                    "draw_lidar_clusters": True,
                    "cluster_grid_resolution": 0.25,
                    "cluster_min_points": 4,
                    "cluster_max_points": 350,
                    "cluster_min_distance": 0.80,
                    "cluster_max_distance": 10.00,
                    "cluster_min_width": 0.10,
                    "cluster_max_width": 1.40,
                    "cluster_min_depth": 0.10,
                    "cluster_max_depth": 1.60,
                    "cluster_min_height": 0.10,
                    "cluster_max_height": 2.20,
                    "cluster_structure_width": 2.00,
                    "cluster_structure_depth": 2.00,
                    "cluster_human_like_max_distance": 10.00,
                    "cluster_association_distance": 0.90,
                    "cluster_candidate_control_distance": 6.00,
                    "cluster_max_draw_count": 12,

                }
            ]
        ),
    ])

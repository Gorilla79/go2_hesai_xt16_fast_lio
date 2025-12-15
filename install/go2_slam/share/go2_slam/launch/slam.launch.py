from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # slam.yaml 경로
    slam_yaml_path = os.path.join(
        get_package_share_directory('go2_slam'),
        'config',
        'slam.yaml'
    )

    return LaunchDescription([

        # 1. Hesai PointCloud -> LaserScan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc_to_scan_hesai',
            output='screen',
            remappings=[
                ('cloud_in', '/lidar_points'),
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'base_link',
                'min_height': -0.2,
                'max_height': 0.2,
                'range_min': 0.1,
                'range_max': 30.0,
                'use_inf': True,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.00349
            }]
        ),

        # 2. SLAM Toolbox
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc_to_scan_hesai',
            remappings=[
                ('cloud_in', '/lidar_points'),
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'base_link',

                'min_height': -0.2,
                'max_height': 0.2,

                'range_min': 0.1,
                'range_max': 30.0,
                'use_inf': True,

                # 🔴 핵심
                'angle_min': -3.141592,
                'angle_max':  3.141592,
                'angle_increment': 0.0174533   # 1 deg → 정확히 360 beams
            }]
        )
    ])


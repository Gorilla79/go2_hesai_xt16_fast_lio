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
                ('cloud_in', '/merged_pointcloud'),
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'base_link',
                # widened height range to include more returns
                'min_height': -1.0,
                'max_height': 1.0,
                'range_min': 0.1,
                'range_max': 30.0,
                'use_inf': True,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                # 1 degree resolution (approx) to produce a full 360 scan
                'angle_increment': 0.0174533
            }]
        ),

        # 2. SLAM Toolbox node (load parameters from slam.yaml)
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_yaml_path]
        )
    ])


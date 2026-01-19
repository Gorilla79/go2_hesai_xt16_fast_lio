from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    slam_yaml_path = os.path.join(
        get_package_share_directory('go2_slam'),
        'config',
        'slam.yaml'
    )

    return LaunchDescription([
        # Merge the two pointcloud sources into /merged_pointcloud
        Node(
            package='go2_slam',
            executable='merge_pointclouds',
            name='merge_pointclouds',
            output='screen',
            parameters=[
                {'topic_a': '/pointcloud'},
                {'topic_b': '/lidar_points'},
                {'output_topic': '/merged_pointcloud'}
            ]
        ),
        # Start a pointcloud_to_laserscan converter that listens to the merged cloud
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc_to_scan_merged',
            output='screen',
            remappings=[('cloud_in', '/merged_pointcloud'), ('scan', '/scan')],
            parameters=[{
                'target_frame': 'base_link',
                'min_height': -1.0,
                'max_height': 1.0,
                'range_min': 0.1,
                'range_max': 30.0,
                'use_inf': True,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0174533
            }]
        ),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_yaml_path]
        )
    ])

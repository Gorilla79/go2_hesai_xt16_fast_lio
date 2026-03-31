from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_3d_bbox')
    default_rviz = os.path.join(pkg_share, 'rviz', 'lidar_3d_bbox.rviz')

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/lidar_points',
        description='Hesai XT16 PointCloud2 topic'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='RViz config file'
    )

    bbox_node = Node(
        package='lidar_3d_bbox',
        executable='lidar_3d_bbox_node',
        name='lidar_3d_human_bbox_node',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'marker_topic': '/lidar_3d_bboxes',

            'min_x': -6.0,
            'max_x': 12.0,
            'min_y': -6.0,
            'max_y': 6.0,
            'min_z': -2.0,
            'max_z': 2.5,

            'process_every_n': 2,
            'voxel_leaf': 0.10,
            'max_points_after_voxel': 3500,

            'ground_percentile': 8.0,
            'ground_margin': 0.18,

            'dbscan_eps': 0.42,
            'dbscan_min_samples': 6,
            'cluster_min_points': 8,
            'cluster_max_points': 220,

            'human_min_height': 0.45,
            'human_max_height': 2.2,
            'human_min_width': 0.12,
            'human_max_width': 1.20,
            'human_min_length': 0.12,
            'human_max_length': 1.20,
            'human_min_points': 8,
            'human_max_points': 220,
            'human_max_distance': 10.0,

            'marker_lifetime': 0.30,
            'line_width': 0.05,
            'text_height': 0.28,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    return LaunchDescription([
        input_topic_arg,
        rviz_config_arg,
        bbox_node,
        rviz_node
    ])

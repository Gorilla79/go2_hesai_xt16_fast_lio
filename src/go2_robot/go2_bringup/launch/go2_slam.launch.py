from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',   # radar or converted hesai
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        slam_node
    ])


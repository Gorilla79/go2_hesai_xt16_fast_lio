import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    fast_lio_dir = get_package_share_directory('fast_lio')
    bag_path = LaunchConfiguration('bag_path')

    return LaunchDescription([
        DeclareLaunchArgument('bag_path', description='Path to rosbag2 directory'),

        # FAST-LIO
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laserMapping',
            parameters=[
                os.path.join(fast_lio_dir, 'config', 'hesaixt16.yaml'),
                {'use_sim_time': False}
            ],
            output='screen'
        ),

        # bag 재생 (3초 후)
        TimerAction(period=3.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', bag_path, '--rate', '0.3'],
                output='screen'
            ),
        ]),
    ])

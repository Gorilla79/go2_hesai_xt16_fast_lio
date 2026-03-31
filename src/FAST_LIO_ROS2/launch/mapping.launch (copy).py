import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    fast_lio_pkg = get_package_share_directory('fast_lio')
    description_pkg = get_package_share_directory('go2_description')

    urdf_path = os.path.join(description_pkg, 'urdf', 'go2_description.urdf')
    rviz_config_path = os.path.join(fast_lio_pkg, 'rviz', 'fastlio.rviz')
    yaml_config_path = os.path.join(fast_lio_pkg, 'config', 'hesaixt16.yaml')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['python3', os.path.expanduser('~/go2_ws/src/FAST_LIO_ROS2/scripts/imu_correct.py')],
            output='screen'
        ),

        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laser_mapping',
            parameters=[yaml_config_path],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_init_to_map_viz',
            arguments=['0', '0', '0', '1', '0', '0', '0', 'camera_init', 'map_viz'],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])

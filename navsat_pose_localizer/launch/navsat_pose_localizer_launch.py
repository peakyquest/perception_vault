
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navsat_pose_localizer',
            executable='navsat_pose_localizer_node',
            name='navsat_pose_localizer_node',
            output='screen',
            parameters=[],
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_image_processing',
            executable='image_rectifier',
            name='image_rectifier',
            output='screen',
            parameters=[{
                'input_image_topic': '/airsim_node/Car1/camera_link/Scene',
                'input_camera_info_topic': '/airsim_node/Car1/camera_link/Scene/camera_info',
                'output_image_topic': '/image_rect',
                'queue_size': 10,
            }],
        ),
    ])


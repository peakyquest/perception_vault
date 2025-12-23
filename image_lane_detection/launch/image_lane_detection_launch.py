
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_dataset = LaunchConfiguration('use_dataset')
    debug = LaunchConfiguration('debug')
    dataset_file = LaunchConfiguration('dataset_file')
    image_topic = LaunchConfiguration('image_topic')

    return LaunchDescription([
        # Whether to use simulation time (e.g. from Gazebo)
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Whether to read frames from dataset video or subscribe to an image topic
        DeclareLaunchArgument(
            'use_dataset',
            default_value='true',
            description='If true, read from dataset video; if false, subscribe to image_topic'
        ),

        # Enable/disable debug image publishing
        DeclareLaunchArgument(
            'debug',
            default_value='true',
            description='Enable debug image publishing'
        ),

        # Name of the dataset video file under the package resources folder
        DeclareLaunchArgument(
            'dataset_file',
            default_value='dataset_3.mp4',
            description='Name of the dataset video file located in the package resources folder.'
        ),

        # Input image topic when use_dataset is false
        DeclareLaunchArgument(
            'image_topic',
            default_value='camera/image_raw',
            description='Input image topic when use_dataset is false'
        ),

        Node(
            package='image_lane_detection',
            executable='image_lane_detection_node',
            name='image_lane_detection_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'use_dataset': use_dataset,
                'debug': debug,
                'dataset_file': dataset_file,
                'image_topic': image_topic,
            }],
        ),
    ])

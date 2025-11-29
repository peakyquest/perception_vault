
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_euclidean_cluster',
            executable='lidar_euclidean_cluster_node',
            name='lidar_euclidean_cluster_node',
            output='screen',
            parameters=[{
                'input_topic': '/points/filtered',
                'output_topic': '/clusters',
                'cluster_tolerance': 0.5,
                'min_cluster_size': 30,
                'max_cluster_size': 5000,
            }],
        ),
    ])

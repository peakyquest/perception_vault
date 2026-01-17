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
                # Topic parameters
                'input_topic': '/points/filtered',
                'output_topic': '/cluster_points',
                'marker_topic': '/cluster_markers',
                
                # Clustering parameters
                'cluster_tolerance': 0.6,  # Balanced: prevents merging while keeping clusters (distance threshold in meters)
                'min_cluster_size': 30,  # Reduced from 30 to allow smaller valid clusters
                'max_cluster_size': 2000,
                
                # Visualization and debugging
                'publish_markers': True,
                'verbose': True,  # Enable verbose for debugging
            }],
        ),
    ])

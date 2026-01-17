from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_rule_based_classifier',
            executable='lidar_rule_based_classifier_node',
            name='lidar_rule_based_classifier_node',
            output='screen',
            parameters=[{
                # Topic parameters
                'cluster_points_topic': '/cluster_points',
                'cluster_markers_topic': '/cluster_markers',
                'cluster_classifier_marker_topic': '/cluster_classifier_marker',
                
                # Detection parameters
                'max_detection_distance': 50.0,  # Maximum distance for detection (meters)
                'strict_distance_threshold': 30.0,  # Distance beyond which rules become stricter
                
                # Detection classes (list of strings: "person", "car")
                'detect_classes': ['person', 'car'],
                
                # Debugging
                'verbose': False,
            }],
        ),
    ])

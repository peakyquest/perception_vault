from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/image_rect',
        description='Input rectified image topic'
    )
    
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/airsim_node/Car1/lidar/laser_link',
        description='Input point cloud topic'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/airsim_node/Car1/camera_link/Scene/camera_info',
        description='Camera info topic'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/points/colored',
        description='Output colored point cloud topic'
    )
    
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_link_optical/static',
        description='Camera frame ID'
    )
    
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value='Car1/laser_link',
        description='LiDAR frame ID'
    )
    
    queue_size_arg = DeclareLaunchArgument(
        'queue_size',
        default_value='10',
        description='Queue size for subscribers'
    )
    
    use_approximate_sync_arg = DeclareLaunchArgument(
        'use_approximate_sync',
        default_value='true',
        description='Use approximate time synchronization'
    )
    
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='100.0',
        description='Maximum range for point projection (meters)'
    )
    
    detections_topic_arg = DeclareLaunchArgument(
        'detections_topic',
        default_value='/detections',
        description='YOLO detections topic (yolo_msgs/DetectionArray)'
    )
    
    markers_topic_arg = DeclareLaunchArgument(
        'markers_topic',
        default_value='/detection_bboxes_3d',
        description='3D bounding box markers topic'
    )
    
    enable_3d_bboxes_arg = DeclareLaunchArgument(
        'enable_3d_bboxes',
        default_value='true',
        description='Enable 3D bounding box visualization for detections'
    )
    
    bbox_min_height_arg = DeclareLaunchArgument(
        'bbox_min_height',
        default_value='0.1',
        description='Minimum height above ground for points in bounding box (meters)'
    )
    
    bbox_outlier_percentile_arg = DeclareLaunchArgument(
        'bbox_outlier_percentile',
        default_value='0.95',
        description='Percentile threshold for removing outlier points (0.0-1.0)'
    )
    
    bbox_remove_ground_arg = DeclareLaunchArgument(
        'bbox_remove_ground',
        default_value='true',
        description='Remove ground points from bounding box computation'
    )
    
    bbox_max_dimension_arg = DeclareLaunchArgument(
        'bbox_max_dimension',
        default_value='10.0',
        description='Maximum dimension for bounding box (meters) - used as hard limit'
    )
    
    bbox_use_tight_fit_arg = DeclareLaunchArgument(
        'bbox_use_tight_fit',
        default_value='true',
        description='Use tight fitting algorithm for bounding boxes'
    )

    return LaunchDescription([
        image_topic_arg,
        pointcloud_topic_arg,
        camera_info_topic_arg,
        output_topic_arg,
        camera_frame_arg,
        lidar_frame_arg,
        queue_size_arg,
        use_approximate_sync_arg,
        max_range_arg,
        detections_topic_arg,
        markers_topic_arg,
        enable_3d_bboxes_arg,
        bbox_min_height_arg,
        bbox_outlier_percentile_arg,
        bbox_remove_ground_arg,
        bbox_max_dimension_arg,
        bbox_use_tight_fit_arg,
        Node(
            package='pixel_cloud_fusion',
            executable='pixel_cloud_fusion_node',
            name='pixel_cloud_fusion_node',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'lidar_frame': LaunchConfiguration('lidar_frame'),
                'queue_size': LaunchConfiguration('queue_size'),
                'use_approximate_sync': LaunchConfiguration('use_approximate_sync'),
                'max_range': LaunchConfiguration('max_range'),
                'detections_topic': LaunchConfiguration('detections_topic'),
                'markers_topic': LaunchConfiguration('markers_topic'),
                'enable_3d_bboxes': LaunchConfiguration('enable_3d_bboxes'),
                'bbox_min_height': LaunchConfiguration('bbox_min_height'),
                'bbox_outlier_percentile': LaunchConfiguration('bbox_outlier_percentile'),
                'bbox_remove_ground': LaunchConfiguration('bbox_remove_ground'),
                'bbox_max_dimension': LaunchConfiguration('bbox_max_dimension'),
                'bbox_use_tight_fit': LaunchConfiguration('bbox_use_tight_fit'),
            }],
        ),
    ])

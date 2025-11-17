from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Voxel grid parameters
    voxel_input_topic = LaunchConfiguration('voxel_input_topic')
    voxel_output_topic = LaunchConfiguration('voxel_output_topic')
    leaf_size_x = LaunchConfiguration('leaf_size_x')
    leaf_size_y = LaunchConfiguration('leaf_size_y')
    leaf_size_z = LaunchConfiguration('leaf_size_z')
    voxel_debug = LaunchConfiguration('voxel_debug')
    
    # Ground segmentation parameters
    ground_input_topic = LaunchConfiguration('ground_input_topic')
    ground_output_topic = LaunchConfiguration('ground_output_topic')
    ground_threshold = LaunchConfiguration('ground_threshold')
    radial_div_num = LaunchConfiguration('radial_div_num')
    concentric_div_num = LaunchConfiguration('concentric_div_num')
    max_range = LaunchConfiguration('max_range')
    min_range = LaunchConfiguration('min_range')
    ground_debug = LaunchConfiguration('ground_debug')
    
    # Outliers filter parameters
    outliers_mean_k = LaunchConfiguration('outliers_mean_k')
    outliers_stddev = LaunchConfiguration('outliers_stddev')
    outliers_debug = LaunchConfiguration('outliers_debug')

    return LaunchDescription([
        # Global arguments
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        
        # Voxel grid arguments
        DeclareLaunchArgument('voxel_input_topic', 
                             default_value='/cx/lslidar_point_cloud',
                             description='Input topic for voxel grid filter'),
        DeclareLaunchArgument('voxel_output_topic', 
                             default_value='/points/voxel_filter',
                             description='Output topic for voxel grid filter'),
        DeclareLaunchArgument('leaf_size_x', default_value='0.1', description='Voxel leaf size X (m)'),
        DeclareLaunchArgument('leaf_size_y', default_value='0.1', description='Voxel leaf size Y (m)'),
        DeclareLaunchArgument('leaf_size_z', default_value='0.1', description='Voxel leaf size Z (m)'),
        DeclareLaunchArgument('voxel_debug', default_value='false', description='Enable voxel grid debug output'),
        
        # Ground segmentation arguments
        DeclareLaunchArgument('ground_input_topic', 
                             default_value='/points/voxel_filter',
                             description='Input topic for ground segmentation'),
        DeclareLaunchArgument('ground_output_topic', 
                             default_value='/points/no_ground',
                             description='Output topic for ground segmentation'),
        DeclareLaunchArgument('ground_threshold', default_value='0.2', description='Ground threshold (m)'),
        DeclareLaunchArgument('radial_div_num', default_value='60.0', description='Radial division number'),
        DeclareLaunchArgument('concentric_div_num', default_value='30.0', description='Concentric division number'),
        DeclareLaunchArgument('max_range', default_value='50.0', description='Maximum range (m)'),
        DeclareLaunchArgument('min_range', default_value='2.0', description='Minimum range (m)'),
        DeclareLaunchArgument('ground_debug', default_value='false', description='Enable ground segmentation debug output'),
        
        # Outliers filter arguments
        DeclareLaunchArgument('outliers_mean_k', default_value='20', description='Mean K for outlier removal'),
        DeclareLaunchArgument('outliers_stddev', default_value='1.5', description='Standard deviation multiplier threshold'),
        DeclareLaunchArgument('outliers_debug', default_value='false', description='Enable outliers filter debug output'),

        # Voxel Grid Node
        Node(
            package='lidar_pointcloud_filters',
            executable='voxel_grid_node',
            name='voxel_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': voxel_input_topic,
                'output_topic': voxel_output_topic,
                'leaf_size_x': leaf_size_x,
                'leaf_size_y': leaf_size_y,
                'leaf_size_z': leaf_size_z,
                'debug': voxel_debug,
            }]
        ),

        # Ground Segmentation Node
        Node(
            package='lidar_pointcloud_filters',
            executable='ground_segmentation',
            name='ground_segmentation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': ground_input_topic,
                'output_topic': ground_output_topic,
                'ground_threshold': ground_threshold,
                'radial_div_num': radial_div_num,
                'concentric_div_num': concentric_div_num,
                'max_range': max_range,
                'min_range': min_range,
                'debug': ground_debug,
            }]
        ),

        # Outliers Filter Node
        Node(
            package='lidar_pointcloud_filters',
            executable='outliers_filters',
            name='outliers_filters',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'mean_k': outliers_mean_k,
                'stddev_mul_thresh': outliers_stddev,
                'debug': outliers_debug,
            }]
        ),
    ])


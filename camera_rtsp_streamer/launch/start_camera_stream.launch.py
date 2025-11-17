
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Declare launch arguments
    hostname_arg = DeclareLaunchArgument(
        'hostname',
        default_value='192.168.1.1',
        description='RTSP camera hostname/IP'
    )
    
    username_arg = DeclareLaunchArgument(
        'username',
        default_value='admin',
        description='RTSP camera username'
    )
    
    password_arg = DeclareLaunchArgument(
        'password',
        default_value='admin', 
        description='RTSP camera password'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='554',
        description='RTSP camera port'
    )
    
    stream_arg = DeclareLaunchArgument(
        'stream',
        default_value='Streaming/Channels/101',
        description='RTSP stream path'
    )
    
    inverted_arg = DeclareLaunchArgument(
        'inverted',
        default_value='false',
        description='Rotate image 180 degrees if true'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='30',
        description='Publishing rate in Hz'
    )

    return LaunchDescription([
        hostname_arg,
        username_arg,
        password_arg,
        port_arg,
        stream_arg,
        inverted_arg,
        publish_rate_arg,
        Node(
            package='camera_rtsp_streamer',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'hostname': LaunchConfiguration('hostname'),
                'username': LaunchConfiguration('username'),
                'password': LaunchConfiguration('password'),
                'port': LaunchConfiguration('port'),
                'stream': LaunchConfiguration('stream'),
                'inverted': LaunchConfiguration('inverted'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }],
        ),
    ])

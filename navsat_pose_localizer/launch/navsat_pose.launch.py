from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # Launch configurations
    gps_fix_topic = LaunchConfiguration("gps_fix_topic")
    reference_origin_topic = LaunchConfiguration("reference_origin_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    yaw_min_distance = LaunchConfiguration("yaw_min_distance")

    navsat_pose_estimator_node = Node(
        package="navsat_pose_localizer",
        executable="navsat_pose_localizer_node",
        name="navsat_pose_estimator",
        output="screen",
        parameters=[
            {
                "gps_fix_topic": gps_fix_topic,
                "reference_origin_topic": reference_origin_topic,
                "odom_topic": odom_topic,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "yaw_min_distance": yaw_min_distance,
            }
        ],
    )

    return LaunchDescription(
        [
            # Exposed launch-time arguments
            DeclareLaunchArgument(
                "gps_fix_topic",
                default_value="gps/fix",
                description="Input GPS fix topic (sensor_msgs/msg/NavSatFix)",
            ),
            DeclareLaunchArgument(
                "reference_origin_topic",
                default_value="/reference_origin",
                description="Reference origin topic (geographic_msgs/msg/GeoPoint) - used as origin for odometry",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/navsat/odometry",
                description="Output odometry topic (nav_msgs/msg/Odometry)",
            ),
            DeclareLaunchArgument(
                "odom_frame",
                default_value="odom",
                description="Frame ID used in the odometry message header.frame_id",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description="Child frame ID used in odometry child_frame_id",
            ),
            DeclareLaunchArgument(
                "yaw_min_distance",
                default_value="0.2",
                description="Minimum travelled distance in metres before yaw is updated",
            ),
            navsat_pose_estimator_node,
        ]
    )


#!/usr/bin/env python3
import math
from typing import Optional

import numpy as np
import rclpy
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
import utm

class NavsatPoseLocalizer(Node):
    """Publishes odometry derived purely from GPS positions relative to a reference origin."""

    def __init__(self) -> None:
        super().__init__('navsat_pose_localizer_node')
        self.get_logger().info('navsat pose localizer node started')
        self.initial_utm: Optional[np.ndarray] = None
        self.last_utm: Optional[np.ndarray] = None
        self.current_utm: Optional[np.ndarray] = None
        self.last_yaw: float = 0.0

        # Parameters for topics and frames
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('gps_fix_topic', 'gps/fix')
        self.declare_parameter('reference_origin_topic', 'reference_origin')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('yaw_min_distance', 0.2)  

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.gps_fix_topic = self.get_parameter('gps_fix_topic').get_parameter_value().string_value
        self.reference_origin_topic = self.get_parameter('reference_origin_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.yaw_min_distance = self.get_parameter('yaw_min_distance').get_parameter_value().double_value

        self.odom_msg = self.init_odom_msg()

        self.qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, self.qos_profile)
        self.pos_sub = self.create_subscription(NavSatFix, self.gps_fix_topic, self.position_cb, self.qos_profile)
        self.reference_origin_sub = self.create_subscription(
            GeoPoint, self.reference_origin_topic, self.reference_origin_cb, self.qos_profile
        )

        self.displacement = np.zeros(3)
        self.reference_origin_received = False

    def reference_origin_cb(self, msg: GeoPoint) -> None:
        """Callback for reference origin. Sets the origin for odometry calculation."""
        if self.reference_origin_received:
            return
        
        self.initial_utm = np.array(utm.from_latlon(msg.latitude, msg.longitude)[:2])
        self.last_utm = self.initial_utm.copy()
        self.reference_origin_received = True
        self.get_logger().info(
            f'Reference origin set: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}. '
            'GPS odometry calculation started.'
        )

    def init_odom_msg(self) -> Odometry:
        msg = Odometry()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        return msg

    def position_cb(self, msg: NavSatFix) -> None:
        """Callback for GPS position updates. Calculates odometry relative to reference origin."""
        if msg.status.status < 0:
            return  # Invalid GPS

        if not self.reference_origin_received:
            self.get_logger().debug('GPS fix received but reference origin not set yet. Waiting...')
            return

        self.current_utm = np.array(utm.from_latlon(msg.latitude, msg.longitude)[:2])

        self.displacement[:2] = self.current_utm - self.initial_utm

        # Update yaw based on movement
        yaw = self.last_yaw
        if self.last_utm is not None:
            delta = self.current_utm - self.last_utm
            travelled = np.linalg.norm(delta)
            if travelled > self.yaw_min_distance:
                yaw = math.atan2(delta[1], delta[0])
                self.last_yaw = yaw
                self.last_utm = self.current_utm

        orientation = self.quaternion_from_euler(0.0, 0.0, yaw)

        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose.position.x = float(self.displacement[0])
        self.odom_msg.pose.pose.position.y = float(self.displacement[1])
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.pose.orientation = orientation
        self.odom_pub.publish(self.odom_msg)

    @staticmethod
    def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        return q


def main(args=None):
    rclpy.init(args=args)
    node = NavsatPoseLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

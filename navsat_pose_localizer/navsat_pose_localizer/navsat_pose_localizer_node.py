#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class NavsatPoseLocalizerNode(Node):

    def __init__(self):
        super().__init__("navsat_pose_localizer_node")
        self.get_logger().info("Hello world from the Python node navsat_pose_localizer_node")


def main(args=None):
    rclpy.init(args=args)

    navsat_pose_localizer_node = NavsatPoseLocalizerNode()

    try:
        rclpy.spin(navsat_pose_localizer_node)
    except KeyboardInterrupt:
        pass

    navsat_pose_localizer_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

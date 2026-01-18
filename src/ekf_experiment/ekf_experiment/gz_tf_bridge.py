#!/usr/bin/env python3
"""
Bridges Gazebo pose info to proper TF frames
"""
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import tf2_ros


class GzTfBridge(Node):
    def __init__(self):
        super().__init__('gz_tf_bridge')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.sub = self.create_subscription(
            TFMessage,
            '/world/project_world/pose/info',
            self.pose_callback,
            10
        )

        self.get_logger().info("Gazebo TF Bridge started")

    def pose_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            # Only publish vehicle_blue frame
            if transform.child_frame_id == 'vehicle_blue':
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'world'
                t.child_frame_id = 'vehicle_blue/chassis'
                t.transform = transform.transform

                # Offset for chassis (0.5, 0, 0.4) relative to vehicle base
                t.transform.translation.x += 0.5
                t.transform.translation.z += 0.4

                self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = GzTfBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
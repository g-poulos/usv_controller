#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


class WaveSubscriberNode(Node):
    def __init__(self):
        super().__init__("wave_subscriber")
        self.get_logger().info("Started wave subscriber node")
        self.wave_force_subscriber = self.create_subscription(
            Vector3, "/wave/force", self.force_callback, 10)
        self.wave_torque_subscriber = self.create_subscription(
            Vector3, "/wave/torque", self.torque_callback, 10)

    def force_callback(self, msg: Vector3):
        self.get_logger().info("Force: " + str(msg))

    def torque_callback(self, msg: Vector3):
        self.get_logger().info("Torque: " + str(msg))


def main(args=None):
    rclpy.init(args=args)
    node = WaveSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

MODEL_NAME = "boat"


class WaveSubscriberNode(Node):
    def __init__(self):
        super().__init__("wave_subscriber")
        self.get_logger().info("Started wave subscriber node")
        self.wave_force_subscriber = self.create_subscription(
            Vector3, "/wave/force", self.force_callback, 10)
        self.wave_torque_subscriber = self.create_subscription(
            Vector3, "/wave/torque", self.torque_callback, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, f"/model/{MODEL_NAME}/odometry", self.odom_callback, 10)

    def force_callback(self, msg: Vector3):
        self.get_logger().info("Force: " + str(msg))

    def torque_callback(self, msg: Vector3):
        self.get_logger().info("Torque: " + str(msg))

    def odom_callback(self, msg: Odometry):
        self.get_logger().info("Odometry: " + str(msg.twist.twist))


def main(args=None):
    rclpy.init(args=args)
    node = WaveSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

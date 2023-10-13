#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class BoatControllerNode(Node):
    def __init__(self):
        super().__init__("boat_controller")
        self.get_logger().info("Started boat controller node!")
        self.thrust_publisher = self.create_publisher(
            Float64, "/model/boat/joint/propeller_joint/cmd_thrust", 10)
        self.steer_publisher = self.create_publisher(
            Float64, "/model/boat/joint/engine_joint/cmd_steer", 10)
        self.timer = self.create_timer(0.5, self.send_thrust_command)

    def send_thrust_command(self):
        thrust_msg = Float64()
        thrust_msg.data = 2500.0
        self.thrust_publisher.publish(thrust_msg)

        steer_msg = Float64()
        steer_msg.data = 5.0
        self.steer_publisher.publish(steer_msg)

        self.get_logger().info("Thrust: " + str(thrust_msg) + "Steer: " + str(steer_msg))


def main(args=None):
    rclpy.init(args=args)
    node = BoatControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

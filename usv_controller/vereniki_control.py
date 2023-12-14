#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np

L_bc = 3.5          # Length of the triangular base
L_ac = 4.5          # Length of the triangular side

d_ad = np.sqrt(L_ac ** 2 - (L_bc / 2) ** 2)
d_bf = L_bc / 2
d_ae = d_ad * (2 / 3)


def get_thrust_msgs(input_vec):
    dv = (L_ac + L_bc) / 2
    J_star = np.array([[1, 0, -(d_bf - (L_bc / 2)) / dv],
                       [0, -1, -d_ae / dv],
                       [1, 0, -d_bf / dv],
                       [0, -1, (d_ad - d_ae) / dv],
                       [1, 0, (L_bc - d_bf) / dv],
                       [0, -1, (d_ad - d_ae) / dv]])

    J_star_plus = J_star @ np.linalg.inv(J_star.T @ J_star)
    q_star = input_vec
    return J_star_plus @ q_star.T


class VerenikiControllerNode(Node):
    def __init__(self):
        super().__init__("vereniki_controller")
        self.get_logger().info("Started Vereniki controller node!")

        # Thrusters
        self.thrustA_publisher = self.create_publisher(
            Float64, "/model/vereniki/joint/propeller_jointA/cmd_thrust", 10)
        self.thrustB_publisher = self.create_publisher(
            Float64, "/model/vereniki/joint/propeller_jointB/cmd_thrust", 10)
        self.thrustC_publisher = self.create_publisher(
            Float64, "/model/vereniki/joint/propeller_jointC/cmd_thrust", 10)

        # Steering
        self.steerA_publisher = self.create_publisher(
            Float64, "/model/vereniki/joint/engine_jointA/cmd_steer", 10)
        self.steerB_publisher = self.create_publisher(
            Float64, "/model/vereniki/joint/engine_jointB/cmd_steer", 10)
        self.steerC_publisher = self.create_publisher(
            Float64, "/model/vereniki/joint/engine_jointC/cmd_steer", 10)

        self.timer = self.create_timer(1, self.send_thrust_command)

    def send_thrust_command(self):
        msgs = get_thrust_msgs(np.array([0, 0, 0]))

        thrustA_msg = Float64()
        directionA_msg = Float64()
        thrustB_msg = Float64()
        directionB_msg = Float64()
        thrustC_msg = Float64()
        directionC_msg = Float64()

        thrustA_msg.data = msgs[0]
        directionA_msg.data = np.radians(msgs[1])
        thrustB_msg.data = msgs[2]
        directionB_msg.data = np.radians(msgs[3])
        thrustC_msg.data = msgs[4]
        directionC_msg.data = np.radians(msgs[5])

        self.thrustA_publisher.publish(thrustA_msg)
        self.thrustB_publisher.publish(thrustB_msg)
        self.thrustC_publisher.publish(thrustC_msg)

        self.steerA_publisher.publish(directionA_msg)
        self.steerB_publisher.publish(directionB_msg)
        self.steerC_publisher.publish(directionC_msg)

        self.get_logger().info("Thrust info: " + str(msgs))


def main(args=None):
    rclpy.init(args=args)
    node = VerenikiControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

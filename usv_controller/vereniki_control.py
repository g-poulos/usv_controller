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


def get_thrust_msgs2(input_vec):
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


def get_thrust_msgs(input_vec):
    dv = (L_ac + L_bc) / 2
    J_star = np.array([[1, 0, -(d_bf - (L_bc / 2)) / dv],
                       [0, -1, -d_ae / dv],
                       [1, 0, -d_bf / dv],
                       [0, -1, (d_ad - d_ae) / dv],
                       [1, 0, (L_bc - d_bf) / dv],
                       [0, -1, (d_ad - d_ae) / dv]])
    J_star = J_star.T

    J_star_plus = J_star.T @ np.linalg.inv(J_star @ J_star.T)
    # q_star = np.array([input_vec[0], input_vec[1], input_vec[2]/dv])
    q_star = input_vec
    return J_star_plus @ q_star


def cartesian_to_polar(x, y):
    r = np.sqrt(x**2 + y**2)
    theta = -np.arctan2(y, x)
    return r, theta


def thrust_to_rotations(thrust):
    rad_s = np.sqrt(abs(thrust) / (1025 * 0.01 * (0.48 ** 4)))
    if thrust < 0:
        rad_s = -rad_s
    return rad_s


class VerenikiControllerNode(Node):
    def __init__(self):
        super().__init__("vereniki_controller")
        self.get_logger().info("Started Vereniki controller node!")

        # Thrusters
        self.thrustA_publisher = self.create_publisher(
            Float64, "/model/vereniki/joint/propeller_jointA/cmd_vel", 10)
        self.thrustB_publisher = self.create_publisher(
            Float64, "/model/vereniki/joint/propeller_jointB/cmd_vel", 10)
        self.thrustC_publisher = self.create_publisher(
            Float64, "/model/vereniki/joint/propeller_jointC/cmd_vel", 10)

        # Steering
        self.steerA_publisher = self.create_publisher(
            Float64, "/model/vereniki/joint/engine_jointA/cmd_steer", 10)
        self.steerB_publisher = self.create_publisher(
            Float64, "/model/vereniki/joint/engine_jointB/cmd_steer", 10)
        self.steerC_publisher = self.create_publisher(
            Float64, "/model/vereniki/joint/engine_jointC/cmd_steer", 10)

        self.timer = self.create_timer(1, self.send_thrust_command)

    def send_thrust_command(self):
        msgs = get_thrust_msgs(np.array([100, 0, 10]))

        thrustA_msg = Float64()
        directionA_msg = Float64()
        thrustB_msg = Float64()
        directionB_msg = Float64()
        thrustC_msg = Float64()
        directionC_msg = Float64()


        thrustA, thetaA = cartesian_to_polar(msgs[0], msgs[1])
        thrustB, thetaB = cartesian_to_polar(msgs[2], msgs[3])
        thrustC, thetaC = cartesian_to_polar(msgs[4], msgs[5])

        thrustA_msg.data = thrust_to_rotations(thrustA)
        directionA_msg.data = thetaA
        thrustB_msg.data = thrust_to_rotations(thrustB)
        directionB_msg.data = thetaB
        thrustC_msg.data = thrust_to_rotations(thrustC)
        directionC_msg.data = thetaC

        self.thrustA_publisher.publish(thrustA_msg)
        self.thrustB_publisher.publish(thrustB_msg)
        self.thrustC_publisher.publish(thrustC_msg)

        self.steerA_publisher.publish(directionA_msg)
        self.steerB_publisher.publish(directionB_msg)
        self.steerC_publisher.publish(directionC_msg)

        # self.get_logger().info(str(msgs))
        # self.get_logger().info("Thrust info (thrust, deg): \n"
        #                        f"A: {msgs[0]} {msgs[1]}\n"
        #                        f"B: {msgs[2]} {msgs[3]}\n"
        #                        f"C: {msgs[4]} {msgs[5]}")
        self.get_logger().info("Thrust info (rad/s, radians): \n"
                               f"{thrustA} {thetaA}\n"
                               f"{thrustB} {thetaB}\n"
                               f"{thrustC} {thetaC}")


def main():
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node = VerenikiControllerNode()
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()

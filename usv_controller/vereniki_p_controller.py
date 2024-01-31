#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np
from nav_msgs.msg import Odometry

L_bc = 3.5          # Length of the triangular base
L_ac = 4.5          # Length of the triangular side

d_ad = np.sqrt(L_ac ** 2 - (L_bc / 2) ** 2)
d_bf = L_bc / 2
d_ae = d_ad * (2 / 3)


def get_thrust_7a(input_vec):
    J = np.array([[1, 0, -(d_bf - (L_bc / 2))],
                  [0, -1, -d_ae],
                  [1, 0, -d_bf],
                  [0, -1, (d_ad - d_ae)],
                  [1, 0, (L_bc - d_bf)],
                  [0, -1, (d_ad - d_ae)]])
    J = J.T

    J_plus = J.T @ np.linalg.inv(J @ J.T)
    return J_plus @ input_vec


def cartesian_to_polar(x, y):
    magnitude = np.sqrt(x**2 + y**2)
    theta = np.arctan2(x, y) - np.pi/2
    return magnitude, theta


def thrust_to_rotations(thrust):
    rad_s = np.sqrt(abs(thrust) / (1025 * 0.01 * (0.48 ** 4)))
    if thrust < 0:
        rad_s = -rad_s
    return rad_s


def translate_point(P_A, frameB, theta):
    tx, ty = frameB
    R = np.array([[np.cos(theta), -np.sin(theta), tx],
                  [np.sin(theta), np.cos(theta), ty],
                  [0, 0, 1]])  #

    P_B = np.dot(np.linalg.inv(R), np.array([P_A[0], P_A[1], 1]))
    return P_B


def get_input_thrust(value, Kp):
    value = value * Kp
    if abs(value) > 150:
        return 150 * np.sign(value)
    return value


def quaternion_to_yaw(quaternion):
    q0, q1, q2, q3 = quaternion
    yaw = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2**2 + q3**2))
    return np.degrees(yaw)


class VerenikiControllerNode(Node):
    def __init__(self):
        super().__init__("vereniki_p_controller")
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

        self.odom_subscription = self.create_subscription(
            Odometry,
            "/model/vereniki/odometry",
            self.pose_callback,
            10)

    def pose_callback(self, msg):
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw([msg.pose.pose.orientation.w,
                                 msg.pose.pose.orientation.x,
                                 msg.pose.pose.orientation.y,
                                 msg.pose.pose.orientation.z])

        x_A = 20
        y_A = 10
        theta_des = 45

        P_B = translate_point(np.array([x_A, y_A]),
                              np.array([position_x, position_y]),
                              np.radians(yaw))
        direction_angle = np.arctan2(P_B[1], P_B[0])

        self.send_thrust_command(np.array([get_input_thrust(P_B[0], Kp=10),
                                           get_input_thrust(P_B[1], Kp=10),
                                           get_input_thrust(theta_des-yaw, Kp=10)]))

        print(f"Translated point: {P_B}")
        print(f"Direction: {np.degrees(direction_angle + np.pi / 2)}")
        print(f"Distance: {np.linalg.norm(P_B)}")
        print(f"Input: {get_input_thrust(P_B[0], Kp=10), get_input_thrust(P_B[1], Kp=10)}")

    def send_thrust_command(self, input_vector):
        msgs = get_thrust_7a(input_vector)

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

        self.get_logger().info("Thrust info (rad/s, radians): \n"
                               f"Engine A: {thrustA} {thetaA}\n"
                               f"Engine B: {thrustB} {thetaB}\n"
                               f"Engine C: {thrustC} {thetaC}")


def main():
    # ros2 run usv_controller vereniki_controller --ros-args -p thrust:="100, 0, 0"
    # ros2 run usv_controller vereniki_controller --ros-args -p cmd_type := direction_only
    # ros2 run usv_controller vereniki_controller --ros-args -p cmd_type := direction_only
    #                                                        -p thrust:="100, 0, 0"

    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node = VerenikiControllerNode()
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()

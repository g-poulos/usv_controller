#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from .vereniki_utilities import *


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


class VerenikiControllerNode(Node):
    def __init__(self):
        super().__init__("vereniki_p_controller")
        self.get_logger().info("Started Vereniki P-controller node!")

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
            self.thrust_callback,
            10)

    def thrust_callback(self, msg):
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

        input_vector = np.array([get_input_thrust(P_B[0], Kp=10),
                                 get_input_thrust(P_B[1], Kp=10),
                                 get_input_thrust(theta_des-yaw, Kp=10)])

        direction_msgs, thrust_msgs = get_thrust_msgs(input_vector)

        self.thrustA_publisher.publish(thrust_msgs[0])
        self.thrustB_publisher.publish(thrust_msgs[1])
        self.thrustC_publisher.publish(thrust_msgs[2])
        self.steerA_publisher.publish(direction_msgs[0])
        self.steerB_publisher.publish(direction_msgs[1])
        self.steerC_publisher.publish(direction_msgs[2])

        print(f"Translated point: {P_B}")
        print(f"Direction: {np.degrees(direction_angle + np.pi / 2)}")
        print(f"Distance: {np.linalg.norm(P_B)}")
        print(f"Input: {get_input_thrust(P_B[0], Kp=10), get_input_thrust(P_B[1], Kp=10)}")

        self.get_logger().info("Thrust info (rad/s, radians): \n"
                               f"Engine A: {thrust_msgs[0].data} {direction_msgs[0].data}\n"
                               f"Engine B: {thrust_msgs[1].data} {direction_msgs[1].data}\n"
                               f"Engine C: {thrust_msgs[2].data} {direction_msgs[2].data}")


def main():
    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node = VerenikiControllerNode()
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()

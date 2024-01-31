#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from .vereniki_utilities import *


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
        self.declare_parameter('cmd_type', 'default')
        self.declare_parameter('thrust', '100, 0, 0')

    def send_thrust_command(self):
        cmd_type = self.get_parameter('cmd_type').get_parameter_value().string_value
        input_thrust = self.get_parameter('thrust').get_parameter_value().string_value
        input_thrust_lst = input_thrust.split(",")
        input_vector = np.array(list(map(float, input_thrust_lst)))

        self.get_logger().info(f"Thrust: {input_vector}")
        self.get_logger().info(f"Command type: {cmd_type}")

        direction_msgs, thrust_msgs = get_thrust_msgs(input_vector)

        if cmd_type == "default" or cmd_type == "thrust_only":
            self.thrustA_publisher.publish(thrust_msgs[0])
            self.thrustB_publisher.publish(thrust_msgs[1])
            self.thrustC_publisher.publish(thrust_msgs[2])

        if cmd_type == "default" or cmd_type == "direction_only":
            self.steerA_publisher.publish(direction_msgs[0])
            self.steerB_publisher.publish(direction_msgs[1])
            self.steerC_publisher.publish(direction_msgs[2])

        self.get_logger().info("Thrust info (rad/s, radians): \n"
                               f"Engine A: {thrust_msgs[0].data} {direction_msgs[0].data}\n"
                               f"Engine B: {thrust_msgs[1].data} {direction_msgs[1].data}\n"
                               f"Engine C: {thrust_msgs[2].data} {direction_msgs[2].data}")


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

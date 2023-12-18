import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
import rosbag2_py
import os


RECORD_NUM = 0
sbr = None
first_msg = True
start_time = 0


class SimpleBagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder')
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri=f'bagfiles/record{RECORD_NUM}',
            storage_id='mcap')
        converter_options = rosbag2_py._storage.ConverterOptions('cdr', 'cdr')
        self.writer.open(storage_options, converter_options)

        wave_force_topic_info = rosbag2_py._storage.TopicMetadata(
            name='/wave/force',
            type='geometry_msgs/msg/Vector3',
            serialization_format='cdr')
        self.writer.create_topic(wave_force_topic_info)

        wave_torque_topic_info = rosbag2_py._storage.TopicMetadata(
            name='/wave/torque',
            type='geometry_msgs/msg/Vector3',
            serialization_format='cdr')
        self.writer.create_topic(wave_torque_topic_info)

        odom_topic_info = rosbag2_py._storage.TopicMetadata(
            name='/model/vereniki/odometry',
            type='nav_msgs/msg/Odometry',
            serialization_format='cdr')
        self.writer.create_topic(odom_topic_info)

        imu_topic_info = rosbag2_py._storage.TopicMetadata(
            name='/model/vereniki/imu',
            type='sensor_msgs/msg/Imu',
            serialization_format='cdr')
        self.writer.create_topic(imu_topic_info)

        self.wave_force_subscription = self.create_subscription(
            Vector3,
            "/wave/force",
            self.wave_force_callback,
            10)

        self.wave_torque_subscription = self.create_subscription(
            Vector3,
            "/wave/torque",
            self.wave_torque_callback,
            10)

        self.odom_subscription = self.create_subscription(
            Odometry,
            "/model/vereniki/odometry",
            self.pose_callback,
            10)

        self.clock_subscription = self.create_subscription(
            Clock,
            "/clock",
            self.clock_callback,
            10)

        self.imu_subscription = self.create_subscription(
            Imu,
            "/model/vereniki/imu",
            self.imu_callback,
            10)

    def wave_force_callback(self, msg):
        self.writer.write(
            '/wave/force',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def wave_torque_callback(self, msg):
        self.writer.write(
            '/wave/torque',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def pose_callback(self, msg):
        self.writer.write(
            "/model/vereniki/odometry",
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def clock_callback(self, msg):
        global start_time, first_msg
        sim_time = msg.clock.sec
        rec_time = sim_time - start_time

        if first_msg:
            start_time = sim_time
            first_msg = False

        if rec_time > 60 and sbr:
            sbr.destroy_node()
            rclpy.shutdown()

        self.get_logger().info(f"Sim time: {sim_time}, Rec time: {rec_time}",
                               throttle_duration_sec=1)

    def imu_callback(self, msg):
        self.writer.write(
            "/model/vereniki/imu",
            serialize_message(msg),
            self.get_clock().now().nanoseconds)


def count_records(path):
    print(path)
    files = folders = 0
    for _, dirnames, filenames in os.walk(path):
        files += len(filenames)
        folders += len(dirnames)
    return folders


def main(args=None):
    global RECORD_NUM, sbr

    RECORD_NUM = count_records(
        os.path.dirname(os.path.realpath(__file__)) + "/../bagfiles")
    print(f"Writing record: record{RECORD_NUM}")

    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()
    sbr = SimpleBagRecorder()
    executor.add_node(sbr)
    executor.spin()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from geometry_msgs.msg import Vector3, Pose
from nav_msgs.msg import Odometry
import rosbag2_py


class SimpleBagRecorder(Node):
    def __init__(self):
        super().__init__('simple_bag_recorder')
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri='src/usv_controller/bagfiles/record5',
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
            name='/model/boat/pose',
            type='geometry_msgs/msg/Pose',
            serialization_format='cdr')
        self.writer.create_topic(odom_topic_info)

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
            Pose,
            "/model/boat/pose",
            self.pose_callback,
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
            "/model/boat/pose",
            serialize_message(msg),
            self.get_clock().now().nanoseconds)


def main(args=None):
    rclpy.init(args=args)
    sbr = SimpleBagRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
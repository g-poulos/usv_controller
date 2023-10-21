import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message

from geometry_msgs.msg import Vector3
import rosbag2_py


class SimpleBagRecorder(Node):
    def __init__(self):
        super().__init__('simple_bag_recorder')
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(uri='bagfile', storage_id='mcap')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
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


        self.subscription = self.create_subscription(
            Vector3,
            '/wave/force',
            self.topic_callback,
            10)
        self.subscription

    def topic_callback(self, msg):
        self.writer.write(
            '/wave/force',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)


def main(args=None):
    rclpy.init(args=args)
    sbr = SimpleBagRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
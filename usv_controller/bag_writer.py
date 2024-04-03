import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
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

        # ------------------------| TOPIC INFO |------------------------

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

        linear_acc_topic_info = rosbag2_py._storage.TopicMetadata(
            name='/model/vereniki/acceleration/linear',
            type='geometry_msgs/msg/Vector3',
            serialization_format='cdr')
        self.writer.create_topic(linear_acc_topic_info)

        angular_acc_topic_info = rosbag2_py._storage.TopicMetadata(
            name='/model/vereniki/acceleration/angular',
            type='geometry_msgs/msg/Vector3',
            serialization_format='cdr')
        self.writer.create_topic(angular_acc_topic_info)

        current_speed_topic_info = rosbag2_py._storage.TopicMetadata(
            name='/waterCurrent/speed',
            type='std_msgs/msg/Float32',
            serialization_format='cdr')
        self.writer.create_topic(current_speed_topic_info)

        current_direction_topic_info = rosbag2_py._storage.TopicMetadata(
            name='/waterCurrent/direction',
            type='std_msgs/msg/Float32',
            serialization_format='cdr')
        self.writer.create_topic(current_direction_topic_info)

        current_force_topic_info = rosbag2_py._storage.TopicMetadata(
            name='/waterCurrent/force',
            type='geometry_msgs/msg/Vector3',
            serialization_format='cdr')
        self.writer.create_topic(current_force_topic_info)

        current_torque_topic_info = rosbag2_py._storage.TopicMetadata(
            name='/waterCurrent/torque',
            type='geometry_msgs/msg/Vector3',
            serialization_format='cdr')
        self.writer.create_topic(current_torque_topic_info)

        wind_speed_topic_info = rosbag2_py._storage.TopicMetadata(
            name='/wind/speed',
            type='std_msgs/msg/Float32',
            serialization_format='cdr')
        self.writer.create_topic(wind_speed_topic_info)

        wind_direction_topic_info = rosbag2_py._storage.TopicMetadata(
            name='/wind/direction',
            type='std_msgs/msg/Float32',
            serialization_format='cdr')
        self.writer.create_topic(wind_direction_topic_info)

        wind_force_topic_info = rosbag2_py._storage.TopicMetadata(
            name='/wind/force',
            type='geometry_msgs/msg/Vector3',
            serialization_format='cdr')
        self.writer.create_topic(wind_force_topic_info)

        wind_torque_topic_info = rosbag2_py._storage.TopicMetadata(
            name='/wind/torque',
            type='geometry_msgs/msg/Vector3',
            serialization_format='cdr')
        self.writer.create_topic(wind_torque_topic_info)

        thrust_vec_topic_info = rosbag2_py._storage.TopicMetadata(
            name='/model/vereniki/controller/thrust_vec',
            type='geometry_msgs/msg/Vector3',
            serialization_format='cdr')
        self.writer.create_topic(thrust_vec_topic_info)

        # ------------------------| SUBSCRIPTIONS |------------------------

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

        self.linear_acc_subscription = self.create_subscription(
            Vector3,
            "/model/vereniki/acceleration/linear",
            self.linear_acc_callback,
            10)

        self.angular_acc_subscription = self.create_subscription(
            Vector3,
            "/model/vereniki/acceleration/angular",
            self.angular_acc_callback,
            10)

        self.current_speed_subscription = self.create_subscription(
            Float32,
            "/waterCurrent/speed",
            self.current_speed_callback,
            10)

        self.current_direction_subscription = self.create_subscription(
            Float32,
            "/waterCurrent/direction",
            self.current_direction_callback,
            10)

        self.current_force_subscription = self.create_subscription(
            Vector3,
            "/waterCurrent/force",
            self.current_force_callback,
            10)

        self.current_torque_subscription = self.create_subscription(
            Vector3,
            "/waterCurrent/torque",
            self.current_torque_callback,
            10)

        self.wind_speed_subscription = self.create_subscription(
            Float32,
            "/wind/speed",
            self.wind_speed_callback,
            10)

        self.wind_direction_subscription = self.create_subscription(
            Float32,
            "/wind/direction",
            self.wind_direction_callback,
            10)

        self.wind_force_subscription = self.create_subscription(
            Vector3,
            "/wind/force",
            self.wind_force_callback,
            10)

        self.wind_torque_subscription = self.create_subscription(
            Vector3,
            "/wind/torque",
            self.wind_torque_callback,
            10)

        self.thrust_vec_subscription = self.create_subscription(
            Vector3,
            "/model/vereniki/controller/thrust_vec",
            self.thrust_vec_callback,
            10)

        # ------------------------| PARAMETERS |------------------------

        self.declare_parameter('duration', 60.0)

    # ------------------------| CALLBACK FUNCTIONS |------------------------

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

    def linear_acc_callback(self, msg):
        self.writer.write(
            "/model/vereniki/acceleration/linear",
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def angular_acc_callback(self, msg):
        self.writer.write(
            "/model/vereniki/acceleration/angular",
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def current_speed_callback(self, msg):
        self.writer.write(
            "/waterCurrent/speed",
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def current_direction_callback(self, msg):
        self.writer.write(
            "/waterCurrent/direction",
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def current_force_callback(self, msg):
        self.writer.write(
            "/waterCurrent/force",
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def current_torque_callback(self, msg):
        self.writer.write(
            "/waterCurrent/torque",
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def wind_speed_callback(self, msg):
        self.writer.write(
            "/wind/speed",
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def wind_direction_callback(self, msg):
        self.writer.write(
            "/wind/direction",
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def wind_force_callback(self, msg):
        self.writer.write(
            "/wind/force",
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def wind_torque_callback(self, msg):
        self.writer.write(
            "/wind/torque",
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def thrust_vec_callback(self, msg):
        self.writer.write(
            "/model/vereniki/controller/thrust_vec",
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def clock_callback(self, msg):
        global start_time, first_msg
        sim_time = msg.clock.sec
        rec_time = sim_time - start_time
        duration = self.get_parameter('duration').get_parameter_value().double_value

        if first_msg:
            start_time = sim_time
            rec_time = 0
            first_msg = False

        if rec_time > duration and sbr:
            sbr.destroy_node()
            rclpy.shutdown()

        self.get_logger().info(f"Sim time: {sim_time}, Rec time: {rec_time}/{duration}",
                               throttle_duration_sec=1)


def count_records(path):
    print(path)
    files = folders = 0
    for _, dirnames, filenames in os.walk(path):
        files += len(filenames)
        folders += len(dirnames)
    return folders


def main(args=None):
    global RECORD_NUM, sbr

    # ros2 run usv_controller bag_writer - -ros - args - p duration := 120.0

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

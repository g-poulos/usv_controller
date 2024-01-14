import pandas as pd
import numpy as np
from mcap_ros2.reader import read_ros2_messages
import matplotlib.pyplot as plt


def get_mcap_messages(file):
    pos = np.empty((3, 0))
    vel = np.empty((3, 0))
    acc = np.empty((3, 0))

    for msg in read_ros2_messages(file):
        if msg.channel.topic == "/model/vereniki/odometry":
            # print(f"{msg.ros_msg.pose.pose.position.x}: \n")
            pos = np.column_stack((pos, np.array([msg.ros_msg.pose.pose.position.x,
                                                  msg.ros_msg.pose.pose.position.y,
                                                  msg.ros_msg.pose.pose.orientation.z])))
            vel = np.column_stack((vel, np.array([msg.ros_msg.twist.twist.linear.x,
                                                  msg.ros_msg.twist.twist.linear.y,
                                                  msg.ros_msg.twist.twist.angular.z])))

        temp_acc = np.zeros(3)
        if msg.channel.topic == "/model/vereniki/acceleration/linear":
            # print(f"{msg.ros_msg.linear_acceleration.x}")
            # acc = np.column_stack((acc, np.array([msg.ros_msg.x,
            #                                       msg.ros_msg.y,
            #                                       0])))
            temp_acc[0] = msg.ros_msg.x
            temp_acc[1] = msg.ros_msg.y

        if msg.channel.topic == "/model/vereniki/acceleration/angular":
            temp_acc[2] = msg.ros_msg.z

        acc = np.column_stack((acc, temp_acc))


    odom_data = {'Position-x': pos[0, :],
                 'Position-y': pos[1, :],
                 'Orientation-z': pos[2, :],
                 'Velocity-x': vel[0, :],
                 'Velocity-y': vel[1, :],
                 'Velocity-z': vel[2, :]}

    imu_data = {'Acceleration-x': acc[0, :],
                'Acceleration-y': acc[1, :],
                'Acceleration-z': acc[2, :]}
    return pd.DataFrame(odom_data), pd.DataFrame(imu_data)


def align_values(data, size):
    selected_indices = np.linspace(0, len(data) - 1, size, dtype=int)
    return data.iloc[selected_indices].reset_index(drop=True)


def radians_to_degrees(radians):
    degrees = np.degrees(radians)
    degrees = (degrees + 180) % 360 - 180
    return degrees


def plot3_1(diff_values, sim_values, title):
    fig, ax = plt.subplots(3, 1, sharex=True, sharey=False)

    ax[0].plot(diff_values.iloc[:, 0], label="Dynamic Model", color="blue")
    ax[1].plot(diff_values.iloc[:, 1], label="Dynamic Model", color="blue")
    ax[2].plot(diff_values.iloc[:, 2], label="Dynamic Model", color="blue")
    ax[0].plot(sim_values.iloc[:, 0], label="Gazebo", color="orange")
    ax[1].plot(sim_values.iloc[:, 1], label="Gazebo", color="orange")
    ax[2].plot(sim_values.iloc[:, 2], label="Gazebo", color="orange")

    fig.set_figwidth(16)
    fig.set_figheight(9)
    fig.suptitle(title, fontsize=20)
    for i in range(3):
        ax[i].grid(True)
        ax[i].legend(loc="best")

    return ax, fig


if __name__ == '__main__':
    # Read simulation data
    odom_data, imu_data = get_mcap_messages("../bagfiles/record7/record7_0.mcap")
    odom_data_size = odom_data.shape[0]
    imu_data_size = imu_data.shape[0]

    # Read dynamic model data
    diff_data = pd.read_csv('output.csv')

    # Align value to
    x_pos_diff = align_values(diff_data['Position-x'], odom_data_size)
    y_pos_diff = align_values(diff_data['Position-y'], odom_data_size)
    z_orient_diff = align_values(diff_data['Orientation-z'], odom_data_size)

    x_vel_diff = align_values(diff_data['Velocity-x'], odom_data_size)
    y_vel_diff = align_values(diff_data['Velocity-y'], odom_data_size)
    z_vel_diff = align_values(diff_data['Velocity-z'], odom_data_size)

    x_acc_diff = align_values(diff_data['Acceleration-x'], imu_data_size)
    y_acc_diff = align_values(diff_data['Acceleration-y'], imu_data_size)
    z_acc_diff = align_values(diff_data['Acceleration-z'], imu_data_size)

    pose_diff = pd.DataFrame({'Position-x': x_pos_diff,
                              'Position-y': y_pos_diff,
                              'Orientation-z': z_orient_diff})

    twist_diff = pd.DataFrame({'Velocity-x': x_vel_diff,
                               'Velocity-y': y_vel_diff,
                               'Velocity-z': z_vel_diff})

    acc_diff = pd.DataFrame({'Acceleration-x': x_acc_diff,
                             'Acceleration-y': y_acc_diff,
                             'Acceleration-z': z_acc_diff})

    odom_data = pd.DataFrame(odom_data)
    odom_data['Orientation-z'] = odom_data['Orientation-z'].apply(radians_to_degrees)
    imu_data = pd.DataFrame(imu_data)

    plot3_1(pose_diff, odom_data, title="Position and Orientation")
    plot3_1(twist_diff, odom_data.iloc[:, 3:], title="Linear and Angular Velocity")
    plot3_1(acc_diff, imu_data, title="Linear and Angular Acceleration")

    plt.show()

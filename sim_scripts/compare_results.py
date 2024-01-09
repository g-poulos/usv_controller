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
        if msg.channel.topic == "/model/vereniki/imu":
            # print(f"{msg.ros_msg.linear_acceleration.x}")
            acc = np.column_stack((acc, np.array([msg.ros_msg.linear_acceleration.x,
                                                  msg.ros_msg.linear_acceleration.y,
                                                  0])))

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


if __name__ == '__main__':
    # Read simulation data
    odom_data, imu_data = get_mcap_messages("../bagfiles/record8/record8_0.mcap")
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

    fig, ax = plt.subplots(3, 3, sharex=False, sharey=False)
    fig.set_figwidth(32)
    fig.set_figheight(18)

    # Pose
    ax[0, 0].plot(odom_data['Position-x'], label='SIM')
    ax[0, 0].plot(x_pos_diff, label='Diff')

    ax[0, 1].plot(odom_data['Position-y'], label='SIM')
    ax[0, 1].plot(y_pos_diff, label='Diff')

    ax[0, 2].plot(pd.DataFrame(odom_data['Orientation-z']).apply(radians_to_degrees),
                  label='SIM')
    ax[0, 2].plot(z_orient_diff, label='Diff')

    # Twist
    ax[1, 0].plot(odom_data['Velocity-x'], label='SIM')
    ax[1, 0].plot(x_vel_diff, label='Diff')

    ax[1, 1].plot(odom_data['Velocity-y'], label='SIM')
    ax[1, 1].plot(y_vel_diff, label='Diff')

    ax[1, 2].plot(pd.DataFrame(odom_data['Velocity-z']), label='SIM')
    ax[1, 2].plot(z_vel_diff, label='Diff')

    # Acceleration
    ax[2, 0].plot(imu_data['Acceleration-x'], label='SIM')
    ax[2, 0].plot(x_acc_diff, label='Diff')

    ax[2, 1].plot(imu_data['Acceleration-y'], label='SIM')
    ax[2, 1].plot(y_acc_diff, label='Diff')

    # ax[2, 2].plot(pd.DataFrame(odom_data['Velocity-z']), label='SIM')
    # ax[2, 2].plot(z_vel_diff, label='Diff')

    for i in range(3):
        for j in range(3):
            ax[i, j].grid(True)
            ax[i, j].legend()

    plt.show()

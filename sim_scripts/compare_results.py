import pandas as pd
import numpy as np
from mcap_ros2.reader import read_ros2_messages
import matplotlib.pyplot as plt


def get_mcap_messages(file):
    pos = np.empty((3, 0))
    vel = np.empty((3, 0))
    angular_acc_z = []
    linear_acc_xy = []

    for msg in read_ros2_messages(file):
        if msg.channel.topic == "/model/vereniki/odometry":
            # print(f"{msg.ros_msg.pose.pose.position.x}: \n")
            yaw = quaternion_to_yaw([msg.ros_msg.pose.pose.orientation.w,
                                     msg.ros_msg.pose.pose.orientation.x,
                                     msg.ros_msg.pose.pose.orientation.y,
                                     msg.ros_msg.pose.pose.orientation.z])

            pos = np.column_stack((pos, np.array([msg.ros_msg.pose.pose.position.x,
                                                  msg.ros_msg.pose.pose.position.y,
                                                  yaw])))
            vel = np.column_stack((vel, np.array([msg.ros_msg.twist.twist.linear.x,
                                                  msg.ros_msg.twist.twist.linear.y,
                                                  msg.ros_msg.twist.twist.angular.z])))

        if msg.channel.topic == "/model/vereniki/acceleration/linear":
            linear_acc_xy.append([msg.ros_msg.x, msg.ros_msg.y])

        if msg.channel.topic == "/model/vereniki/acceleration/angular":
            angular_acc_z.append(msg.ros_msg.z)

    odom_data = {'Position-x': pos[0, :],
                 'Position-y': pos[1, :],
                 'Orientation-z': pos[2, :],
                 'Velocity-x': vel[0, :],
                 'Velocity-y': vel[1, :],
                 'Velocity-z': vel[2, :]}

    acc_data = pd.DataFrame(linear_acc_xy)
    acc_data['Acceleration-z'] = angular_acc_z
    acc_data.rename(columns={0: "Acceleration-x", 1: "Acceleration-y"}, inplace=True)

    return pd.DataFrame(odom_data), acc_data


def align_values(data, size):
    selected_indices = np.linspace(0, len(data) - 1, size, dtype=int)
    return data.iloc[selected_indices].reset_index(drop=True)


def quaternion_to_yaw(quaternion):
    # Extract the relevant quaternion components
    q0, q1, q2, q3 = quaternion

    # Calculate yaw (rotation around the vertical axis)
    yaw = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2**2 + q3**2))

    # Convert yaw from radians to degrees
    return np.degrees(yaw)


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

    return fig, ax


if __name__ == '__main__':
    # Read simulation data
    odom_data, acc_data = get_mcap_messages("../bagfiles/record9/record9_0.mcap")
    odom_data_size = odom_data.shape[0]
    acc_data_size = acc_data.shape[0]

    # Read dynamic model data
    diff_data = pd.read_csv('output.csv')

    # Align Gazebo simulation and Dynamic Model values
    x_pos_diff = align_values(diff_data['Position-x'], odom_data_size)
    y_pos_diff = align_values(diff_data['Position-y'], odom_data_size)
    z_orient_diff = align_values(diff_data['Orientation-z'], odom_data_size)

    x_vel_diff = align_values(diff_data['Velocity-x'], odom_data_size)
    y_vel_diff = align_values(diff_data['Velocity-y'], odom_data_size)
    z_vel_diff = align_values(diff_data['Velocity-z'], odom_data_size)

    x_acc_diff = align_values(diff_data['Acceleration-x'], acc_data_size)
    y_acc_diff = align_values(diff_data['Acceleration-y'], acc_data_size)
    z_acc_diff = align_values(diff_data['Acceleration-z'], acc_data_size)

    # Create separate dataframes
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
    acc_data = pd.DataFrame(acc_data)

    # Plot results
    plt.rcParams["font.family"] = "Times New Roman"
    plt.rcParams.update({'font.size': 15})
    pose_fig, pose_ax = plot3_1(pose_diff, odom_data, title="Position and Orientation")
    twist_fig, twist_ax = plot3_1(twist_diff, odom_data.iloc[:, 3:], title="Linear and Angular Velocity")
    acc_fig, acc_ax = plot3_1(acc_diff, acc_data, title="Linear and Angular Acceleration")

    pose_ax[0].set_ylabel('X-Axis Position [m]')
    pose_ax[1].set_ylabel('Y-Axis Position [m]')
    pose_ax[2].set_ylabel('Z-Axis Orientation [deg]')
    pose_ax[2].set_xlabel('Time [s]')

    twist_ax[0].set_ylabel('X-Axis \nLinear Velocity [m/s]')
    twist_ax[1].set_ylabel('Y-Axis \nLinear Velocity [m/s]')
    twist_ax[2].set_ylabel('Z-Axis \nAngular Velocity [rad/s]')
    twist_ax[2].set_xlabel('Time [s]')

    acc_ax[0].set_ylabel('X-Axis Linear\nAcceleration [m/s^2]')
    acc_ax[1].set_ylabel('Y-Axis Linear\nAcceleration [m/s^2]')
    acc_ax[2].set_ylabel('Z-Axis Angular\nAcceleration [rad/s^2]')
    acc_ax[2].set_xlabel('Time [s]')
    plt.show()

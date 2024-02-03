import pandas as pd
import numpy as np
from mcap_ros2.reader import read_ros2_messages
import matplotlib.pyplot as plt
from diff_calc import run_simulation
import sys
from matplotlib.offsetbox import OffsetImage, AnnotationBbox


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
    q0, q1, q2, q3 = quaternion
    yaw = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2**2 + q3**2))
    return np.degrees(yaw)


def set_min_plot_range(ax, min_value):
    if abs(ax.get_ylim()[0]) < min_value and abs(ax.get_ylim()[1]) < min_value:
        ax.set_ylim(ymin=-min_value, ymax=min_value)


def plot3_1(diff_values, sim_values, title):
    fig = plt.figure()
    ax0 = plt.subplot(311)
    ax1 = plt.subplot(312, sharex=ax0, sharey=ax0)
    ax2 = plt.subplot(313, sharex=ax0)
    time = np.linspace(0, 60, diff_values.shape[0])

    ax0.plot(time, diff_values.iloc[:, 0], label="Python Simulation", color="royalblue")
    ax1.plot(time, diff_values.iloc[:, 1], label="Python Simulation", color="royalblue")
    ax2.plot(time, diff_values.iloc[:, 2], label="Python Simulation", color="royalblue")
    ax0.plot(time, sim_values.iloc[:, 0], label="Gazebo Simulation", color="darkorange")
    ax1.plot(time, sim_values.iloc[:, 1], label="Gazebo Simulation", color="darkorange")
    ax2.plot(time, sim_values.iloc[:, 2], label="Gazebo Simulation", color="darkorange")

    if "Position" in title:
        set_min_plot_range(ax0, 5)
    if "Velocity" in title:
        set_min_plot_range(ax0, 0.2)
    if "Acceleration" in title:
        set_min_plot_range(ax0, 0.02)

    fig.set_figwidth(16)
    fig.set_figheight(9)
    fig.suptitle(title, fontsize=20)

    ax0.grid(True)
    ax0.legend(loc="best")
    ax1.grid(True)
    ax1.legend(loc="best")
    ax2.grid(True)
    ax2.legend(loc="best")
    # ax2.legend(ncol=2, bbox_to_anchor=(1, -0.2))
    return fig, [ax0, ax1, ax2]


def plot_trajectories(sim_data, diff_data):
    plt.rcParams["font.family"] = "Times New Roman"
    plt.rcParams.update({'font.size': 15})
    fig, ax = plt.subplots(figsize=(10, 10))

    plt.plot(sim_data.iloc[:, 1], sim_data.iloc[:, 0],
             label="Gazebo Simulation",
             color="darkorange")
    plt.plot(diff_data.iloc[:, 1], diff_data.iloc[:, 0],
             label="Python Simulation",
             color="royalblue")
    max_val = max(max(diff_data.iloc[:, 1]), max(diff_data.iloc[:, 0]))

    padding = 10
    plt.xlim([max_val + padding, -max_val - padding])
    plt.ylim([-max_val - padding, max_val + padding])

    plt.grid()
    plt.legend(loc="upper left")

    ax.set_xlabel('Y Axis Position [m]')
    ax.set_ylabel('X Axis Position [m]')
    fig.suptitle("Platform Trajectory", fontsize=20)

    im = plt.imread("/home/g-poulos/Downloads/vereniki.png")
    scale = (abs(ax.get_xlim()[0]) + abs(ax.get_xlim()[1])) / 5
    imagebox = OffsetImage(im, zoom=1 / scale)
    imagebox.image.axes = ax
    ab = AnnotationBbox(imagebox, (0.5, 0.5), xycoords='axes fraction',
                        bboxprops={'lw': 0, 'alpha': 0})
    ax.add_artist(ab)

    ax.annotate("Start position",
                xy=(-1, -1), xycoords='data',
                xytext=(-6, -4), textcoords='data',
                size=15, va="center", ha="center",
                arrowprops=dict(arrowstyle="-|>"))

    ax.annotate("Current Direction", xy=(-5, 5), xytext=(-5, 15),
                arrowprops=dict(facecolor='black', shrink=0.1),
                )

    ax.annotate("Wind Direction", xy=(6, 2), xytext=(23, 2.3),
                arrowprops=dict(facecolor='black', shrink=0.1),
                )


def get_input_from_bagfile(bagfile_name):
    file_name = bagfile_name.split("/")[-2].split("-")[1]
    input_vector = file_name.split("_")
    input_vector = np.array(list(map(float, input_vector)))
    return input_vector


def compare_results(bagfile_name, read_dist=False, p_control=False):
    # Read simulation data
    odom_data, acc_data = get_mcap_messages(bagfile_name)
    odom_data_size = odom_data.shape[0]
    acc_data_size = acc_data.shape[0]

    # Read dynamic model data
    input_vector = get_input_from_bagfile(bagfile_name)

    if p_control:
        target = [input_vector[0], input_vector[1], input_vector[2]]
        print("Running dynamic model simulation with target: ", input_vector)
    else:
        target = False
        print("Running dynamic model simulation with input: ", input_vector)

    if read_dist:
        dist = True
        filename = bagfile_name
        plot = True
        print("Reading disturbance data from bagfile...")
    else:
        dist = False
        filename = None
        plot = False

    run_simulation(input_vector,
                   p_control=target,
                   dist=dist,
                   filename=filename,
                   plot=plot)

    diff_data = pd.read_csv('disturbances_info/dynamic_model_out.csv')

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

    plot_trajectories(odom_data, pose_diff)

    plt.show()


if __name__ == '__main__':
    if len(sys.argv) > 4 or len(sys.argv) < 2:
        print('Usage: compare_results.py -record.mcap-')
    elif len(sys.argv) == 2:
        compare_results(sys.argv[1])
    elif len(sys.argv) == 3:
        compare_results(sys.argv[1], read_dist=True)
    elif len(sys.argv) == 4:
        compare_results(sys.argv[1], p_control=True, read_dist=True)

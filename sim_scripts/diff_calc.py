import math
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.ticker import MultipleLocator

from disturbances import IntegratedWhiteNoise, read_csv, calculate_wrench
from kinematics import *


# Duplicate function to avoid 'Code Unreachable' bug
def cross(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.cross(a, b)


def vector_to_xy_components(speed, direction):
    x_comp = speed * np.cos(direction)
    y_comp = speed * np.sin(direction)
    return np.array([x_comp, y_comp])


# Eq. 5a
def force_on_point(vel, acc):
    added_mass_force = m_a * (-acc)

    drag_force = Cd * water_density * (R_uc * (H_uc - h) + R_lc * H_lc)
    drag_force = drag_force * np.linalg.norm(-vel) * (-vel)

    # print(f"Added mass: {added_mass_force}")
    # print(f"Drag {drag_force}")
    return added_mass_force + drag_force


def hydrodynamics(vel, acc):
    point_a_force = force_on_point(point_a_vel(vel), point_a_acc(vel, acc))
    point_a_torque = cross(s_a(), point_a_force)
    q_a = [point_a_force, point_a_torque]

    point_b_force = force_on_point(point_b_vel(vel), point_b_acc(vel, acc))
    point_b_torque = cross(s_b(), point_b_force)
    q_b = [point_b_force, point_b_torque]

    point_c_force = force_on_point(point_c_vel(vel), point_c_acc(vel, acc))
    point_c_torque = cross(s_c(), point_c_force)
    q_c = [point_c_force, point_c_torque]

    force = q_a[0] + q_b[0] + q_c[0]
    torque = q_a[1] + q_b[1] + q_c[1]
    return np.append(force, torque)


# (Eq. 8f)
def get_mass_matrix():
    mass_matrix = np.zeros((3, 3))

    mass_matrix[0][0] = mass_matrix[1][1] = m + (3*m_a)
    mass_matrix[0][1] = mass_matrix[1][0] = 0
    mass_matrix[0][2] = mass_matrix[2][0] = 3 * (d_bf - (L_bc / 2)) * m_a
    mass_matrix[1][2] = mass_matrix[2][1] = ((2 * d_ad) - (3 * d_ae)) * m_a
    mass_matrix[2][2] = I_zz + m_a * (- (2*d_ad) + (4*d_ad*d_ae) - 3*(d_ae**2)
                                      - (5/4)*(L_bc**2) + (3*L_bc*d_bf) - (3*(d_bf**2)))
    return mass_matrix


def plot_pos_vel_acc(pos, vel, acc):
    fig, ax = plt.subplots(3, 3, sharex=True, sharey=False)
    ax[0, 0].plot(range(it), pos[0, :])
    ax[0, 1].plot(range(it), pos[1, :])
    ax[0, 2].plot(range(it), pos[2, :])
    ax[1, 0].plot(range(it), vel[0, :])
    ax[1, 1].plot(range(it), vel[1, :])
    ax[1, 2].plot(range(it), vel[2, :])
    ax[2, 0].plot(range(it), acc[0, :])
    ax[2, 1].plot(range(it), acc[1, :])
    ax[2, 2].plot(range(it), acc[2, :])
    ax[0, 0].set_title('X-Axis Position')
    ax[0, 1].set_title('Y-Axis Position')
    ax[0, 2].set_title('Z-Axis Orientation ')
    ax[0, 0].set_ylabel('Distance (m)')

    ax[1, 0].set_title('X-Axis Linear Velocity')
    ax[1, 1].set_title('Y-Axis Linear Velocity')
    ax[1, 2].set_title('Z-Axis Angular Velocity')
    ax[1, 0].set_ylabel('m/s')

    ax[2, 0].set_title('X-Axis Linear Acceleration')
    ax[2, 1].set_title('Y-Axis Linear Acceleration')
    ax[2, 2].set_title('Z-Axis Angular Acceleration')
    ax[2, 0].set_ylabel('m/s^2')

    fig.text(0.5, 0.04, 'Time (ms)', ha='center', va='center', size='large')

    for i in range(3):
        for j in range(3):
            ax[i, j].grid(True)
    fig.set_figwidth(16)
    fig.set_figheight(9)
    plt.show()


def plot_dist(magnitude, direction, title):
    fig, ax = plt.subplots(2, 1, sharex=True)
    ax[0].plot(np.array(range(it-1))/1000, magnitude)
    ax[1].plot(np.array(range(it-1))/1000, direction)
    ax[0].set_ylabel('Magnitude [m/s]')
    ax[1].set_ylabel('Direction [deg]')
    ax[1].set_xlabel('Time [deg]')

    ax[0].grid(True)
    ax[1].grid(True)

    fig.suptitle(title, fontsize=20)
    fig.set_figwidth(16)
    fig.set_figheight(9)
    plt.show()


def big_plots(pos, vel, acc):
    pos_ax, pos_fig = plot3_1(pos, "Position & Orientation")
    vel_ax, vel_fig = plot3_1(vel, "Linear & Angular Velocity")
    acc_ax, acc_fig = plot3_1(acc, "Linear & Angular Acceleration")

    pos_ax[0].set_ylabel('X-Axis Position [m]')
    pos_ax[1].set_ylabel('Y-Axis Position [m]')
    pos_ax[2].set_ylabel('Z-Axis Orientation [deg]')
    pos_ax[2].set_xlabel('Time [s]')

    vel_ax[0].set_ylabel('X-Axis \nLinear Velocity [m/s]')
    vel_ax[1].set_ylabel('Y-Axis \nLinear Velocity [m/s]')
    vel_ax[2].set_ylabel('Z-Axis \nAngular Velocity [rad/s]')
    vel_ax[2].set_xlabel('Time [s]')

    acc_ax[0].set_ylabel('X-Axis Linear\nAcceleration [m/s^2]')
    acc_ax[1].set_ylabel('Y-Axis Linear\nAcceleration [m/s^2]')
    acc_ax[2].set_ylabel('Z-Axis Angular\nAcceleration [rad/s^2]')
    acc_ax[2].set_xlabel('Time [s]')
    plt.show()


def plot3_1(values, title):
    fig, ax = plt.subplots(3, 1, sharex=True, sharey=False)

    ax[0].plot(np.array(range(it)) / 1000, values[0, :])
    ax[1].plot(np.array(range(it)) / 1000, values[1, :])
    ax[2].plot(np.array(range(it)) / 1000, values[2, :])

    fig.set_figwidth(16)
    fig.set_figheight(9)
    fig.suptitle(title, fontsize=20)
    for i in range(3):
        ax[i].grid(True)

    return ax, fig


def radians_to_degrees(radians):
    degrees = math.degrees(radians)
    degrees = (degrees + 180) % 360 - 180
    return degrees


if __name__ == '__main__':
    plt.rcParams["font.family"] = "Times New Roman"
    plt.rcParams.update({'font.size': 15})
    # Disturbances
    current_velocity = IntegratedWhiteNoise(0, 0.5, 0.1, 0.3)
    current_direction = IntegratedWhiteNoise(90, 120, 100, 20)
    current_wrench_info = read_csv("disturbances_info/current_table.csv")

    wind_velocity = IntegratedWhiteNoise(0, 7, 2, 2)
    wind_direction = IntegratedWhiteNoise(30, 60, 40, 20)
    wind_wrench_info = read_csv("disturbances_info/wind_table.csv")

    mass_inv = np.linalg.inv(get_mass_matrix())

    # Iterations
    step = 0.001
    minutes = 10
    it = int(1000 * 60) * minutes

    # Data
    acc = np.zeros((3, it), dtype=np.float64)
    vel = np.zeros((3, it), dtype=np.float64)
    pos = np.zeros((3, it), dtype=np.float64)
    current = np.zeros((3, it))
    wind = np.zeros((3, it))

    current_mag = np.zeros(it-1, dtype=np.float64)
    current_dir = np.zeros(it-1, dtype=np.float64)
    wind_mag = np.zeros(it-1, dtype=np.float64)
    wind_dir = np.zeros(it-1, dtype=np.float64)

    # Thrust input
    engine_thrust = np.array([100, 0, 0])
    # engine_thrust = np.full((3, it), 0)
    # engine_thrust[0, :it // 2] = 500
    # engine_thrust[1, :it // 2] = 500

    for i in range(it - 1):
        wind_mag[i] = wind_velocity.get_value()
        wind_dir[i] = wind_direction.get_value()
        wind[:, i] = calculate_wrench(pos[:, i], vel[:, i], wind_mag[i],
                                      wind_dir[i], wind_wrench_info, mode="wind")

        current_mag[i] = current_velocity.get_value()
        current_dir[i] = current_direction.get_value()
        current[:, i] = calculate_wrench(pos[:, i], vel[:, i], current_mag[i],
                                         current_dir[i], current_wrench_info, mode="current")

        # q_dist = wind[:, i]
        # q_dist = current[:, i]
        q_dist = wind[:, i] + current[:, i]

        acting_forces = engine_thrust + hydrodynamics(vel[:, i], acc[:, i]) + q_dist

        # Acceleration
        acc[:, i + 1] = mass_inv @ acting_forces

        # Velocity
        vel[:, i + 1] = vel[:, i] + step * (mass_inv @ acting_forces)

        # Position
        vel_I = get_rotation_matrix(pos[2, i]) @ vel[:, i]
        pos[:, i + 1] = pos[:, i] + step * vel_I
        pos[2, i] = radians_to_degrees(pos[2, i])

    pos[2, it-1] = radians_to_degrees(pos[2, it-1])

    # print(f"\n ------- Iteration {i} ------- :")
    # print(f"Acting Forces: {acting_forces}")
    # print(f"Acceleration: {acc[:, i + 1]}")
    # print(f"Velocity: {vel[:, i + 1]}")
    # print(f"Position: {pos[:, i + 1]}")
    # print()

    data = {'Position-x': pos[0, :], 'Position-y': pos[1, :], 'Orientation-z': pos[2, :],
            'Velocity-x': vel[0, :], 'Velocity-y': vel[1, :], 'Velocity-z': vel[2, :],
            'Acceleration-x': acc[0, :], 'Acceleration-y': acc[1, :], 'Acceleration-z': acc[2, :]}
    df = pd.DataFrame(data)
    df.to_csv('output.csv', index=False)
    print("Simulation output saved to: output.csv")

    # plot_pos_vel_acc(pos, vel, acc)
    big_plots(pos, vel, acc)
    plot_dist(current_mag, current_dir, "Ocean Current Disturbance")
    plot_dist(wind_mag, wind_dir, "Wind Disturbance")

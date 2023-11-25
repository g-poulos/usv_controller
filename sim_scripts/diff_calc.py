import math
import matplotlib.pyplot as plt
import numpy as np
from disturbances import IntegratedWhiteNoise
import os
from constants import *


def height_above_surface():     # (Eq. 4)
    return H_uc - (1/(R_uc**2)) * (m / (3 * math.pi * water_density) - (R_lc ** 2) * H_lc)


def dist_to_force(speed, direction, model_velocity, mode):   # (Eq. 15)
    C = 1
    relative_angle = direction - psi

    if mode == "wind":
        density = air_density
        At = At_upper
        Al = Al_upper
    elif mode == "current":
        density = water_density
        At = At_lower
        Al = Al_lower
    else:
        print("Invalid force mode")
        return

    relative_velocity = model_velocity[:2] - vector_to_xy_components(speed, direction)
    magnitude = np.linalg.norm(relative_velocity)
    force_x = 0.5 * C * density * (magnitude**2) * At
    force_y = 0.5 * C * density * (magnitude**2) * Al
    moment_z = 0.5 * C * density * (magnitude**2) * Al * L
    return np.array([force_x, force_y, moment_z])


def vector_to_xy_components(speed, direction):
    x_comp = speed * np.cos(direction)
    y_comp = speed * np.sin(direction)
    return np.array([x_comp, y_comp])


# (Eq. 1d)
def get_rotation_matrix(angle):
    return np.array([[np.cos(angle), -np.sin(angle), 0],
                     [np.sin(angle), np.cos(angle), 0],
                     [0, 0, 1]])


def force_on_point(acc):
    # TODO: Add velocities and accelerations
    added_mass_force = Ca * math.pi * water_density * ((R_uc ** 2) * (H_uc - h) + (R_lc ** 2) * H_lc)
    inertia_force = math.pi * water_density * ((R_uc ** 2) * (H_uc - h) + (R_lc ** 2) * H_lc)
    drag_force = Cd * water_density * (R_uc * (H_uc - h) + R_lc * H_lc)

    return added_mass_force + inertia_force + drag_force


# (Eq. 8f)
def get_mass_matrix():
    mass_matrix = np.zeros((3, 3))
    m_a = - Ca * math.pi * 1025 * (R_uc ** 2 * (H_uc + R_lc ** 2 * H_lc))

    mass_matrix[0][0] = mass_matrix[1][1] = m - m_a
    mass_matrix[0][1] = mass_matrix[1][0] = 0
    mass_matrix[0][2] = mass_matrix[2][0] = 3 * (d_bf - (L_bc / 2)) * m_a
    mass_matrix[1][2] = mass_matrix[2][1] = (2 * d_ad - 3 * d_ae) * m_a
    mass_matrix[2][2] = I_zz + m_a * (- 2*d_ad + 4*d_ad*d_ae - 3*(d_ae**2) - (5/4)*(L_bc**2) + 3*L_bc*d_bf - 3*(d_bf**2))
    return mass_matrix


# (Eq. 7b)
def get_engine_vectored_thrust(i):
    return np.exp(i)
    # return np.array([100, 100, 0])


def plot_pos_vel_acc(pos, vel, acc):
    fig, ax = plt.subplots(3, 3, sharex=True)
    ax[0, 0].plot(range(it), pos[0, :])
    ax[0, 1].plot(range(it), pos[1, :])
    ax[0, 2].plot(range(it), pos[2, :])
    ax[1, 0].plot(range(it), vel[0, :])
    ax[1, 1].plot(range(it), vel[1, :])
    ax[1, 2].plot(range(it), vel[2, :])
    ax[2, 0].plot(range(it), acc[0, :])
    ax[2, 1].plot(range(it), acc[1, :])
    ax[2, 2].plot(range(it), acc[2, :])
    ax[0, 0].set_title('Position X-Axis')
    ax[0, 1].set_title('Position Y-Axis')
    ax[0, 2].set_title('Orientation Z-Axis')
    ax[1, 0].set_title('Linear Velocity X-Axis')
    ax[1, 1].set_title('Linear Velocity Y-Axis')
    ax[1, 2].set_title('Angular Velocity Z-Axis')
    for i in range(3):
        for j in range(3):
            ax[i, j].grid(True)
    fig.set_figwidth(16)
    fig.set_figheight(9)
    plt.show()


def plot_dist(current, wind):
    fig, ax = plt.subplots(2, 3, sharex=True)
    ax[0, 0].plot(range(it), current[0, :])
    ax[0, 1].plot(range(it), current[1, :])
    ax[0, 2].plot(range(it), current[2, :])
    ax[1, 0].plot(range(it), wind[0, :])
    ax[1, 1].plot(range(it), wind[1, :])
    ax[1, 2].plot(range(it), wind[2, :])

    fig.set_figwidth(16)
    fig.set_figheight(9)
    plt.show()


if __name__ == '__main__':
    current_velocity = IntegratedWhiteNoise(0, 0.514, 0.1, 1)
    current_direction = IntegratedWhiteNoise(0, 360, 140, 6)
    wind_velocity = IntegratedWhiteNoise(0, 7.716, 2, 2)
    wind_direction = IntegratedWhiteNoise(0, 360, 270, 6)
    mass_inv = np.linalg.inv(get_mass_matrix())

    engine_thrust = np.array([10, 10, 0])

    h = 0.01
    it = int((0.01 * 100) * 3600)

    acc = np.zeros((3, it))
    vel = np.zeros((3, it))
    pos = np.zeros((3, it))
    current = np.zeros((3, it))
    wind = np.zeros((3, it))

    for i in range(it - 1):
        wind_speed = wind_velocity.get_value()
        wind_dir = wind_direction.get_value()
        wind[:, i] = dist_to_force(wind_speed, wind_dir, vel[:, i], mode="wind")

        current_speed = current_velocity.get_value()
        current_dir = current_direction.get_value()
        current[:, i] = dist_to_force(current_speed, current_dir, vel[:, i], mode="current")

        acting_forces = engine_thrust + wind[:, i]
        acc[:, i + 1] = mass_inv.dot(acting_forces)                             # Acceleration
        vel[:, i + 1] = vel[:, i] + h * mass_inv.dot(acting_forces)             # Velocity
        pos[:, i + 1] = pos[:, i] + h * mass_inv.dot(acting_forces) * (i * h)   # Position

    plot_pos_vel_acc(pos, vel, acc)
    plot_dist(current, wind)

    # fig, ax = plt.subplots(2, 1, sharex=True)
    # ax[0].plot(cv, label='Current Speed')
    # ax[0].set_ylabel('Speed (m/s)')
    #
    # ax[1].plot(cd, label='Current Direction')
    # ax[1].set_ylabel('Direction (degrees)')
    # plt.show()




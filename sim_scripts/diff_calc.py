import math
import matplotlib.pyplot as plt
import numpy as np
from disturbances import IntegratedWhiteNoise
import os
from constants import *


def height_above_surface():     # (Eq. 4)
    return H_uc - (1/(R_uc**2)) * (m / (3 * math.pi * water_density) - (R_lc ** 2) * H_lc)


def wind_force(speed, direction, velocity):   # (Eq. 15)
    C = 1
    relative_angle = direction - psi
    relative_velocity = vector_to_xy_components(speed, direction) - velocity[:2]
    magnitude = np.linalg.norm(relative_velocity)
    force_x = 0.5 * C * air_density * (magnitude**2) * At_upper
    force_y = 0.5 * C * air_density * (magnitude**2) * Al_upper
    moment_z = 0.5 * C * air_density * (magnitude**2) * Al_upper * L
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


if __name__ == '__main__':
    current_velocity = IntegratedWhiteNoise(0, 0.514, 0.1, 0.001)
    current_direction = IntegratedWhiteNoise(0, 360, 100, 1)
    wind_velocity = IntegratedWhiteNoise(0, 7.716, 2, 2)
    wind_direction = IntegratedWhiteNoise(0, 360, 200, 7)
    mass_inv = np.linalg.inv(get_mass_matrix())

    engine_thrust = np.array([10, 10, 0])

    h = 0.01
    it = 1000

    vel = np.zeros((3, it))
    pos = np.zeros((3, it))

    for i in range(it - 1):
        v = wind_velocity.get_value()
        d = wind_direction.get_value()
        vec = vector_to_xy_components(v, d)
        wind = wind_force(vec, d, vel[:, i])

        acting_forces = engine_thrust + wind

        vel[:, i + 1] = vel[:, i] + h * mass_inv.dot(acting_forces)             # Velocity
        pos[:, i + 1] = pos[:, i] + h * mass_inv.dot(acting_forces) * (i * h)   # Position

    fig, ax = plt.subplots(2, 3, sharex=True, sharey=True)
    ax[0, 0].plot(range(it), pos[0, :])
    ax[0, 1].plot(range(it), pos[1, :])
    ax[0, 2].plot(range(it), pos[2, :])

    ax[1, 0].plot(range(it), vel[0, :])
    ax[1, 1].plot(range(it), vel[1, :])
    ax[1, 2].plot(range(it), vel[2, :])
    ax[0, 0].set_title('Position X-Axis')
    ax[0, 1].set_title('Position Y-Axis')
    ax[0, 2].set_title('Orientation Z-Axis')
    ax[1, 0].set_title('Linear Velocity X-Axis')
    ax[1, 1].set_title('Linear Velocity Y-Axis')
    ax[1, 2].set_title('Angular Velocity Z-Axis')

    for i in range(2):
        for j in range(3):
            ax[i, j].grid(True)
    fig.set_figwidth(15)
    plt.show()



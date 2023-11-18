import math
import matplotlib.pyplot as plt
import numpy as np
from disturbances import IntegratedWhiteNoise
import os
from constants import *


def height_above_surface():     # (Eq. 4)
    return H_uc - (1/(R_uc**2)) * (m / (3 * math.pi * water_density) - (R_lc ** 2) * H_lc)


def wind_force(speed, direction, velocity):   # (Eq. 15)

    # TODO: Check relative velocities
    C = 1
    relative_angle = direction - psi
    # relative_speed_x = speed[0] - velocity[0]
    # relative_speed_y = speed[1] - velocity[1]
    relative_speed_x = velocity[0] - speed[0]
    relative_speed_y = velocity[1] - speed[1]
    force_x = 0.5 * C * air_density * (relative_speed_x**2) * A_t
    force_y = 0.5 * C * air_density * (relative_speed_y**2) * A_l
    moment_z = 0.5 * C * air_density * (relative_speed_y**2) * A_l * L
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

    # for i in range(100):
    #     v = wind_velocity.get_value()
    #     d = wind_direction.get_value()
    #     vec = vector_to_xy_components(v, d)
    #     f = wind_force(vec, d)
    #     print(f)

    # mass_inv = np.linalg.inv(get_mass_matrix())
    # init_vel = np.array([0, 0, 0])
    # h = 0.01
    #
    # vel_lst = np.array([0, 0, 0])
    # it = 10000
    # for i in range(it):
    #     v = wind_velocity.get_value()
    #     d = wind_direction.get_value()
    #     vec = vector_to_xy_components(v, d)
    #     f = wind_force(vec, d)
    #
    #     vel = init_vel + h * mass_inv.dot(get_engine_vectored_thrust())
    #     init_vel = np.linalg.inv(get_rotation_matrix(psi)).dot(vel)
    #     vel_lst = np.vstack([vel_lst, vel])
    #
    # fig, (ax1, ax2, ax3) = plt.subplots(3)
    # ax1.plot(range(it+1), vel_lst[:, 0])
    # ax2.plot(range(it+1), vel_lst[:, 1])
    # ax3.plot(range(it+1), vel_lst[:, 2])
    # plt.show()



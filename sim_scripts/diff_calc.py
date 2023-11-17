import math

import matplotlib.pyplot as plt
import numpy as np
from bisect import bisect_left
from disturbances import IntegratedWhiteNoise
import os

############### Dimensions and Mass ###############
m = 425             # Mass of the structure
R_uc = 0.22         # Upper cylinder radius
H_uc = 0.65         # Upper cylinder height
R_lc = 0.35         # Lower cylinder radius
H_lc = 0.30         # Lower cylinder height

L_bc = 3.5          # Length of the triangular base
L_ac = 4.5          # Length of the triangular side

d_ad = math.sqrt(L_ac ** 2 - (L_bc / 2) ** 2)
d_bf = L_bc / 2
d_ae = d_ad * 2 / 3
L = np.sqrt((L_ac**2) - ((L_bc/2)**2))  # Overall length

I_zz = 1488.504

A_t = 5.587971908895481    # Transverse projected area
A_l = 6.08997115068253     # Lateral projected area


############### Coefficients and Constants ###############
water_density = 1025    # Water density
air_density = 1.222     # Air density
Ca = 0.8                # Added mass coefficient
Cd = 0.8                # Drag coefficient

############### Initial Values ###############
x_s = 5             # (m)     Start position in X axis
y_s = 5             # (m)     Start position in Y axis
psi_s = 10          # (deg)   Start direction
u_s = 0.10          # (m/s)   Start surge velocity
v_s = -0.10         # (m/s)   Start sway velocity
r_s = 0.01          # (rad/s) Start yaw velocity


############### Current Values ###############
x = x_s
y = y_s
psi = psi_s
u = u_s
v = v_s
r = r_s


def height_above_surface():     # (Eq. 4)
    return H_uc - (1/(R_uc**2)) * (m / (3 * math.pi * water_density) - (R_lc ** 2) * H_lc)


def wind_force(speed, direction):   # (Eq. 15)
    C = 1.2
    relative_angle = direction - psi
    relative_speed_x = speed[0] - u
    relative_speed_y = speed[1] - v
    force_x = 0.5 * C * relative_angle * air_density * (relative_speed_x**2) * A_t
    force_y = 0.5 * C * relative_angle * air_density * (relative_speed_y**2) * A_l
    moment_z = 0.5 * C * relative_angle * air_density * (relative_speed_y**2) * A_l * L
    return np.array([force_x, force_y, moment_z])


def vector_to_xy_components(speed, direction):
    x_comp = speed * np.cos(direction)
    y_comp = speed * np.sin(direction)
    return np.array([x_comp, y_comp])



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
def get_engine_vectored_thrust():
    return np.array([100, 100, 10])


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

    mass_inv = np.linalg.inv(get_mass_matrix())
    init_vel = np.array([0, 0, 0])
    h = 0.01

    vel_lst = np.array([0, 0, 0])
    for i in range(256):
        v = wind_velocity.get_value()
        d = wind_direction.get_value()
        vec = vector_to_xy_components(v, d)
        f = wind_force(vec, d)

        vel = init_vel + h * mass_inv.dot(get_engine_vectored_thrust() + f)
        init_vel = vel
        vel_lst = np.vstack([vel_lst, vel])

    fig, (ax1, ax2, ax3) = plt.subplots(3)
    ax1.plot(range(257), vel_lst[:, 0])
    ax2.plot(range(257), vel_lst[:, 1])
    ax3.plot(range(257), vel_lst[:, 2])
    plt.show()



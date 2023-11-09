import math
import numpy as np
import random
import matplotlib.pyplot as plt


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

I_zz = 2.82698738e+03

############### Coefficients and Constants ###############
density = 1025      # Water density
Ca = 0.8            # Added mass coefficient
Cd = 0.8            # Drag coefficient

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


class IntegratedWhiteNoise:
    def __init__(self, min_val, max_val, stddev):
        self.min_val = min_val
        self.max_val = max_val
        self.stddev = stddev
        self.dt = 0.01
        self.prev_val = 0.5 * (max_val - min_val)

    def get_value(self):
        next_value = self.prev_val + self.dt * random.gauss(0, self.stddev)

        if next_value > self.max_val or next_value < self.min_val:
            next_value = next_value - self.dt * random.gauss(0, self.stddev)

        self.prev_val = next_value
        return next_value


def height_above_surface():     # (Eq. 4)
    return H_uc - (1/(R_uc**2)) * (m/(3 * math.pi * density) - (R_lc**2)*H_lc)


def force_on_a(h, ):            # (Eq. 5a)
    # TODO: Add velocities and accelerations
    added_mass_force = Ca * math.pi * density * ((R_uc**2) * (H_uc - h) + (R_lc**2)*H_lc)
    inertia_force = math.pi * density * ((R_uc**2) * (H_uc - h) + (R_lc**2)*H_lc)
    drag_force = Cd * density * (R_uc * (H_uc - h) + R_lc*H_lc)

    return added_mass_force + inertia_force + drag_force


def velocity_a():        # (Eq. 3a)
    x_vel = u + r * ((L_bc/2) - d_bf)
    y_vel = v + r * d_ae
    return x_vel, y_vel


def position_a():
    # Initial
    h = 0.01
    x_pos = d_ae
    y_pos = d_bf - (L_bc/2)

    # First step
    x_v, y_v = velocity_a()
    x_pos_temp = x_pos
    y_pos_temp = y_pos

    # Continue
    x = [x_pos]
    y = [y_pos]
    for i in range(1000):
        x_pos_temp = x_pos_temp + h * x_v
        y_pos_temp = y_pos_temp + h * y_v
        x.append(x_pos_temp)
        y.append(y_pos_temp)

    return x, y


def position():
    x_dot = np.transpose(np.array([x, y, psi]))
    rotation = np.array([[np.cos(psi), -np.sin(psi), 0],
                        [np.sin(psi), np.cos(psi), 0],
                        [0, 0, 1]])
    velocity = np.transpose(np.array([u, v, r]))
    print(x_dot)
    print(rotation)
    print(velocity)


if __name__ == '__main__':
    current_velocity = IntegratedWhiteNoise(0, 0.514, 0.001)
    current_direction = IntegratedWhiteNoise(0, 360, 1)
    wind_velocity = IntegratedWhiteNoise(0, 7.716, 0.04)
    wind_direction = IntegratedWhiteNoise(0, 360, 1)

    # r = 1000000
    # cv = []
    # cd = []
    # wv = []
    # wd = []
    # for i in range(r):
    #     cv.append(current_velocity.get_value())
    #     wv.append(wind_velocity.get_value())
    #
    # plt.plot(range(r), cv)
    # plt.plot(range(r), wv)
    # plt.show()

    position()

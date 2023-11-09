import math
import numpy as np
import random
import matplotlib.pyplot as plt


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

density = 1025      # Water density
Ca = 0.8            # Added mass coefficient
Cd = 0.8            # Drag coefficient


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


def force_on_a(h, ):              # (Eq. 5a)
    # TODO: Add velocities and accelerations
    added_mass_force = Ca * math.pi * density * ((R_uc**2) * (H_uc - h) + (R_lc**2)*H_lc)
    inertia_force = math.pi * density * ((R_uc**2) * (H_uc - h) + (R_lc**2)*H_lc)
    drag_force = Cd * density * (R_uc * (H_uc - h) + R_lc*H_lc)

    return added_mass_force + inertia_force + drag_force


if __name__ == '__main__':
    dist = IntegratedWhiteNoise(0, 10, 0.1)

    r = 1000000
    x = []
    for i in range(r):
        x.append(dist.get_value())

    plt.plot(range(r), x)
    plt.show()

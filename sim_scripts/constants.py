import numpy as np


############## Dimensions and Mass ###############
m = 425             # Mass of the structure
R_uc = 0.22         # Upper cylinder radius
H_uc = 0.65         # Upper cylinder height
R_lc = 0.35         # Lower cylinder radius
H_lc = 0.30         # Lower cylinder height

L_bc = 3.5          # Length of the triangular base
L_ac = 4.5          # Length of the triangular side

d_ad = np.sqrt(L_ac ** 2 - (L_bc / 2) ** 2)
d_bf = L_bc / 2
d_ae = d_ad * 2 / 3
L = np.sqrt((L_ac**2) - ((L_bc/2)**2))  # Overall length

I_zz = 1488.504

A_t = 3.9311583257344327    # Transverse projected area
A_l = 4.433157604350512     # Lateral projected area


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

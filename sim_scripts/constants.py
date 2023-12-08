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
d_ae = d_ad * (2 / 3)
L = np.sqrt((L_ac**2) - ((L_bc/2)**2))  # Overall length

I_zz = 1488.504

# Surface part
At_upper = 1.470668406331706    # Transverse projected area
Al_upper = 1.5663578081005782   # Lateral projected area

# Submerged part
At_lower = 0.6294737439566249   # Transverse projected area
Al_lower = 0.4196491616786311   # Lateral projected area

############### Coefficients and Constants ###############
water_density = 1025    # Water density
air_density = 1.222     # Air density
Ca = 0.6                # Added mass coefficient
Cd = 0.8                # Drag coefficient

# Eq. 4
h = H_uc - (1/(R_uc**2)) * (m / (3 * np.pi * water_density) - (R_lc ** 2) * H_lc)
m_a = Ca * np.pi * water_density * (((R_uc**2) * (H_uc - h)) + ((R_lc**2) * H_lc))

############### Initial Values ###############
x_s = 5             # (m)     Start position in X axis
y_s = 5             # (m)     Start position in Y axis
psi_s = 10          # (deg)   Start direction
u_s = 0.10          # (m/s)   Start surge velocity
v_s = -0.10         # (m/s)   Start sway velocity
r_s = 0.01          # (rad/s) Start yaw velocity
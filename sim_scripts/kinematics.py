from constants import *

s_a = [d_ae, d_bf-(L_bc/2)]         # Eq. 2a
s_b = [-(d_ad-d_ae), d_bf]          # Eq. 2b
s_c = [-(d_ad-d_ae), d_bf-L_bc]     # Eq. 2c


# Eq. 3a
def point_a_vel(vel):
    return np.array([vel[0] + vel[2]*((L_bc/2)-d_bf), vel[1] + vel[2]*d_ae])


# Eq. 3b
def point_a_acc(vel, acc):
    return np.array([acc[0] - acc[2]*(d_bf - (L_bc/2)) - (vel[2]**2)*d_ae,
                     acc[1] + acc[2]*d_ae - (vel[2]**2)*(d_bf - (L_bc/2))])


# Eq. 3c
def point_b_vel(vel):
    return np.array([vel[0] - vel[2]*d_bf, vel[1] - vel[2]*(d_ad-d_ae)])


# Eq. 3d
def point_b_acc(vel, acc):
    return np.array([acc[0] - acc[2]*d_bf + (vel[2]**2)*(d_ad-d_ae),
            acc[1] - acc[2]*(d_ad-d_ae) - (vel[2]**2)*d_bf])


# Eq. 3e
def point_c_vel(vel):
    return np.array([vel[0] + vel[2]*(L_bc-d_bf), vel[1] - vel[2]*(d_ad-d_ae)])


# Eq. 3f
def point_c_acc(vel, acc):
    return np.array([acc[0] + acc[2]*(L_bc-d_bf) + (vel[2]**2)*(d_ad-d_ae),
            acc[1] - acc[2]*(d_ad-d_ae) + (vel[2]**2)*(L_bc-d_bf)])

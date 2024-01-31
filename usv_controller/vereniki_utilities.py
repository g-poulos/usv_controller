import numpy as np
from std_msgs.msg import Float64

L_bc = 3.5          # Length of the triangular base
L_ac = 4.5          # Length of the triangular side

d_ad = np.sqrt(L_ac ** 2 - (L_bc / 2) ** 2)
d_bf = L_bc / 2
d_ae = d_ad * (2 / 3)


def get_thrust_7a(input_vec):
    J = np.array([[1, 0, -(d_bf - (L_bc / 2))],
                  [0, -1, -d_ae],
                  [1, 0, -d_bf],
                  [0, -1, (d_ad - d_ae)],
                  [1, 0, (L_bc - d_bf)],
                  [0, -1, (d_ad - d_ae)]])
    J = J.T

    J_plus = J.T @ np.linalg.inv(J @ J.T)
    return J_plus @ input_vec


def cartesian_to_polar(x, y):
    magnitude = np.sqrt(x**2 + y**2)
    theta = np.arctan2(x, y) - np.pi/2
    return magnitude, theta


def thrust_to_rotations(thrust):
    rad_s = np.sqrt(abs(thrust) / (1025 * 0.01 * (0.48 ** 4)))
    if thrust < 0:
        rad_s = -rad_s
    return rad_s


def quaternion_to_yaw(quaternion):
    q0, q1, q2, q3 = quaternion
    yaw = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2**2 + q3**2))
    return np.degrees(yaw)


def get_thrust_msgs(input_vector):
    msgs = get_thrust_7a(input_vector)

    thrustA_msg = Float64()
    directionA_msg = Float64()
    thrustB_msg = Float64()
    directionB_msg = Float64()
    thrustC_msg = Float64()
    directionC_msg = Float64()

    thrustA, thetaA = cartesian_to_polar(msgs[0], msgs[1])
    thrustB, thetaB = cartesian_to_polar(msgs[2], msgs[3])
    thrustC, thetaC = cartesian_to_polar(msgs[4], msgs[5])

    thrustA_msg.data = thrust_to_rotations(thrustA)
    directionA_msg.data = thetaA
    thrustB_msg.data = thrust_to_rotations(thrustB)
    directionB_msg.data = thetaB
    thrustC_msg.data = thrust_to_rotations(thrustC)
    directionC_msg.data = thetaC

    return ([directionA_msg, directionB_msg, directionC_msg],
            [thrustA_msg, thrustB_msg, thrustC_msg])

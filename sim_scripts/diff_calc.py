import math
import matplotlib.pyplot as plt
from disturbances import IntegratedWhiteNoise, WrenchInfo, read_csv, calculate_wrench
import os
from kinematics import *


def dist_to_force(speed, direction, model_velocity, mode):   # (Eq. 15)
    C = 1

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

    relative_velocity = vector_to_xy_components(speed, direction) - model_velocity[:2]
    x_mag = np.linalg.norm(relative_velocity[0])
    y_mag = np.linalg.norm(relative_velocity[1])

    magnitude = np.linalg.norm(relative_velocity)
    force_x = 0.5 * C * density * relative_velocity[0] * x_mag * At
    force_y = 0.5 * C * density * relative_velocity[1] * y_mag * Al
    moment_z = 0.5 * C * density * (magnitude**2) * Al * L

    # print(f"Model Vel: {model_velocity}")
    # print(f"Dist Vel: {vector_to_xy_components(speed, direction)}")
    # print(f"Relative Vel: {relative_velocity}")
    # print(f"Magnitude: {magnitude}")

    return np.array([force_x, force_y, 0])


def vector_to_xy_components(speed, direction):
    x_comp = speed * np.cos(direction)
    y_comp = speed * np.sin(direction)
    return np.array([x_comp, y_comp])


# Eq. 5a
def force_on_point(vel, acc):
    magnitude = np.linalg.norm(vel)
    h = height_above_surface()
    water_acc = np.array([0, 0])        # TODO: Add water vel

    added_mass = Ca * math.pi * water_density * ((R_uc**2) * (H_uc - h) + (R_lc**2) * H_lc)
    added_mass_force = added_mass * (-acc)

    # inertia_force = math.pi * water_density * ((R_uc ** 2) * (H_uc - h) + (R_lc ** 2) * H_lc)
    # inertia_force = inertia_force * (-water_acc)

    drag_force = Cd * water_density * (R_uc * (H_uc - h) + R_lc * H_lc)
    drag_force = drag_force * magnitude * (-vel)
    print(f"Added mass: {added_mass_force}")
    # print(f"Inertia {inertia_force}")
    # print(f"Drag {drag_force}")
    return drag_force


def hydrodynamics(vel, acc):
    point_a_force = force_on_point(point_a_vel(vel), point_a_acc(vel, acc))
    point_a_torque = np.cross(s_a(), point_a_force)
    q_a = [point_a_force, point_a_torque]

    point_b_force = force_on_point(point_b_vel(vel), point_b_acc(vel, acc))
    point_b_torque = np.cross(s_b(), point_b_force)
    q_b = [point_b_force, point_b_torque]

    point_c_force = force_on_point(point_c_vel(vel), point_c_acc(vel, acc))
    point_c_torque = np.cross(s_c(), point_c_force)
    q_c = [point_c_force, point_c_torque]

    force = q_a[0] + q_b[0] + q_c[0]
    torque = q_a[1] + q_b[1] + q_c[1]
    # print()
    # print(q_a[0], q_b[0], q_c[0])
    # print(q_a[1], q_b[1], q_c[1])
    return np.append(force, torque)


def q_B(r):
    h = height_above_surface()

    added_mass = - Ca * math.pi * water_density * ((R_uc**2) * (H_uc - h) + (R_lc**2) * H_lc)
    q_b = np.array([added_mass * ((2*d_ad) - (3*d_ae)) * (r**2),
                    added_mass * (((3/2)*L_bc)-(3*d_bf)) * (r**2),
                    0])
    print(q_b)
    return q_b


# (Eq. 8f)
def get_mass_matrix():
    mass_matrix = np.zeros((3, 3))
    h = height_above_surface()
    m_a = - Ca * math.pi * water_density * ((R_uc**2) * (H_uc - h) + (R_lc**2) * H_lc)

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
    fig, ax = plt.subplots(3, 3, sharex=True, sharey=False)
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

    wrench_info = read_csv("current_table.csv")

    engine_thrust = np.array([0, 200, 0])

    step = 0.01
    it = int((0.01 * 100) * 20)

    acc = np.zeros((3, it))
    vel = np.zeros((3, it))
    pos = np.zeros((3, it))
    current = np.zeros((3, it))
    wind = np.zeros((3, it))

    for i in range(it - 1):
        # wind_speed = wind_velocity.get_value()
        # wind_dir = wind_direction.get_value()
        # wind[:, i] = dist_to_force(wind_speed, wind_dir, vel[:, i], mode="wind")

        # current_speed = current_velocity.get_value()
        # current_dir = current_direction.get_value()
        # current[:, i] = dist_to_force(current_speed, current_dir, vel[:, i], mode="current")
        #
        # calculate_wrench(pos[:, i], vel[:2, i], current_speed, current_dir, wrench_info)
        #
        # q_dist = current[:, i]
        # print(hydrodynamics(vel[:, i], acc[:, i]))

        # Acceleration
        acting_forces = engine_thrust + hydrodynamics(vel[:, i], acc[:, i]) + q_B(vel[2, i])
        acc[:, i + 1] = mass_inv.dot(acting_forces)

        # Velocity
        vel[:, i + 1] = vel[:, i] + step * mass_inv.dot(acting_forces)

        # Position
        vel_I = get_rotation_matrix(pos[2, i]).dot(vel[:, i])
        pos[:, i + 1] = pos[:, i] + step * vel_I * (i * step)


    plot_pos_vel_acc(pos, vel, acc)
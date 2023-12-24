import random
import matplotlib.pyplot as plt
import numpy as np
from constants import *


class IntegratedWhiteNoise:
    def __init__(self, min_val, max_val, init_val, stddev):
        self.min_val = min_val
        self.max_val = max_val
        self.stddev = stddev
        self.dt = 0.01
        self.prev_val = init_val

    def get_value(self):
        next_value = self.prev_val + self.dt * random.gauss(0, self.stddev)

        if next_value > self.max_val or next_value < self.min_val:
            next_value = self.prev_val - self.dt * random.gauss(0, self.stddev)

        self.prev_val = next_value
        return next_value


class WrenchInfo:
    def __init__(self, angle, force_area, torque_area, offset):
        self.angle = angle
        self.force_area = force_area
        self.torque_area = torque_area
        self.offset = offset

    def __str__(self):
        return_str = ""
        for i in range(len(self.angle)):
            return_str += f"{self.angle[i]} {self.force_area[i]} {self.torque_area[i]} {self.offset[i]}\n"
        return return_str


def read_csv(filename):
    f = open(filename, "r")
    angle_list = []
    force_area_list = []
    torque_area_list =[]
    offset_list = []

    size = f.readline()
    # print(f"Size: {size}")

    for line in f:
        angle, force_a, torque_a, offset = line.split(",")
        angle_list.append(float(angle))
        force_area_list.append(float(force_a))
        torque_area_list.append(float(torque_a))
        offset_list.append(float(offset))

        # print(angle, force_a, torque_a, offset)
    return WrenchInfo(angle_list, force_area_list, torque_area_list, offset_list)


def find_closest(arr, target):
    left, right = 0, len(arr) - 1
    while left < right:
        if abs(arr[left] - target) <= abs(arr[right] - target):
            right -= 1
        else:
            left += 1
    return left


def calculate_wrench(model_pose, model_vel, force_speed, force_direction, wrench_info, mode):
    force_vel = np.array([force_speed * np.cos(force_direction),
                          force_speed * np.sin(force_direction)])
    if mode == "wind":
        fluid_dens = air_density
    else:
        fluid_dens = water_density

    relative_vel = force_vel - model_vel[:2]
    relative_angle = (force_direction * (np.pi/180)) - model_pose[2]
    index = find_closest(wrench_info.angle, relative_angle)

    force_surface = wrench_info.force_area[index]
    torque_surface = wrench_info.torque_area[index]
    offset = wrench_info.offset[index]

    # Force
    force = 0.5 * fluid_dens * Cd * relative_vel * np.linalg.norm(relative_vel) * force_surface

    # Torque
    torque_force = 0.5 * fluid_dens * Cd * relative_vel * np.linalg.norm(relative_vel) * torque_surface
    torque = np.linalg.norm(torque_force) * offset

    return np.array([force[0], force[1], torque])


def test_dist():
    current_velocity = IntegratedWhiteNoise(0, 0.514, 0.1, 0.001)
    current_direction = IntegratedWhiteNoise(0, 360, 100, 1)
    wind_velocity = IntegratedWhiteNoise(0, 10, 3, 0.3)
    wind_direction = IntegratedWhiteNoise(200, 360, 200, 7)

    wind_info = read_csv("wind_table.csv")
    current_info = read_csv("current_table.csv")

    minutes = 60
    # plot_dist(minutes, wind_direction, wind_velocity, wind_info, mode="wind")
    # plot_dist(minutes, current_direction, current_velocity, current_info, mode="current")
    plot_mag_dir(minutes, wind_direction, wind_velocity)


def plot_dist(minutes, direction_distribution, speed_distribution, wrench_info, mode):
    time = int(100 * 60) * minutes
    speed = []
    direction = []
    f = np.zeros((3, time))
    for i in range(time):
        speed_val = speed_distribution.get_value()
        direction_val = direction_distribution.get_value()
        f[:, i] = calculate_wrench(np.array([0, 0, 0]), np.array([0, 0, 0]),
                                   speed_val, direction_val, wrench_info, mode=mode)
        speed.append(speed_val)
        direction.append(direction_val)
    fig, ax = plt.subplots(5, 1, sharex=True)
    ax[0].plot(speed, label='Wind Speed')
    ax[0].set_ylabel('Speed (m/s)')
    ax[1].plot(direction, label='Wind Direction')
    ax[1].set_ylabel('Direction (degrees)')
    ax[2].plot(f[0, :], label='X Force')
    ax[2].set_ylabel('X Force')
    ax[3].plot(f[1, :], label='Y Force')
    ax[3].set_ylabel('Y Force')
    ax[4].plot(f[2, :], label='Z Torque')
    ax[4].set_ylabel('Z Torque')
    ax[4].set_xlabel('Time (ms)')
    fig.set_figwidth(16)
    fig.set_figheight(9)
    fig.suptitle(mode)
    plt.show()


def plot_mag_dir(minutes, direction_distribution, speed_distribution):
    time = int(100 * 60) * minutes
    time_stamps = []
    speed = []
    direction = []
    for i in range(time):
        time_stamps.append(i)
        speed_val = speed_distribution.get_value()
        direction_val = direction_distribution.get_value()
        speed.append(speed_val)
        direction.append(direction_val)

    csfont = {'fontname': 'fonts/afm/ptmr8a.afm'}
    hfont = {'fontname': 'Helvetica'}

    time_stamps = np.array(time_stamps)
    time_stamps = time_stamps * 10e-3
    fig, ax = plt.subplots(2, 1, sharex=True)
    ax[0].plot(time_stamps, speed, label='Wind Speed')
    ax[0].set_ylabel('Magnitude (m/s)', fontsize=16)
    ax[1].plot(time_stamps, direction, label='Wind Direction')
    ax[1].set_ylabel('Direction (degrees)', fontsize=16)
    ax[0].grid()
    ax[1].grid()
    ax[1].set_xlabel('Time (s)', fontsize=16)
    fig.set_figwidth(16)
    fig.set_figheight(9)
    # fig.suptitle('Wind', fontsize=24)
    plt.show()


if __name__ == '__main__':

    test_dist()
    # wrench_info = read_csv("wind_table.csv")
    # i = find_closest(wrench_info.angle, 0.56)
    # print(wrench_info)
    # print(wrench_info.angle[i])
import random
import matplotlib.pyplot as plt
import numpy as np


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
            next_value = next_value - self.dt * random.gauss(0, self.stddev)

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


def calculate_wrench(model_pose, model_vel, force_speed, force_direction, wrench_info):

    force_vel = np.array([force_speed * np.cos(force_direction),
                          force_speed * np.sin(force_direction)])

    relative_vel = force_vel - model_vel
    relative_angle = force_direction - model_pose[2]
    table_index = find_closest(wrench_info.angle, force_direction)

    print(force_direction)
    print(model_pose[2])
    print(relative_angle)


def test_dist():
    current_velocity = IntegratedWhiteNoise(0, 0.514, 0.1, 0.001)
    current_direction = IntegratedWhiteNoise(0, 360, 100, 1)
    wind_velocity = IntegratedWhiteNoise(0, 7.716, 2, 2)
    wind_direction = IntegratedWhiteNoise(0, 360, 200, 25)

    #      one minute           * x
    time = int(0.01 * 100 * 60) * 60
    cv = []
    cd = []
    wv = []
    wd = []
    f = []
    for i in range(time):
        vel = wind_velocity.get_value()
        dir = wind_direction.get_value()
        wv.append(vel)
        wd.append(dir)
        vel = current_velocity.get_value()
        dir = current_direction.get_value()
        cv.append(vel)
        cd.append(dir)

    fig, ax = plt.subplots(4, 1, sharex=True)
    ax[0].plot(wv, label='Wind Speed')
    ax[0].set_ylabel('Speed (m/s)')

    ax[1].plot(wd, label='Wind Direction')
    ax[1].set_ylabel('Direction (degrees)')

    ax[2].plot(cv, label='Current Speed')
    ax[2].set_ylabel('Speed (m/s)')

    ax[3].plot(cd, label='Current Direction')
    ax[3].set_ylabel('Direction (degrees)')

    fig.set_figwidth(16)
    fig.set_figheight(9)
    plt.show()


if __name__ == '__main__':

    wrench_info = read_csv("wind_table.csv")
    i = find_closest(wrench_info.angle, 0.56)
    print(wrench_info)
    print(wrench_info.angle[i])
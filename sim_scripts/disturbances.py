import random
import matplotlib.pyplot as plt


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


if __name__ == '__main__':
    current_velocity = IntegratedWhiteNoise(0, 0.514, 0.001)
    current_direction = IntegratedWhiteNoise(0, 360, 1)
    wind_velocity = IntegratedWhiteNoise(0, 7.716, 2)
    wind_direction = IntegratedWhiteNoise(0, 360, 7)

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

    plt.figure(figsize=(12, 6))
    plt.subplot(2, 1, 1)
    plt.plot(wv, label='Wind Speed')
    plt.ylabel('Speed (m/s)')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(wd, label='Wind Direction')
    plt.ylabel('Direction (degrees)')
    plt.xlabel('Time (s)')
    plt.legend()

    plt.show()
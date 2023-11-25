import random
import matplotlib.pyplot as plt


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


if __name__ == '__main__':
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

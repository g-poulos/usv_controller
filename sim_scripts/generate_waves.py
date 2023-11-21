import numpy as np
import matplotlib.pyplot as plt


def modified_spectrum(omega, Hs):
    g = 9.81
    A = 8.1 * (10 ** -3) * g ** 2
    B = 3.11 / (Hs ** 2)

    m0 = A / (4 * B)
    m1 = 0.306 * (A / (B ** (3 / 4)))
    m2 = (np.sqrt(np.pi) / 4) * (A / np.sqrt(B))
    Tz = 2 * np.pi * np.sqrt(m0 / m2)

    first_part = (4 * (np.pi ** 3) * Hs ** 2) / ((Tz ** 4) * (omega ** 5))
    second_part = np.exp((-16 * (np.pi ** 3)) / ((Tz ** 4) * (omega ** 4)))
    return first_part * second_part


def spectrum(omega, Hs):
    g = 9.81
    A = 8.1 * (10 ** -3) * g ** 2
    B = 3.11 / (Hs ** 2)
    print(np.sqrt((Hs * g) / 0.21))
    return A * (omega ** -5) * np.exp(-B * (omega ** -4))


def sim_waves(duration, dt, spectrum):
    num_points = int(duration / dt)
    time = np.arange(0, duration, dt)

    phases = np.random.rand(len(frequencies), num_points) * 2 * np.pi
    elevation = np.sum(
        np.sqrt(2 * spectrum[:, np.newaxis] * dt) *
        np.cos(frequencies[:, np.newaxis] * time + phases), axis=0)

    # s =
    # x_wave = np.sum(1025*L*L*0.6*np.cos(0)*)
    return time, elevation


if __name__ == '__main__':
    # Generate frequency array
    frequencies = np.linspace(0.1, 1.4, 1000)

    # Calculate the spectrum
    spectrum_values = spectrum(frequencies, 2)
    spectrum_values3 = spectrum(frequencies, 3)
    spectrum_values5 = spectrum(frequencies, 5)
    spectrum_values7 = spectrum(frequencies, 7)

    modified_spectrum_values = modified_spectrum(frequencies, 2)
    modified_spectrum_values3 = modified_spectrum(frequencies, 3)
    modified_spectrum_values5 = modified_spectrum(frequencies, 5)
    modified_spectrum_values7 = modified_spectrum(frequencies, 7)

    # Plot the spectrum
    fig, (ax1, ax2) = plt.subplots(1, 2, sharex=True, sharey=True)

    ax1.plot(frequencies, spectrum_values)
    ax1.plot(frequencies, spectrum_values3)
    ax1.plot(frequencies, spectrum_values5)
    ax1.plot(frequencies, spectrum_values7)
    ax1.set_title('Wave Spectrum')
    ax1.set_xlabel('Frequency (rad/s)')
    ax1.set_ylabel('Wave Energy Density')
    ax1.grid(True)

    ax2.plot(frequencies, modified_spectrum_values)
    ax2.plot(frequencies, modified_spectrum_values3)
    ax2.plot(frequencies, modified_spectrum_values5)
    ax2.plot(frequencies, modified_spectrum_values7)
    ax2.set_title('Modified Wave Spectrum')
    ax2.set_xlabel('Frequency (rad/s)')
    ax2.set_ylabel('Wave Energy Density')
    ax2.grid(True)

    fig.set_figwidth(15)
    plt.show()


    # Plot wave elevation
    spectrum_values = modified_spectrum(frequencies, 0.7)
    time, elevation = sim_waves(3600, 1, spectrum_values)
    plt.figure(figsize=(15, 6))
    plt.plot(time, elevation)
    plt.title('Simulated Waves')
    plt.xlabel('Time (s)')
    plt.ylabel('Wave Elevation (m)')
    plt.grid(True)
    plt.show()

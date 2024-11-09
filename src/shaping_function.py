import numpy as np
import matplotlib.pyplot as plt


def step_function(t, period=3):
    first_step = np.where((t % period) < 1, 1, -1)
    second_step = np.where(((t % period) >= 1) & ((t % period) < 2), -1, 0)
    return first_step + second_step


if __name__ == "__main__":
    t = np.linspace(0, 10, 1000)
    s_t = step_function(t)

    plt.plot(t, s_t)
    plt.xlabel("Time [t]")
    plt.ylabel("Function Value")
    plt.title("Step Function with Zero Interval")
    plt.grid(True)
    plt.show()
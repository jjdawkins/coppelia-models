from matplotlib import pyplot as plt
import numpy as np


def plot_vel(self):
    # plot the velocity values
    self.lines[0].set_data(self.t_hist, self.z_dot_hist[0, :])
    self.lines[1].set_data(self.t_hist, self.z_dot_d_hist[0, :])
    self.lines[2].set_data(self.t_hist, self.z_dot_hist[1, :])
    self.lines[3].set_data(self.t_hist, self.z_dot_d_hist[1, :])

    # update the plot
    self.ax[0].relim()
    self.ax[0].autoscale_view()
    self.ax[1].relim()
    self.ax[1].autoscale_view()

    plt.draw()


def update_histories(self):
    # update the HISTORIES
    self.z_dot_hist = np.append(
        self.z_dot_hist, np.reshape(self.z_dot, (3, 1)), axis=1)
    self.z_dot_d_hist = np.append(
        self.z_dot_d_hist, np.reshape(self.z_dot_d, (3, 1)), axis=1)
    self.t_hist = np.append(self.t_hist, self.t)

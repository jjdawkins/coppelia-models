from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
import numpy as np


def init_vel_plot(self):
    # make axes
    self.fig, self.ax = plt.subplots(2, 1)
    self.ax[0].set_title("Forward Velocity")
    self.ax[0].set_xlabel("Time (s)")
    self.ax[0].set_ylabel("Velocity (m/s)")
    self.ax[1].set_title("Yaw Rate")
    self.ax[1].set_xlabel("Time (s)")
    self.ax[1].set_ylabel("Yaw Rate (rad/s)")

    self.vel_lines = [Line2D([0], [0], color='blue', lw=2),
                  Line2D([0], [0], color='orange', lw=2, linestyle='--'),
                  Line2D([0], [0], color='blue', lw=2),
                  Line2D([0], [0], color='orange', lw=2, linestyle='--')]

    self.ax[0].add_line(self.vel_lines[0])
    self.ax[0].add_line(self.vel_lines[1])
    self.ax[1].add_line(self.vel_lines[2])
    self.ax[1].add_line(self.vel_lines[3])

    self.ax[0].legend(
        self.vel_lines, ['Measured', 'Desired'], loc='upper left')
    self.ax[1].legend(
        self.vel_lines, ['Measured', 'Desired'], loc='upper left')
    plt.draw()


def plot_vel(self):
    # plot the velocity values
    self.vel_lines[0].set_data(self.t_hist, self.z_dot_hist[0, :])
    self.vel_lines[1].set_data(self.t_hist, self.z_dot_d_hist[0, :])
    self.vel_lines[2].set_data(self.t_hist, self.z_dot_hist[1, :])
    self.vel_lines[3].set_data(self.t_hist, self.z_dot_d_hist[1, :])

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

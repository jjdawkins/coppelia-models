from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
import numpy as np


def init_vel_plot(self):
    # make axes
    self.fig_vel, self.ax_vel = plt.subplots(2, 1)
    self.ax_vel[0].set_title("Forward Velocity")
    self.ax_vel[0].set_xlabel("Time (s)")
    self.ax_vel[0].set_ylabel("Velocity (m/s)")
    self.ax_vel[1].set_title("Yaw Rate")
    self.ax_vel[1].set_xlabel("Time (s)")
    self.ax_vel[1].set_ylabel("Yaw Rate (rad/s)")
    # grid on
    self.ax_vel[0].grid()
    self.ax_vel[1].grid()

    self.vel_lines = [
        Line2D([0], [0], color="blue", lw=2),
        Line2D([0], [0], color="orange", lw=2, linestyle="--"),
        Line2D([0], [0], color="blue", lw=2),
        Line2D([0], [0], color="orange", lw=2, linestyle="--"),
    ]

    self.ax_vel[0].add_line(self.vel_lines[0])
    self.ax_vel[0].add_line(self.vel_lines[1])
    self.ax_vel[1].add_line(self.vel_lines[2])
    self.ax_vel[1].add_line(self.vel_lines[3])

    self.ax_vel[0].legend(self.vel_lines, ["Measured", "Desired"], loc="upper left")
    self.ax_vel[1].legend(self.vel_lines, ["Measured", "Desired"], loc="upper left")
    # set tight layout
    plt.tight_layout()
    self.fig_vel.canvas.draw()
    plt.pause(1e-4)


def init_error_plot(self):
    # make axes
    self.fig_error, self.ax_error = plt.subplots(2, 1)
    self.ax_error[0].set_title("Longitudinal Velocity Error")
    self.ax_error[0].set_xlabel("Time (s)")
    self.ax_error[0].set_ylabel("Velocity Error (m/s)")
    self.ax_error[1].set_title("Yaw Rate Error")
    self.ax_error[1].set_xlabel("Time (s)")
    self.ax_error[1].set_ylabel("Yaw Rate Error (rad/s)")
    # grid on
    self.ax_error[0].grid()
    self.ax_error[1].grid()

    # plot y = 0 line
    self.ax_error[0].axhline(0, color="black", lw=1)
    self.ax_error[1].axhline(0, color="black", lw=1)

    self.error_lines = [
        Line2D([0], [0], color="purple", lw=2),
        Line2D([0], [0], color="purple", lw=2),
    ]

    self.ax_error[0].add_line(self.error_lines[0])
    self.ax_error[1].add_line(self.error_lines[1])

    self.ax_error[0].legend(self.error_lines, ["Error"], loc="upper left")
    self.ax_error[1].legend(self.error_lines, ["Error"], loc="upper left")
    # set tight layout
    plt.tight_layout()
    self.fig_error.canvas.draw()
    plt.pause(1e-4)


def plot_vel(self):
    # plot the velocity values
    self.vel_lines[0].set_data(self.t_z_dot_hist, self.z_dot_hist[0, :])
    self.vel_lines[1].set_data(self.t_z_dot_d_hist, self.z_dot_d_hist[0, :])
    self.vel_lines[2].set_data(self.t_z_dot_hist, self.z_dot_hist[1, :])
    self.vel_lines[3].set_data(self.t_z_dot_d_hist, self.z_dot_d_hist[1, :])

    # update the plot
    self.ax_vel[0].relim()
    self.ax_vel[0].autoscale_view()
    self.ax_vel[1].relim()
    self.ax_vel[1].autoscale_view()

    self.fig_vel.canvas.draw()


def plot_error(self):

    # plot the error values
    self.error_lines[0].set_data(
        self.t_hist, self.z_dot_hist[0, :] - self.z_dot_d_hist[0, :]
    )
    self.error_lines[1].set_data(
        self.t_hist, self.z_dot_hist[1, :] - self.z_dot_d_hist[1, :]
    )

    # update the plot
    self.ax_error[0].relim()
    self.ax_error[0].autoscale_view()
    self.ax_error[1].relim()
    self.ax_error[1].autoscale_view()

    self.fig_error.canvas.draw()

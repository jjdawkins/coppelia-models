from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
import numpy as np


def init_input_plot(self):
    # make axes
    self.fig_input, self.ax_input = plt.subplots(2, 1)
    self.ax_input[0].set_title("Motor Input")
    self.ax_input[0].set_xlabel("Time (s)")
    self.ax_input[0].set_ylabel("Input (\omega)")
    self.ax_input[1].set_title("Steering Input")
    self.ax_input[1].set_xlabel("Time (s)")
    self.ax_input[1].set_ylabel("Input (\delta)")
    # grid on
    self.ax_input[0].grid()
    self.ax_input[1].grid()

    self.input_lines = [
        Line2D([0], [0], color="blue", lw=2),
        Line2D([0], [0], color="orange", lw=2),
    ]

    self.ax_input[0].add_line(self.input_lines[0])
    self.ax_input[1].add_line(self.input_lines[1])
    plt.tight_layout()
    self.fig_input.canvas.draw()
    plt.pause(1e-4)


def plot_input(self):
    # plot the input values
    self.input_lines[0].set_data(self.t_hist, self.u_hist[0, :])
    self.input_lines[1].set_data(self.t_hist, self.u_hist[1, :])

    # update the plot
    self.ax_input[0].relim()
    self.ax_input[0].autoscale_view()
    self.ax_input[1].relim()
    self.ax_input[1].autoscale_view()

    self.fig_input.canvas.draw()

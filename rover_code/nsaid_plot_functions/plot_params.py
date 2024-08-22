import numpy as np
import matplotlib.pyplot as plt


def init_param_plot(self):
    param_names = ["m", "Jz", "k", "c_rr", "c_af", "c_s", "c_d"]
    self.p = len(param_names)

    # make axes
    self.fig_param, self.ax_param = plt.subplots(self.p, 1, figsize=(6, 12))
    self.fig_param.suptitle("Parameter Estimates")

    # label each axis
    for i in range(self.p):
        self.ax_param[i].set_title(param_names[i])
        self.ax_param[i].set_xlabel("Time (s)")
        self.ax_param[i].set_ylabel("Value")
        # grid on
        self.ax_param[i].grid()

    # create list to hold the lines
    self.param_lines = []

    # create lines for each parameter & add to plot
    for i in range(len(param_names)):
        self.param_lines.append(
            [
                plt.Line2D([0], [0], color="blue", lw=2),
            ]
        )
        self.ax_param[i].add_line(self.param_lines[i][0])
        self.ax_param[i].legend(self.param_lines[i], [param_names[i]], loc="upper left")
    # set tight layout
    plt.tight_layout()
    self.fig_param.canvas.draw()
    plt.pause(1e-4)


def plot_params(self):
    # update the parameter lines using the history
    for i in range(self.p):
        self.param_lines[i][0].set_data(self.t_hist, self.theta_h_hist[i, :])
        # autscale the y-axis
        self.ax_param[i].relim()
        self.ax_param[i].autoscale_view()
        # dont show scientific notation
        self.ax_param[i].get_yaxis().get_major_formatter().set_useOffset(False)
    # redraw the plot
    self.fig_param.canvas.draw()

from nsaid_module import nsaidEstimation
import rclpy
import matplotlib.pyplot as plt
import time


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the parameterEstimation class
    nsaid_est = nsaidEstimation()

    doPlot = True
    # turn on interactive plotting
    plt.ion()

    t0 = time.time()
    # for plotting
    i = 0
    plot_num = 0
    plot_funcs = [nsaid_est.plot_vel, nsaid_est.plot_error, nsaid_est.plot_params]

    while rclpy.ok():
        i += 1

        # update history data for plotting
        nsaid_est.update_histories()

        # Update the plots
        if doPlot and i % 500 == 0:
            plot_funcs[plot_num]()
            plot_num = (plot_num + 1) % len(plot_funcs)
            plt.pause(1e-10)
        elif not doPlot and i % 10 == 0:
            # print the parameter estimates
            for i in range(nsaid_est.p):
                print(f"{nsaid_est.theta_h[i]:.2f}", end=" ")
            print()

        # Process any pending events and callbacks
        rclpy.spin_once(nsaid_est)

        # Sleep for a short duration to control the loop rate
        while time.time() - t0 < 0.01:
            pass
        t0 = time.time()

    # Clean up and destroy the node
    nsaidEstimation.destroy_node()

    # Shut down the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == "__main__":
    main()

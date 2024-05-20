from nsaid_module import nsaidEstimation
import rclpy
import matplotlib.pyplot as plt
import time


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the parameterEstimation class
    nsaid_est = nsaidEstimation()

    # do plot setup
    plt.ion()

    i = 0
    t0 = time.time()

    while rclpy.ok():
        i += 1

        # Update the plots
        if i % 5 == 0:
            nsaid_est.plot_vel()
            plt.pause(1e-10)

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


if __name__ == '__main__':
    main()

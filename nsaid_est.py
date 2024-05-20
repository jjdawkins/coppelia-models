from nsaid_module import nsaidEstimation
import rclpy
import matplotlib.pyplot as plt
import time


def ros_spin(node):
    rclpy.spin(node)


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
        t1 = time.time()
        delta_t = t1 - t0
        t0 = t1
        print(f"freq = {1/delta_t:3.0f}", end="\r")

        # Update the plots
        if i % 5 == 0:
            nsaid_est.plot_vel()
            plt.pause(1e-7)

        # Process any pending events and callbacks
        rclpy.spin_once(nsaid_est)

        # Sleep for a short duration to control the loop rate
        time.sleep(1e-7)

    # Clean up and destroy the node
    nsaidEstimation.destroy_node()

    # Print a message indicating that the run has finished
    print("Run Finished")

    # Shut down the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()

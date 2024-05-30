from nsaid_module import nsaidEstimation
import rclpy
import matplotlib.pyplot as plt
import time


class nsaidPlot(rclpy.node.Node):
    def __init__(self):
        super().__init__("nsaid_plot")
        print("nsaid_plot node started")

        def run_loop(self):
            pass


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    nsaid_plot = nsaidPlot()

    t0 = time.time()
    i = 0

    while rclpy.ok():
        # Process any pending events and callbacks
        rclpy.spin_once(nsaid_plot)

        # Sleep for a short duration to control the loop rate
        while time.time() - t0 < 0.01:
            pass
        t0 = time.time()

    # Clean up and destroy the node
    nsaidPlot.destroy_node()

    # Shut down the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == "__main__":
    main()

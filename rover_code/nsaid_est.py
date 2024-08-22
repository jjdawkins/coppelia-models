from nsaid_module import nsaidEstimation
import rclpy
import matplotlib.pyplot as plt
import time


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the parameterEstimation class
    nsaid_est = nsaidEstimation()

    t0 = time.time()
    i = 0

    try:
        while rclpy.ok():
            i += 1
            # if i % 10 == 0:
            #     # print the parameter estimates
            #     for i in range(nsaid_est.p):
            #         print(f"{nsaid_est.theta_h[i]:.2f}", end=" ")
            #     print()

            # Process any pending events and callbacks
            rclpy.spin_once(nsaid_est)

    except KeyboardInterrupt:
        print("Keyboard Interrupt")

    finally: 
        # Clean up and destroy the node
        nsaid_est.destroy_node()

        # Shut down the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == "__main__":
    main()

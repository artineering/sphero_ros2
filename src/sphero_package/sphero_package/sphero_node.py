import rclpy
from rclpy.node import Node
import signal
import sys

from spherov2 import scanner
from spherov2.sphero_edu import SpheroEduAPI
from spherov2.types import Color

from sphero_package.spherov2_collision_patch import apply_collision_patch

# Apply collision detection patch for 16-byte collision responses
apply_collision_patch()


class SpheroNode(Node):
    def __init__(self, robot, api):
        super().__init__('sphero_node')
        self.robot = robot
        self.api = api
        self.get_logger().info('Sphero node initialized')

    def cleanup(self):
        """Clean up resources before shutdown"""
        self.get_logger().info('Cleaning up Sphero node...')
        try:
            # Turn off the LED before disconnecting
            with self.api:
                self.api.set_main_led(Color(r=0, g=0, b=0))
                self.get_logger().info('LED turned off')
        except Exception as e:
            self.get_logger().error(f'Error turning off LED: {e}')

        self.get_logger().info('Sphero cleanup complete')


def main(args=None):
    rclpy.init(args=args)
    node = None
    api = None
    shutdown_requested = False

    def signal_handler(_sig, _frame):
        nonlocal shutdown_requested
        print("\nKeyboard interrupt detected. Shutting down...")
        shutdown_requested = True

    # Register signal handler for SIGINT (Ctrl+C)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        robot = scanner.find_toy(toy_name="SB-3660")
        with SpheroEduAPI(toy=robot) as api:
            print("Connected to SB-3660")

            # Set LED to red
            api.set_main_led(Color(r=255, g=0, b=0))

            # Create the node
            node = SpheroNode(robot, api)

            # Spin until shutdown requested
            while rclpy.ok() and not shutdown_requested:
                rclpy.spin_once(node, timeout_sec=0.1)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # Clean up the node before exiting context
        if node:
            node.cleanup()
            node.destroy_node()

        # Shutdown ROS 2
        if rclpy.ok():
            rclpy.shutdown()

        print("Goodbye!")

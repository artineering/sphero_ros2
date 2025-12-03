#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sphero Instance Task Controller Node.

This node accepts high-level tasks from a dedicated topic and executes them
using the TaskExecutor class. Designed for multi-robot setups with namespaced topics.

All topics are namespaced under 'sphero/<sphero_name>/' to allow multiple
instances to run simultaneously without cross-talk.
"""

import json
import time
import signal

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String

from sphero_instance_controller.core.sphero.task import TaskDescriptor, TaskStatus
from sphero_instance_controller.core.sphero.topic_task_executor import TopicTaskExecutor

# Note: Task controller does NOT import scanner, SpheroEduAPI, or Sphero
# It only communicates through ROS topics


class SpheroInstanceTaskController(Node):
    """
    ROS2 node for high-level task control of a single Sphero instance.

    Uses namespaced topics for multi-robot support.
    All topics are prefixed with 'sphero/<sphero_name>/'
    """

    def __init__(self, sphero_name: str):
        """
        Initialize the task controller node.

        Args:
            sphero_name: Name of the Sphero (used for topic namespacing)
        """
        # Sanitize name: replace hyphens with underscores (ROS2 naming rules)
        name_safe = sphero_name.replace("-", "_")
        # Create unique node name with sphero name suffix
        node_name = f'sphero_task_controller_{name_safe}'
        super().__init__(node_name)

        self.sphero_name = sphero_name
        # Sanitize topic name: replace hyphens with underscores (ROS2 topic naming rules)
        topic_name_safe = name_safe
        self.topic_prefix = f'sphero/{topic_name_safe}'

        # Note: Task controller does NOT connect to hardware
        # It only communicates through ROS topics with the device controller

        # State tracking
        self.current_state = {}
        self.current_position = {'x': 0.0, 'y': 0.0}
        self.current_heading = 0

        # Callback group for reentrant callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Create publishers first (needed for command_publisher callback)
        self._create_publishers()

        # Create subscribers
        self._create_subscribers()

        # Initialize task executor (topic-based, no direct Sphero access)
        self.task_executor = TopicTaskExecutor(
            command_publisher=self.publish_command,
            position_callback=self.get_current_position,
            heading_callback=self.get_current_heading
        )

        # Create timer for task execution (10 Hz)
        self.task_timer = self.create_timer(
            0.01,
            self.task_execution_loop,
            callback_group=self.callback_group
        )

        # Log initialization
        self._log_initialization()

        # Note: LED control is done through ROS topics, not direct hardware access

    def _create_subscribers(self):
        """Create all ROS subscribers."""
        # Task command subscriber
        self.task_sub = self.create_subscription(
            String,
            f'{self.topic_prefix}/task',
            self.task_callback,
            10,
            callback_group=self.callback_group
        )

        # State feedback subscriber
        self.state_sub = self.create_subscription(
            String,
            f'{self.topic_prefix}/state',
            self.state_callback,
            10,
            callback_group=self.callback_group
        )

        # Reset aim subscriber
        self.reset_aim_sub = self.create_subscription(
            String,
            f'{self.topic_prefix}/reset_aim',
            self.reset_aim_callback,
            10,
            callback_group=self.callback_group
        )

    def _create_publishers(self):
        """Create all ROS publishers."""
        # Task status publisher
        self.task_status_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/task/status',
            10
        )

        # Command publishers for task execution
        self.raw_motor_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/raw_motor',
            10
        )

        self.motion_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/roll',
            10
        )

        self.led_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/led',
            10
        )

        self.heading_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/heading',
            10
        )

        self.speed_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/speed',
            10
        )

        self.spin_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/spin',
            10
        )

        self.matrix_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/matrix',
            10
        )

        self.stop_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/stop',
            10
        )

        self.stabilization_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/stabilization',
            10
        )

        self.collision_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/collision',
            10
        )

    def _log_initialization(self):
        """Log initialization information."""
        self.get_logger().info('='*70)
        self.get_logger().info('🎯 TASK CONTROLLER ACTIVATED')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Sphero Name: {self.sphero_name}')
        self.get_logger().info(f'Topic Prefix: {self.topic_prefix}')
        self.get_logger().info('Subscribed Topics:')
        self.get_logger().info(f'  - {self.topic_prefix}/task')
        self.get_logger().info(f'  - {self.topic_prefix}/state')
        self.get_logger().info(f'  - {self.topic_prefix}/reset_aim')
        self.get_logger().info('Publishing Topics:')
        self.get_logger().info(f'  - {self.topic_prefix}/task/status')
        self.get_logger().info(f'  - {self.topic_prefix}/raw_motor')
        self.get_logger().info(f'  - {self.topic_prefix}/roll')
        self.get_logger().info(f'  - {self.topic_prefix}/led')
        self.get_logger().info(f'  - {self.topic_prefix}/heading')
        self.get_logger().info(f'  - {self.topic_prefix}/speed')
        self.get_logger().info(f'  - {self.topic_prefix}/spin')
        self.get_logger().info(f'  - {self.topic_prefix}/matrix')
        self.get_logger().info(f'  - {self.topic_prefix}/stop')
        self.get_logger().info(f'  - {self.topic_prefix}/stabilization')
        self.get_logger().info(f'  - {self.topic_prefix}/collision')
        self.get_logger().info('='*70)
        self.get_logger().info('✅ Task Controller READY (Topic-Based Executor)')
        self.get_logger().info('='*70)

    def get_current_position(self):
        """Get current position for task executor."""
        return self.current_position.copy()

    def get_current_heading(self):
        """Get current heading for task executor."""
        return self.current_heading

    def publish_command(self, topic_name: str, params: dict):
        """
        Publish a command to the appropriate ROS topic.

        Args:
            topic_name: The command topic name (e.g., 'raw_motor', 'led', 'motion')
            params: Dictionary of parameters for the command
        """
        msg = String()
        msg.data = json.dumps(params)

        # Route to appropriate publisher based on topic name
        if topic_name == 'raw_motor':
            self.raw_motor_pub.publish(msg)
            self.get_logger().debug(f'Published raw_motor: {params}')
        elif topic_name == 'motion':
            # Motion commands can be roll or stop
            action = params.get('action', 'roll')
            if action == 'stop':
                self.stop_pub.publish(msg)
                self.get_logger().debug('Published stop command')
            else:
                self.motion_pub.publish(msg)
                self.get_logger().debug(f'Published motion: {params}')
        elif topic_name == 'led':
            self.led_pub.publish(msg)
            self.get_logger().debug(f'Published LED: {params}')
        elif topic_name == 'heading':
            self.heading_pub.publish(msg)
            self.get_logger().debug(f'Published heading: {params}')
        elif topic_name == 'speed':
            self.speed_pub.publish(msg)
            self.get_logger().debug(f'Published speed: {params}')
        elif topic_name == 'spin':
            self.spin_pub.publish(msg)
            self.get_logger().debug(f'Published spin: {params}')
        elif topic_name == 'matrix':
            self.matrix_pub.publish(msg)
            self.get_logger().debug(f'Published matrix: {params}')
        elif topic_name == 'stabilization':
            self.stabilization_pub.publish(msg)
            self.get_logger().debug(f'Published stabilization: {params}')
        elif topic_name == 'collision':
            self.collision_pub.publish(msg)
            self.get_logger().debug(f'Published collision: {params}')
        else:
            self.get_logger().warning(f'Unknown command topic: {topic_name}')

    # ===== Callbacks =====

    def task_callback(self, msg: String):
        """
        Handle incoming task messages.

        Expected JSON format:
        {
            "task_id": "unique_id",  // optional
            "task_type": "move_to|patrol|circle|...",
            "parameters": {
                // Task-specific parameters
            }
        }
        """
        try:
            task_data = json.loads(msg.data)

            # Validate required fields
            if 'task_type' not in task_data:
                self.get_logger().error('Task missing required field: task_type')
                return

            # Generate task ID if not provided
            if 'task_id' not in task_data:
                task_data['task_id'] = f"task_{int(time.time() * 1000)}"

            # Create task descriptor
            task = TaskDescriptor(
                task_id=task_data['task_id'],
                task_type=task_data['task_type'],
                parameters=task_data.get('parameters', {})
            )

            # Add to executor queue
            self.task_executor.add_task(task)

            self.get_logger().info(
                f'Added task {task.task_id} ({task.task_type}) to queue. '
                f'Queue length: {len(self.task_executor.task_queue)}'
            )
            self.get_logger().info(f'Task parameters: {json.dumps(task.parameters, indent=2)}')

            # Publish status
            self.publish_task_status(task)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in task message: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing task: {e}')

    def state_callback(self, msg: String):
        """Handle Sphero state updates."""
        try:
            self.current_state = json.loads(msg.data)

            # Update position
            if 'position' in self.current_state:
                self.current_position = self.current_state['position'].copy()

            # Update heading
            if 'motion' in self.current_state and 'heading' in self.current_state['motion']:
                self.current_heading = self.current_state['motion']['heading']

        except json.JSONDecodeError:
            pass

    def reset_aim_callback(self, msg: String):
        """Reset position and heading to origin."""
        self.current_position = {'x': 0.0, 'y': 0.0}
        self.current_heading = 0
        self.get_logger().info('Task controller reset to origin: heading=0°, position=(0, 0)')

    # ===== Task Execution =====

    def task_execution_loop(self):
        """Main task execution loop - called periodically."""
        # Get previous task state
        previous_task = self.task_executor.current_task

        # Process tasks
        current_task = self.task_executor.process_tasks()

        # Check if task changed
        if previous_task != current_task:
            # Task completed or new task started
            if previous_task is not None:
                # Previous task finished
                self.publish_task_status(previous_task)

                duration = previous_task.completed_at - previous_task.started_at
                self.get_logger().info(
                    f'Task {previous_task.task_id} {previous_task.status.value} in {duration:.2f}s'
                )

            if current_task is not None:
                # New task started
                self.publish_task_status(current_task)

                # Update position from state before starting
                if 'position' in self.current_state:
                    self.current_position = self.current_state['position'].copy()
                    self.get_logger().info(
                        f'Starting task {current_task.task_id} at position: '
                        f'x={self.current_position["x"]:.2f}, '
                        f'y={self.current_position["y"]:.2f}'
                    )
                else:
                    self.get_logger().info(f'Starting task {current_task.task_id}')

    def publish_task_status(self, task: TaskDescriptor):
        """Publish task status update."""
        # Include queue information
        status_dict = task.to_dict()
        status_dict['queue_length'] = len(self.task_executor.task_queue)
        status_dict['has_current_task'] = self.task_executor.current_task is not None
        status_dict['total_pending'] = (
            len(self.task_executor.task_queue) +
            (1 if self.task_executor.current_task is not None else 0)
        )

        msg = String()
        msg.data = json.dumps(status_dict)
        self.task_status_pub.publish(msg)

    def cleanup(self):
        """Clean up resources before shutdown."""
        self.get_logger().info('Cleaning up Sphero instance task controller...')
        # Task controller has no hardware to clean up
        # Hardware cleanup is handled by the device controller
        self.get_logger().info('Task controller cleanup complete')


def main(args=None):
    """Main entry point for the Sphero instance task controller node."""
    rclpy.init(args=args)
    node = None
    temp_node = None
    shutdown_requested = False

    def signal_handler(_sig, _frame):
        nonlocal shutdown_requested
        print("\nKeyboard interrupt detected. Shutting down...")
        shutdown_requested = True

    # Register signal handler for SIGINT (Ctrl+C)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        # Create a temporary node to read the sphero_name parameter
        temp_node = rclpy.create_node('temp_param_node')
        temp_node.declare_parameter('sphero_name', '')  # No default - REQUIRED parameter
        sphero_name = temp_node.get_parameter('sphero_name').value

        if not sphero_name:
            raise ValueError(
                "The 'sphero_name' parameter is required but was not provided. "
                "Please launch with: ros2 run sphero_instance_controller sphero_instance_task_controller_node.py "
                "--ros-args -p sphero_name:=<YOUR_SPHERO_NAME>"
            )

        print(f"Sphero name from parameter: {sphero_name}")

        # Destroy temporary node before creating controller node
        temp_node.destroy_node()
        temp_node = None

        # Create the task controller node (no hardware connection needed)
        print(f"Initializing task controller for {sphero_name}...")
        node = SpheroInstanceTaskController(sphero_name)
        print(f"Task controller initialized for {sphero_name}")

        # Spin until shutdown requested
        while rclpy.ok() and not shutdown_requested:
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except rclpy.executors.ExternalShutdownException:
                # Expected during shutdown - ignore
                break

    except KeyboardInterrupt:
        print("\nShutting down...")
    except rclpy.executors.ExternalShutdownException:
        # Expected during external shutdown - ignore
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Clean up temporary node if it exists
        if temp_node:
            temp_node.destroy_node()

        # Clean up the controller node
        if node:
            node.cleanup()
            node.destroy_node()

        # Shutdown ROS 2
        if rclpy.ok():
            rclpy.shutdown()

        print("Goodbye!")


if __name__ == '__main__':
    main()

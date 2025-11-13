#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sphero Instance State Machine Controller Node.

This node provides a dynamically configurable state machine for a single Sphero instance.
Designed for multi-robot setups with namespaced topics.

All topics are namespaced under 'sphero/<sphero_name>/' to allow multiple
instances to run simultaneously without cross-talk.
"""

import json
import importlib
import signal

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from sphero_instance_controller.msg import SpheroSensor

from sphero_instance_controller.core.sphero import StateMachine


class SpheroInstanceStateMachineController(Node):
    """
    ROS2 node for state machine control of a single Sphero instance.

    Uses namespaced topics for multi-robot support.
    All topics are prefixed with 'sphero/<sphero_name>/'
    """

    def __init__(self, sphero_name: str):
        """
        Initialize the state machine controller node.

        Args:
            sphero_name: Name of the Sphero (used for topic namespacing)
        """
        super().__init__('sphero_instance_statemachine_controller_node')

        self.sphero_name = sphero_name
        self.topic_prefix = f'sphero/{sphero_name}'

        # Initialize state machine core class
        self.state_machine = StateMachine(
            logger=self._log_info,
            topic_subscribe_callback=self.subscribe_to_topic,
            topic_unsubscribe_callback=self.unsubscribe_from_topic
        )

        # Dynamic topic subscriptions for state machine conditions
        self.topic_subscriptions = {}
        self.topic_message_types = {}

        # Callback group for reentrant callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Create subscribers
        self._create_subscribers()

        # Create publishers
        self._create_publishers()

        # Create timer for state machine updates (10 Hz)
        self.update_timer = self.create_timer(
            0.1,
            self.update_callback,
            callback_group=self.callback_group
        )

        # Status publishing timer (1 Hz)
        self.status_timer = self.create_timer(
            1.0,
            self.publish_status,
            callback_group=self.callback_group
        )

        # Log initialization
        self._log_initialization()

    def _create_subscribers(self):
        """Create all ROS subscribers."""
        # Configuration subscriber
        self.config_sub = self.create_subscription(
            String,
            f'{self.topic_prefix}/state_machine/config',
            self.config_callback,
            10,
            callback_group=self.callback_group
        )

        # Sensor data subscriber (for condition evaluation)
        self.sensor_sub = self.create_subscription(
            SpheroSensor,
            f'{self.topic_prefix}/sensors',
            self.sensor_callback,
            10,
            callback_group=self.callback_group
        )

    def _create_publishers(self):
        """Create all ROS publishers."""
        # Status publisher
        self.status_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/state_machine/status',
            10
        )

        # Event publisher
        self.event_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/state_machine/events',
            10
        )

        # Task command publisher
        self.task_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/task',
            10
        )

    def _log_initialization(self):
        """Log initialization information."""
        self.get_logger().info(f'Sphero Instance State Machine Controller initialized for {self.sphero_name}')
        self.get_logger().info(f'  - Topic prefix: {self.topic_prefix}')
        self.get_logger().info(f'  - Listening for config on {self.topic_prefix}/state_machine/config')
        self.get_logger().info(f'  - Publishing status to {self.topic_prefix}/state_machine/status')
        self.get_logger().info(f'  - Publishing events to {self.topic_prefix}/state_machine/events')
        self.get_logger().info(f'  - Publishing tasks to {self.topic_prefix}/task')

    def _log_info(self, message: str):
        """Logger function for StateMachine class."""
        self.get_logger().info(message)

    # ===== Callbacks =====

    def config_callback(self, msg: String):
        """
        Handle incoming state machine configuration.

        Expected JSON format:
        {
            "name": "my_state_machine",
            "initial_state": "idle",
            "states": [
                {
                    "name": "idle",
                    "description": "Waiting state",
                    "entry_condition": {
                        "type": "always",
                        "params": {}
                    },
                    "tasks": [
                        {
                            "task_type": "set_led",
                            "parameters": {"red": 0, "green": 0, "blue": 255}
                        }
                    ]
                },
                ...
            ],
            "transitions": [
                {
                    "source": "idle",
                    "destination": "moving",
                    "trigger": "timer_based",
                    "condition": {
                        "type": "timer",
                        "duration": 5.0
                    }
                },
                ...
            ]
        }
        """
        try:
            config = json.loads(msg.data)
            self.get_logger().info(f'Received state machine configuration: {config.get("name", "unnamed")}')

            # Configure the state machine
            if self.state_machine.configure(config):
                # Publish configuration success event
                self.publish_event('configuration_loaded', {
                    'name': config.get('name', 'unnamed'),
                    'num_states': len(self.state_machine.states),
                    'num_transitions': len(self.state_machine.transitions)
                })

                # Execute initial state tasks
                self.execute_current_state_tasks()
            else:
                self.get_logger().error('Failed to configure state machine')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse configuration JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing configuration: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def sensor_callback(self, msg: SpheroSensor):
        """
        Handle incoming Sphero sensor data for condition evaluation.

        Converts SpheroSensor message to a dictionary for easy condition checking.
        """
        try:
            # Convert SpheroSensor message to dictionary for condition evaluation
            sensor_data = {
                'pitch': msg.pitch,
                'roll': msg.roll,
                'yaw': msg.yaw,
                'accel_x': msg.accel_x,
                'accel_y': msg.accel_y,
                'accel_z': msg.accel_z,
                'gyro_x': msg.gyro_x,
                'gyro_y': msg.gyro_y,
                'gyro_z': msg.gyro_z,
                'x': msg.x,
                'y': msg.y,
                'velocity_x': msg.velocity_x,
                'velocity_y': msg.velocity_y,
                'battery_percentage': msg.battery_percentage
            }

            # Update state machine with sensor data
            self.state_machine.update_sensor_data(sensor_data)

        except Exception as e:
            self.get_logger().error(f'Failed to process sensor data: {e}')

    # ===== Dynamic Topic Subscriptions =====

    def subscribe_to_topic(self, topic_name: str, msg_type: str, field_path: str = None):
        """
        Dynamically subscribe to a ROS topic for transition monitoring.

        Args:
            topic_name: The topic to subscribe to (will be namespaced)
            msg_type: Message type as string (e.g., 'std_msgs/String')
            field_path: Optional dot-notation path to extract specific field
        """
        # Namespace the topic
        namespaced_topic = f'{self.topic_prefix}/{topic_name.lstrip("/")}'

        if namespaced_topic in self.topic_subscriptions:
            self.get_logger().info(f'Already subscribed to topic: {namespaced_topic}')
            return

        try:
            # Parse message type string (e.g., 'std_msgs/String' -> std_msgs.msg.String)
            parts = msg_type.split('/')
            if len(parts) != 2:
                self.get_logger().error(f'Invalid message type format: {msg_type}. Expected "package/Type"')
                return

            package_name, type_name = parts
            module_path = f'{package_name}.msg'

            # Import the message type
            try:
                msg_module = importlib.import_module(module_path)
                msg_class = getattr(msg_module, type_name)
            except (ImportError, AttributeError) as e:
                self.get_logger().error(f'Failed to import message type {msg_type}: {e}')
                return

            # Store message type information
            self.topic_message_types[namespaced_topic] = {
                'class': msg_class,
                'field_path': field_path
            }

            # Create callback that stores the received message
            def topic_callback(msg):
                # Extract specific field if specified
                if field_path:
                    try:
                        value = msg
                        for field in field_path.split('.'):
                            value = getattr(value, field)
                        self.state_machine.update_topic_value(namespaced_topic, value)
                    except AttributeError as e:
                        self.get_logger().error(f'Failed to extract field {field_path} from {namespaced_topic}: {e}')
                        self.state_machine.update_topic_value(namespaced_topic, msg)
                else:
                    # Store the entire message
                    self.state_machine.update_topic_value(namespaced_topic, msg)

                self.get_logger().debug(f'Received message on {namespaced_topic}')

            # Create subscription with default QoS
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

            subscription = self.create_subscription(
                msg_class,
                namespaced_topic,
                topic_callback,
                qos,
                callback_group=self.callback_group
            )

            self.topic_subscriptions[namespaced_topic] = subscription
            self.get_logger().info(f'Subscribed to topic: {namespaced_topic} (type: {msg_type})')

        except Exception as e:
            self.get_logger().error(f'Failed to subscribe to topic {namespaced_topic}: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def unsubscribe_from_topic(self, topic_name: str):
        """
        Unsubscribe from a dynamically created topic subscription.

        Args:
            topic_name: The topic to unsubscribe from
        """
        # Namespace the topic
        namespaced_topic = f'{self.topic_prefix}/{topic_name.lstrip("/")}'

        if namespaced_topic in self.topic_subscriptions:
            self.destroy_subscription(self.topic_subscriptions[namespaced_topic])
            del self.topic_subscriptions[namespaced_topic]
            if namespaced_topic in self.topic_message_types:
                del self.topic_message_types[namespaced_topic]
            self.get_logger().info(f'Unsubscribed from topic: {namespaced_topic}')

    # ===== State Machine Processing =====

    def update_callback(self):
        """Periodic state machine update callback."""
        result = self.state_machine.process()

        if result is None:
            return

        # Handle events
        for event in result.get('events', []):
            event_type = event.get('type')

            if event_type == 'state_timeout':
                self.get_logger().warning(
                    f'State {event["state"]} timed out after {event["elapsed"]:.1f}s'
                )
                self.publish_event('state_timeout', event)

            elif event_type == 'state_transition':
                self.get_logger().info(
                    f'Transitioned from {event["from"]} to {event["to"]}'
                )
                self.publish_event('state_transition', event)

                # Execute new state's tasks
                self.execute_current_state_tasks()

    def execute_current_state_tasks(self):
        """Execute tasks for the current state."""
        tasks = self.state_machine.get_current_state_tasks()

        if not tasks:
            self.get_logger().info('No tasks to execute for current state')
            self.state_machine.mark_tasks_completed()
            return

        current_state = self.state_machine.current_state
        self.get_logger().info(f'Executing {len(tasks)} task(s) for state {current_state}')

        # Execute each task in the array
        for idx, task in enumerate(tasks):
            # Convert old format (type/params) to new format (task_type/parameters)
            task_type = task.get('task_type') or task.get('type', 'none')
            task_params = task.get('parameters') or task.get('params', {})

            self.get_logger().info(f'  Task {idx + 1}/{len(tasks)}: {task_type}')

            # Publish task command in TaskExecutor format
            task_command = {
                'task_id': f'sm_{current_state}_{idx}',
                'task_type': task_type,
                'parameters': task_params
            }

            msg = String()
            msg.data = json.dumps(task_command)
            self.task_pub.publish(msg)

            # Publish task execution event
            self.publish_event('task_executed', {
                'state': current_state,
                'task_type': task_type,
                'params': task_params,
                'task_index': idx,
                'total_tasks': len(tasks)
            })

        self.state_machine.mark_tasks_completed()

    def publish_status(self):
        """Publish current state machine status."""
        status = self.state_machine.get_status()

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def publish_event(self, event_type: str, data: dict):
        """
        Publish a state machine event.

        Args:
            event_type: Type of event
            data: Event data
        """
        event = {
            'event_type': event_type,
            'data': data,
            'timestamp': self.state_machine.get_status()['timestamp']
        }

        msg = String()
        msg.data = json.dumps(event)
        self.event_pub.publish(msg)

        self.get_logger().info(f'Event: {event_type}')


def main(args=None):
    """Main entry point for the Sphero instance state machine controller node."""
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
                "Please launch with: ros2 run sphero_instance_controller sphero_instance_statemachine_controller_node.py "
                "--ros-args -p sphero_name:=<YOUR_SPHERO_NAME>"
            )

        print(f"Sphero name from parameter: {sphero_name}")

        # Destroy temporary node before creating controller node
        temp_node.destroy_node()
        temp_node = None

        # Create the node
        node = SpheroInstanceStateMachineController(sphero_name)

        # Spin until shutdown requested
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        print("\nShutting down...")
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
            node.destroy_node()

        # Shutdown ROS 2
        if rclpy.ok():
            rclpy.shutdown()

        print("Goodbye!")


if __name__ == '__main__':
    main()

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
import time
import importlib
import signal
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from sphero_instance_controller.msg import SpheroSensor

from sphero_instance_controller.core.sphero import StateMachine


# SM lanes MIRROR the task-executor actuator lanes (drive/led/matrix/config): an
# SM's ultimate effect is the executor lane its fired tasks land on, so two
# concurrent SMs must occupy different executor domains to run without fighting
# downstream. `default` is a reserved lane for configs that omit `lane`, so the
# pre-lane single-SM publishers (e.g. the websocket server / webapp) keep working
# unchanged. Each lane holds its own StateMachine instance and is ticked
# independently.
SM_ACTUATOR_LANES = ('drive', 'led', 'matrix', 'config')
DEFAULT_SM_LANE = 'default'
ALL_SM_LANES = SM_ACTUATOR_LANES + (DEFAULT_SM_LANE,)


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
        # Sanitize name: replace hyphens with underscores (ROS2 naming rules)
        name_safe = sphero_name.replace("-", "_")
        # Create unique node name with sphero name suffix
        node_name = f'sphero_statemachine_controller_{name_safe}'
        super().__init__(node_name)

        self.sphero_name = sphero_name
        # Sanitize topic name: replace hyphens with underscores (ROS2 topic naming rules)
        topic_name_safe = name_safe
        self.topic_prefix = f'sphero/{topic_name_safe}'

        # One StateMachine per lane, ticked independently. A Similarity-BLINK SM
        # (led/matrix lane) and a Proximity-cadence SM (drive lane) coexist on
        # one unit without contending for a single slot. Every lane shares the
        # node's dynamic subscribe/unsubscribe callbacks; topic values are
        # broadcast to every lane's SM (values are keyed by topic name, and each
        # SM only reads the topics its own states reference).
        self.state_machines: Dict[str, StateMachine] = {
            lane: StateMachine(
                logger=self._log_info,
                topic_subscribe_callback=self.subscribe_to_topic,
                topic_unsubscribe_callback=self.unsubscribe_from_topic,
            )
            for lane in ALL_SM_LANES
        }

        # Per-lane cache of the previously-active path so we can compute the
        # entered branch (newly-active levels) on each state_transition event.
        self._previous_paths: Dict[str, list] = {lane: [] for lane in ALL_SM_LANES}

        # Dynamic topic subscriptions for state machine conditions. Refcounted at
        # the node level so two lanes referencing the same topic share one ROS
        # subscription and one lane unsubscribing does not drop it for the other.
        self.topic_subscriptions = {}
        self.topic_message_types = {}
        self.topic_refcounts: Dict[str, int] = {}

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

        # Runtime control subscriber (pause / resume / clear)
        self.control_sub = self.create_subscription(
            String,
            f'{self.topic_prefix}/state_machine/control',
            self.control_callback,
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
        self.get_logger().info('='*70)
        self.get_logger().info('🔄 STATE MACHINE CONTROLLER ACTIVATED')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Sphero Name: {self.sphero_name}')
        self.get_logger().info(f'Topic Prefix: {self.topic_prefix}')
        self.get_logger().info('Subscribed Topics:')
        self.get_logger().info(f'  - {self.topic_prefix}/state_machine/config')
        self.get_logger().info(f'  - {self.topic_prefix}/state_machine/control')
        self.get_logger().info(f'  - {self.topic_prefix}/sensors')
        self.get_logger().info(f'  - Dynamic collision topics (when configured)')
        self.get_logger().info('Publishing Topics:')
        self.get_logger().info(f'  - {self.topic_prefix}/state_machine/status')
        self.get_logger().info(f'  - {self.topic_prefix}/state_machine/events')
        self.get_logger().info(f'  - {self.topic_prefix}/task')
        self.get_logger().info('='*70)
        self.get_logger().info('✅ State Machine Controller READY')
        self.get_logger().info('='*70)

    def _log_info(self, message: str):
        """Logger function for StateMachine class."""
        self.get_logger().info(message)

    # ===== Callbacks =====

    def config_callback(self, msg: String):
        """
        Handle incoming state machine configuration.

        Each state owns its own ``exits[]`` list. Every exit pairs a
        ``condition`` (always | timer | topic_value | topic_message) with a
        ``destination`` state. A state with no ``exits`` is a leaf state.
        There is no top-level ``transitions`` block.

        Expected JSON format:
        {
            "name": "my_state_machine",
            "initial_state": "idle",
            "states": [
                {
                    "name": "idle",
                    "description": "Waiting state",
                    "entry_condition": {"type": "always"},
                    "tasks": [
                        {
                            "task_type": "set_led",
                            "parameters": {"red": 0, "green": 0, "blue": 255}
                        }
                    ],
                    "exits": [
                        {
                            "condition": {"type": "timer", "duration": 5.0},
                            "destination": "moving"
                        }
                    ]
                },
                {
                    "name": "moving",
                    "description": "Final state - leaf (no exits)",
                    "tasks": [
                        {"task_type": "roll", "parameters": {"speed": 50, "heading": 0}}
                    ]
                }
            ]
        }
        """
        try:
            config = json.loads(msg.data)
            lane = self._resolve_lane(config.get('lane'))
            sm = self.state_machines[lane]
            self.get_logger().info(
                f'Received state machine configuration: '
                f'{config.get("name", "unnamed")} (lane: {lane})')

            # Configure the state machine on this lane (replaces any prior SM).
            if sm.configure(config):
                # Publish configuration success event
                self.publish_event('configuration_loaded', {
                    'lane': lane,
                    'name': config.get('name', 'unnamed'),
                    'num_states': len(sm.states),
                })

                # Execute initial state tasks for this lane.
                self.execute_current_state_tasks(lane)
            else:
                self.get_logger().error(f'Failed to configure state machine (lane: {lane})')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse configuration JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing configuration: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _resolve_lane(self, lane) -> str:
        """Map a config/control ``lane`` id to a known lane.

        An absent / null / unknown lane maps to the reserved ``default`` lane, so
        pre-lane single-SM publishers (which omit ``lane``) keep working.
        """
        if lane is None:
            return DEFAULT_SM_LANE
        lane = str(lane).lower()
        if lane not in self.state_machines:
            self.get_logger().warning(
                f'Unknown SM lane "{lane}"; using "{DEFAULT_SM_LANE}"')
            return DEFAULT_SM_LANE
        return lane

    def sensor_callback(self, msg: SpheroSensor):
        """
        Handle incoming Sphero sensor data for condition evaluation.

        Each field is fed into the state machine's topic-value store under its
        own field name (e.g. ``velocity_x``). Exit conditions can reference them
        as ``{type: topic_value, topic: 'velocity_x', operator: '>', value: 0.5}``.
        """
        try:
            sensor_fields = {
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
                'battery_percentage': msg.battery_percentage,
            }
            # Broadcast each sensor field to every lane's SM (values are keyed by
            # field name; each SM only reads the fields its states reference).
            for field_name, value in sensor_fields.items():
                for sm in self.state_machines.values():
                    sm.update_topic_value(field_name, value)

        except Exception as e:
            self.get_logger().error(f'Failed to process sensor data: {e}')

    def control_callback(self, msg: String):
        """
        Handle runtime control messages.

        Expected JSON: ``{"action": "pause" | "resume" | "clear", "lane": "..."}``.
        A ``lane`` targets one lane (absent -> ``default``); ``"scope": "all"``
        applies the action to every lane.
        """
        try:
            data = json.loads(msg.data)
            action = (data.get('action') or '').lower()
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in control command: {e}')
            return

        if str(data.get('scope', '')).lower() == 'all':
            target_lanes = list(self.state_machines.keys())
        else:
            target_lanes = [self._resolve_lane(data.get('lane'))]

        for lane in target_lanes:
            sm = self.state_machines[lane]
            if action == 'pause':
                ok = sm.pause()
                self.publish_event('sm_paused', {'lane': lane, 'success': ok})
            elif action == 'resume':
                ok = sm.resume()
                self.publish_event('sm_resumed', {'lane': lane, 'success': ok})
            elif action == 'clear':
                sm.clear()
                self._previous_paths[lane] = []
                self.publish_event('sm_cleared', {'lane': lane})
            else:
                self.get_logger().warning(f'Unknown control action: "{action}"')
                return

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

        # Node-level refcount: a second lane referencing the same topic shares the
        # single ROS subscription rather than creating a duplicate.
        if namespaced_topic in self.topic_subscriptions:
            self.topic_refcounts[namespaced_topic] = \
                self.topic_refcounts.get(namespaced_topic, 1) + 1
            self.get_logger().info(
                f'Already subscribed to topic: {namespaced_topic} '
                f'(refcount={self.topic_refcounts[namespaced_topic]})')
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

            # Create callback that stores the received message under the RAW topic
            # name (not the namespaced one) so SM condition lookups by
            # ``condition['topic']`` match. The actual ROS subscription still uses
            # the namespaced topic; only the SM-side key needs to match the config.
            def topic_callback(msg):
                # Broadcast the received value to every lane's SM (each SM only
                # reads the topics its own states reference).
                if field_path:
                    try:
                        value = msg
                        for field in field_path.split('.'):
                            value = getattr(value, field)
                    except AttributeError as e:
                        self.get_logger().error(f'Failed to extract field {field_path} from {namespaced_topic}: {e}')
                        value = msg
                else:
                    value = msg

                for sm in self.state_machines.values():
                    sm.update_topic_value(topic_name, value)

                self.get_logger().debug(f'Received message on {namespaced_topic} (key: {topic_name})')

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
            self.topic_refcounts[namespaced_topic] = 1
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

        if namespaced_topic not in self.topic_subscriptions:
            return

        # Refcounted: only tear down the ROS subscription when the last lane
        # referencing this topic releases it.
        count = self.topic_refcounts.get(namespaced_topic, 1) - 1
        if count > 0:
            self.topic_refcounts[namespaced_topic] = count
            self.get_logger().info(
                f'Released topic ref: {namespaced_topic} (refcount={count})')
            return

        self.destroy_subscription(self.topic_subscriptions[namespaced_topic])
        del self.topic_subscriptions[namespaced_topic]
        self.topic_refcounts.pop(namespaced_topic, None)
        if namespaced_topic in self.topic_message_types:
            del self.topic_message_types[namespaced_topic]
        self.get_logger().info(f'Unsubscribed from topic: {namespaced_topic}')

    # ===== State Machine Processing =====

    def update_callback(self):
        """Periodic update: tick EVERY lane's SM; each fires its own tasks."""
        for lane, sm in self.state_machines.items():
            result = sm.process()
            if result is None:
                continue

            # Handle events for this lane.
            for event in result.get('events', []):
                event_type = event.get('type')

                if event_type == 'state_timeout':
                    path_str = '·'.join(event.get('path') or [event.get('state', '?')])
                    self.get_logger().warning(
                        f'[{lane}] State {path_str} timed out after {event["elapsed"]:.1f}s'
                    )
                    self.publish_event('state_timeout', {'lane': lane, **event})

                elif event_type == 'state_transition':
                    path_str = '·'.join(event.get('path') or [event.get('to', '?')])
                    self.get_logger().info(
                        f'[{lane}] Transitioned to {path_str} (from {event.get("from", "?")})'
                    )
                    self.publish_event('state_transition', {'lane': lane, **event})

                    # Execute tasks for every newly-entered state on the new path.
                    self.execute_tasks_for_entered_branch(lane, event.get('path') or [])

    def execute_tasks_for_entered_branch(self, lane, new_path):
        """Run on-entry tasks for each state newly active on ``lane``'s path.

        Compares ``new_path`` against the lane's cached previous path; any level
        whose state name has changed (or is new) is considered entered, and its
        tasks are dispatched in root-to-leaf order. Levels that are unchanged are
        skipped — their tasks already fired when they were first entered.
        """
        old_path = self._previous_paths[lane]
        for depth, state_name in enumerate(new_path):
            unchanged = depth < len(old_path) and old_path[depth] == state_name
            if unchanged:
                continue
            self._dispatch_tasks_for_state(lane, new_path, depth, state_name)

        self._previous_paths[lane] = list(new_path)
        self.state_machines[lane].mark_tasks_completed()

    def _dispatch_tasks_for_state(self, lane, path, depth, state_name):
        """Resolve the state at ``path[:depth+1]`` in ``lane``'s SM tree and publish its tasks."""
        path_to_here = path[:depth + 1]
        bundles = self.state_machines[lane].get_tasks_for_path(path_to_here)
        if not bundles:
            return
        bundle = bundles[-1]  # tasks for state at this depth
        tasks = bundle.get('tasks') or []
        if not tasks:
            return

        path_str = '·'.join(path_to_here)
        self.get_logger().info(f'[{lane}] Executing {len(tasks)} task(s) for {path_str}')

        for idx, task in enumerate(tasks):
            task_type = task.get('task_type') or task.get('type', 'none')
            task_params = task.get('parameters') or task.get('params', {})

            self.get_logger().info(f'  Task {idx + 1}/{len(tasks)}: {task_type}')

            task_command = {
                'task_id': f'sm_{lane}_{state_name}_{idx}',
                'task_type': task_type,
                'parameters': task_params,
            }
            msg = String()
            msg.data = json.dumps(task_command)
            self.task_pub.publish(msg)

            self.publish_event('task_executed', {
                'lane': lane,
                'state': state_name,
                'path': path_to_here,
                'task_type': task_type,
                'params': task_params,
                'task_index': idx,
                'total_tasks': len(tasks),
            })

    def execute_current_state_tasks(self, lane):
        """Run on-entry tasks for every state on ``lane``'s active path (used after configure())."""
        sm = self.state_machines[lane]
        path = sm.get_active_path()
        if not path:
            self.get_logger().info(
                f'[{lane}] No tasks to execute (state machine has no active path)')
            sm.mark_tasks_completed()
            return
        # Treat the whole path as freshly entered.
        self._previous_paths[lane] = []
        self.execute_tasks_for_entered_branch(lane, path)

    def publish_status(self):
        """Publish per-lane state machine status.

        Back-compat: the ``default`` lane's status stays at the top level (its
        shape is unchanged), so pre-lane consumers keep working. A ``lanes`` map
        adds every lane's status for the multi-lane view.
        """
        status = self.state_machines[DEFAULT_SM_LANE].get_status()
        status['lanes'] = {
            lane: sm.get_status() for lane, sm in self.state_machines.items()
        }

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
            'timestamp': time.time(),
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
            node.destroy_node()

        # Shutdown ROS 2
        if rclpy.ok():
            rclpy.shutdown()

        print("Goodbye!")


if __name__ == '__main__':
    main()

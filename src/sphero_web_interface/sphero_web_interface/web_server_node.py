#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sphero Web Interface Server Node.

This node provides a Flask web server that interfaces with Sphero robots
through ROS2 topics, allowing web-based control and monitoring.
"""

import json
import threading
import time
import subprocess
import os
from pathlib import Path
from typing import Optional, Dict, Any, List

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from ament_index_python.packages import get_package_share_directory

from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit


class SpheroWebServerNode(Node):
    """
    ROS2 node that provides web interface for Sphero control.

    Features:
    - Connection management (start/stop controller nodes)
    - Real-time state monitoring
    - Sensor data visualization
    - LED matrix control
    - Motion control (heading, speed, roll)
    """

    def __init__(self):
        """Initialize the web server node."""
        super().__init__('sphero_web_server_node')

        # Controller process tracking
        self.controller_process: Optional[subprocess.Popen] = None
        self.task_controller_process: Optional[subprocess.Popen] = None
        self.state_machine_process: Optional[subprocess.Popen] = None
        self.task_executor_process: Optional[subprocess.Popen] = None
        self.sphero_name: Optional[str] = None
        self.controller_ready = False

        # Latest data cache
        self.latest_state: Dict[str, Any] = {}
        self.latest_sensors: Dict[str, Any] = {}
        self.latest_battery: Dict[str, Any] = {}
        self.latest_status: Dict[str, Any] = {}
        self.latest_task_status: Dict[str, Any] = {}
        self.latest_sm_status: Dict[str, Any] = {}
        self.latest_sm_events: List[Dict[str, Any]] = []

        # Error tracking
        self.last_message_time = time.time()
        self.connection_timeout = 10.0  # seconds
        self.error_count = 0
        self.max_errors = 5

        # Create subscribers for Sphero data
        self.state_sub = self.create_subscription(
            String,
            'sphero/state',
            self.state_callback,
            10
        )

        self.sensor_sub = self.create_subscription(
            String,
            'sphero/sensors',
            self.sensor_callback,
            10
        )

        self.battery_sub = self.create_subscription(
            BatteryState,
            'sphero/battery',
            self.battery_callback,
            10
        )

        self.status_sub = self.create_subscription(
            String,
            'sphero/status',
            self.status_callback,
            10
        )

        self.task_status_sub = self.create_subscription(
            String,
            'sphero/task/status',
            self.task_status_callback,
            10
        )

        # State machine subscribers
        self.sm_status_sub = self.create_subscription(
            String,
            '/state_machine/status',
            self.sm_status_callback,
            10
        )

        self.sm_events_sub = self.create_subscription(
            String,
            '/state_machine/events',
            self.sm_events_callback,
            10
        )

        # Create publishers for Sphero commands
        self.led_pub = self.create_publisher(String, 'sphero/led', 10)
        self.roll_pub = self.create_publisher(String, 'sphero/roll', 10)
        self.spin_pub = self.create_publisher(String, 'sphero/spin', 10)
        self.heading_pub = self.create_publisher(String, 'sphero/heading', 10)
        self.speed_pub = self.create_publisher(String, 'sphero/speed', 10)
        self.stop_pub = self.create_publisher(String, 'sphero/stop', 10)
        self.reset_aim_pub = self.create_publisher(String, 'sphero/reset_aim', 10)
        self.matrix_pub = self.create_publisher(String, 'sphero/matrix', 10)
        self.task_pub = self.create_publisher(String, 'sphero/task', 10)
        self.sm_config_pub = self.create_publisher(String, '/state_machine/config', 10)

        self.get_logger().info('Sphero Web Server Node initialized')
        self.get_logger().info('Web interface will be available at http://localhost:5000')

    def state_callback(self, msg: String):
        """Handle incoming state messages."""
        try:
            self.latest_state = json.loads(msg.data)
            # Emit to all connected web clients via SocketIO
            if hasattr(self, 'socketio'):
                self.socketio.emit('state_update', self.latest_state)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse state JSON: {e}')

    def sensor_callback(self, msg: String):
        """Handle incoming sensor messages."""
        try:
            # Convert SpheroSensor message to dict if needed
            sensor_data = {
                'timestamp': time.time(),
                'raw_message': msg.data
            }
            self.latest_sensors = sensor_data
            if hasattr(self, 'socketio'):
                self.socketio.emit('sensor_update', sensor_data)
        except Exception as e:
            self.get_logger().error(f'Failed to process sensor data: {e}')

    def battery_callback(self, msg: BatteryState):
        """Handle incoming battery messages."""
        try:
            battery_data = {
                'voltage': msg.voltage,
                'percentage': msg.percentage,
                'power_supply_status': msg.power_supply_status,
                'power_supply_health': msg.power_supply_health,
                'timestamp': time.time()
            }
            self.latest_battery = battery_data
            if hasattr(self, 'socketio'):
                self.socketio.emit('battery_update', battery_data)
        except Exception as e:
            self.get_logger().error(f'Failed to process battery data: {e}')

    def status_callback(self, msg: String):
        """Handle incoming status/heartbeat messages."""
        try:
            self.latest_status = json.loads(msg.data)
            self.controller_ready = True
            self.last_message_time = time.time()
            self.error_count = 0  # Reset error count on successful message
            if hasattr(self, 'socketio'):
                self.socketio.emit('status_update', self.latest_status)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse status JSON: {e}')
            self.handle_error('Failed to parse status message')

    def task_status_callback(self, msg: String):
        """Handle incoming task status messages."""
        try:
            self.latest_task_status = json.loads(msg.data)
            if hasattr(self, 'socketio'):
                self.socketio.emit('task_status_update', self.latest_task_status)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse task status JSON: {e}')

    def sm_status_callback(self, msg: String):
        """Handle incoming state machine status messages."""
        try:
            self.latest_sm_status = json.loads(msg.data)
            if hasattr(self, 'socketio'):
                self.socketio.emit('sm_status_update', self.latest_sm_status)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse state machine status JSON: {e}')

    def sm_events_callback(self, msg: String):
        """Handle incoming state machine event messages."""
        try:
            event = json.loads(msg.data)
            # Keep last 50 events
            self.latest_sm_events.append(event)
            if len(self.latest_sm_events) > 50:
                self.latest_sm_events.pop(0)

            if hasattr(self, 'socketio'):
                self.socketio.emit('sm_event', event)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse state machine event JSON: {e}')

    def handle_error(self, error_message: str):
        """Handle errors and emit to web clients."""
        self.error_count += 1
        self.get_logger().error(f'Error #{self.error_count}: {error_message}')

        if hasattr(self, 'socketio'):
            self.socketio.emit('error', {
                'message': error_message,
                'error_count': self.error_count,
                'timestamp': time.time()
            })

        # Auto-disconnect if too many errors
        if self.error_count >= self.max_errors:
            self.get_logger().error(f'Too many errors ({self.error_count}). Auto-disconnecting...')
            if hasattr(self, 'socketio'):
                self.socketio.emit('force_disconnect', {
                    'reason': f'Too many errors ({self.error_count})',
                    'message': 'Connection lost due to repeated errors. Please reconnect.'
                })
            # Stop the controller
            if self.controller_process is not None:
                self.stop_controller()

    def check_controller_health(self):
        """Check if controller process is still running and healthy."""
        if self.controller_process is None:
            return True  # No controller running, that's fine

        # Check if process has exited
        poll_result = self.controller_process.poll()
        if poll_result is not None:
            self.get_logger().error(f'Controller process died with exit code: {poll_result}')
            self.handle_error(f'Controller process crashed (exit code: {poll_result})')
            if hasattr(self, 'socketio'):
                self.socketio.emit('force_disconnect', {
                    'reason': 'Controller process crashed',
                    'message': 'The Sphero controller has stopped unexpectedly. Please reconnect.'
                })
            self.controller_process = None
            self.controller_ready = False
            return False

        # Check for message timeout
        if self.controller_ready:
            time_since_last = time.time() - self.last_message_time
            if time_since_last > self.connection_timeout:
                self.get_logger().warning(f'No messages received for {time_since_last:.1f}s')
                self.handle_error(f'Connection timeout ({time_since_last:.1f}s)')
                return False

        return True

    def start_controller(self, sphero_name: str) -> Dict[str, Any]:
        """
        Start the sphero_controller_node for the specified Sphero.

        Args:
            sphero_name: Name of the Sphero robot to connect to

        Returns:
            Dict with status and message
        """
        if self.controller_process is not None:
            return {
                'success': False,
                'message': f'Controller already running for {self.sphero_name}'
            }

        try:
            self.sphero_name = sphero_name
            self.controller_ready = False

            # Reset error tracking
            self.error_count = 0
            self.last_message_time = time.time()

            # Start the controller node as a subprocess
            cmd = [
                'ros2', 'run', 'sphero_package', 'sphero_controller_node.py',
                '--ros-args',
                '-p', f'toy_name:={sphero_name}'
            ]

            self.get_logger().info(f'Starting controller with command: {" ".join(cmd)}')
            self.get_logger().info(f'Connecting to Sphero: {sphero_name}')

            # Start process without capturing output so we can see it in terminal
            self.controller_process = subprocess.Popen(
                cmd,
                stdout=None,  # Let output go to terminal
                stderr=None,  # Let errors go to terminal
                text=True
            )

            self.get_logger().info(f'Controller process started with PID: {self.controller_process.pid}')

            # Wait a bit for initialization
            time.sleep(3)

            # Check if process is still running
            poll_result = self.controller_process.poll()
            if poll_result is not None:
                self.get_logger().error(f'Controller process exited with code: {poll_result}')
                self.controller_process = None
                error_msg = f'Controller failed to start (exit code {poll_result}). Check terminal for details.'
                self.handle_error(error_msg)
                return {
                    'success': False,
                    'message': error_msg
                }

            self.get_logger().info(f'Controller process running successfully')
            return {
                'success': True,
                'message': f'Controller started for {sphero_name}. Waiting for connection...',
                'sphero_name': sphero_name
            }

        except Exception as e:
            self.get_logger().error(f'Failed to start controller: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return {
                'success': False,
                'message': f'Error starting controller: {str(e)}'
            }

    def stop_controller(self) -> Dict[str, Any]:
        """
        Stop the running sphero_controller_node.

        Returns:
            Dict with status and message
        """
        if self.controller_process is None:
            return {
                'success': False,
                'message': 'No controller is running'
            }

        try:
            self.get_logger().info(f'Stopping controller for {self.sphero_name}...')
            self.controller_process.terminate()
            self.controller_process.wait(timeout=5)

            self.controller_process = None
            self.sphero_name = None
            self.controller_ready = False
            self.latest_state = {}
            self.latest_sensors = {}
            self.latest_battery = {}
            self.latest_status = {}

            return {
                'success': True,
                'message': 'Controller stopped successfully'
            }

        except subprocess.TimeoutExpired:
            self.controller_process.kill()
            self.controller_process = None
            return {
                'success': True,
                'message': 'Controller forcefully stopped'
            }
        except Exception as e:
            self.get_logger().error(f'Failed to stop controller: {e}')
            return {
                'success': False,
                'message': f'Error stopping controller: {str(e)}'
            }

    def send_led_command(self, red: int, green: int, blue: int, led: str = 'main'):
        """Send LED color command."""
        msg = String()
        msg.data = json.dumps({'red': red, 'green': green, 'blue': blue, 'led': led})
        self.led_pub.publish(msg)
        self.get_logger().debug(f'{led.capitalize()} LED command sent: RGB({red}, {green}, {blue})')

    def send_roll_command(self, heading: int, speed: int, duration: float = 0.0):
        """Send roll command."""
        msg = String()
        msg.data = json.dumps({'heading': heading, 'speed': speed, 'duration': duration})
        self.roll_pub.publish(msg)
        self.get_logger().debug(f'Roll command sent: heading={heading}, speed={speed}')

    def send_heading_command(self, heading: int):
        """Send heading command."""
        msg = String()
        msg.data = json.dumps({'heading': heading})
        self.heading_pub.publish(msg)
        self.get_logger().debug(f'Heading command sent: {heading}')

    def send_speed_command(self, speed: int, duration: float = 0.0):
        """Send speed command."""
        msg = String()
        msg.data = json.dumps({'speed': speed, 'duration': duration})
        self.speed_pub.publish(msg)
        self.get_logger().debug(f'Speed command sent: {speed}')

    def send_stop_command(self):
        """Send stop command."""
        msg = String()
        msg.data = json.dumps({})
        self.stop_pub.publish(msg)
        self.get_logger().debug('Stop command sent')

    def send_reset_aim_command(self):
        """Send reset aim (reset to origin) command."""
        msg = String()
        msg.data = json.dumps({})
        self.reset_aim_pub.publish(msg)
        self.get_logger().info('Reset to origin command sent')

    def send_matrix_command(self, pattern: str, red: int = 255, green: int = 255, blue: int = 255):
        """Send matrix display command."""
        msg = String()
        msg.data = json.dumps({
            'pattern': pattern,
            'red': red,
            'green': green,
            'blue': blue,
            'duration': 0
        })
        self.matrix_pub.publish(msg)
        self.get_logger().debug(f'Matrix command sent: {pattern}')

    def start_task_controller(self) -> Dict[str, Any]:
        """
        Start the sphero_task_controller_node.

        Returns:
            Dict with status and message
        """
        if self.task_controller_process is not None:
            return {
                'success': False,
                'message': 'Task controller already running'
            }

        try:
            cmd = ['ros2', 'run', 'sphero_task_controller', 'task_controller']

            self.get_logger().info(f'Starting task controller with command: {" ".join(cmd)}')

            self.task_controller_process = subprocess.Popen(
                cmd,
                stdout=None,
                stderr=None,
                text=True
            )

            self.get_logger().info(f'Task controller process started with PID: {self.task_controller_process.pid}')

            time.sleep(1)

            poll_result = self.task_controller_process.poll()
            if poll_result is not None:
                self.get_logger().error(f'Task controller process exited with code: {poll_result}')
                self.task_controller_process = None
                return {
                    'success': False,
                    'message': f'Task controller failed to start (exit code {poll_result})'
                }

            return {
                'success': True,
                'message': 'Task controller started successfully'
            }

        except Exception as e:
            self.get_logger().error(f'Failed to start task controller: {e}')
            return {
                'success': False,
                'message': f'Error starting task controller: {str(e)}'
            }

    def stop_task_controller(self) -> Dict[str, Any]:
        """
        Stop the running sphero_task_controller_node.

        Returns:
            Dict with status and message
        """
        if self.task_controller_process is None:
            return {
                'success': False,
                'message': 'No task controller is running'
            }

        try:
            self.get_logger().info('Stopping task controller...')
            self.task_controller_process.terminate()
            self.task_controller_process.wait(timeout=5)

            self.task_controller_process = None
            self.latest_task_status = {}

            return {
                'success': True,
                'message': 'Task controller stopped successfully'
            }

        except subprocess.TimeoutExpired:
            self.task_controller_process.kill()
            self.task_controller_process = None
            return {
                'success': True,
                'message': 'Task controller forcefully stopped'
            }
        except Exception as e:
            self.get_logger().error(f'Failed to stop task controller: {e}')
            return {
                'success': False,
                'message': f'Error stopping task controller: {str(e)}'
            }

    def send_task_command(self, task_data: Dict[str, Any]):
        """Send task command to task controller."""
        msg = String()
        msg.data = json.dumps(task_data)
        self.task_pub.publish(msg)
        self.get_logger().debug(f'Task command sent: {task_data.get("task_type", "unknown")}')

    def clear_matrix(self):
        """Clear the LED matrix."""
        msg = String()
        # Send an all-zeros matrix to clear
        msg.data = json.dumps({
            'pattern': 'custom',
            'matrix': [0] * 64,  # All LEDs off
            'red': 0,
            'green': 0,
            'blue': 0,
            'duration': 0
        })
        self.matrix_pub.publish(msg)
        self.get_logger().debug('Matrix cleared')

    def start_state_machine(self) -> Dict[str, Any]:
        """
        Start the state machine controller node and task executor.

        Returns:
            Dict with status and message
        """
        if self.state_machine_process is not None:
            return {
                'success': False,
                'message': 'State machine already running'
            }

        try:
            # Start the state machine controller
            cmd = ['ros2', 'run', 'sphero_statemachine', 'state_machine_controller']

            self.get_logger().info(f'Starting state machine controller with command: {" ".join(cmd)}')

            self.state_machine_process = subprocess.Popen(
                cmd,
                stdout=None,
                stderr=None,
                text=True
            )

            self.get_logger().info(f'State machine process started with PID: {self.state_machine_process.pid}')

            time.sleep(1)

            poll_result = self.state_machine_process.poll()
            if poll_result is not None:
                self.get_logger().error(f'State machine process exited with code: {poll_result}')
                self.state_machine_process = None
                return {
                    'success': False,
                    'message': f'State machine failed to start (exit code {poll_result})'
                }

            # Start the task executor
            executor_cmd = ['ros2', 'run', 'sphero_statemachine', 'task_executor']

            self.get_logger().info(f'Starting task executor with command: {" ".join(executor_cmd)}')

            self.task_executor_process = subprocess.Popen(
                executor_cmd,
                stdout=None,
                stderr=None,
                text=True
            )

            self.get_logger().info(f'Task executor process started with PID: {self.task_executor_process.pid}')

            time.sleep(0.5)

            poll_result = self.task_executor_process.poll()
            if poll_result is not None:
                self.get_logger().error(f'Task executor process exited with code: {poll_result}')
                # Stop state machine since executor failed
                if self.state_machine_process:
                    self.state_machine_process.terminate()
                    self.state_machine_process = None
                self.task_executor_process = None
                return {
                    'success': False,
                    'message': f'Task executor failed to start (exit code {poll_result})'
                }

            return {
                'success': True,
                'message': 'State machine controller and task executor started successfully'
            }

        except Exception as e:
            self.get_logger().error(f'Failed to start state machine: {e}')
            return {
                'success': False,
                'message': f'Error starting state machine: {str(e)}'
            }

    def stop_state_machine(self) -> Dict[str, Any]:
        """
        Stop the running state machine controller and task executor.

        Returns:
            Dict with status and message
        """
        if self.state_machine_process is None:
            return {
                'success': False,
                'message': 'No state machine is running'
            }

        try:
            # Stop task executor first
            if self.task_executor_process is not None:
                self.get_logger().info('Stopping task executor...')
                try:
                    self.task_executor_process.terminate()
                    self.task_executor_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.task_executor_process.kill()
                self.task_executor_process = None

            # Stop state machine controller
            self.get_logger().info('Stopping state machine controller...')
            self.state_machine_process.terminate()
            self.state_machine_process.wait(timeout=5)

            self.state_machine_process = None
            self.latest_sm_status = {}
            self.latest_sm_events = []

            return {
                'success': True,
                'message': 'State machine controller and task executor stopped successfully'
            }

        except subprocess.TimeoutExpired:
            self.state_machine_process.kill()
            self.state_machine_process = None
            return {
                'success': True,
                'message': 'State machine controller forcefully stopped'
            }
        except Exception as e:
            self.get_logger().error(f'Failed to stop state machine: {e}')
            return {
                'success': False,
                'message': f'Error stopping state machine: {str(e)}'
            }

    def send_state_machine_config(self, config: Dict[str, Any]):
        """Send state machine configuration."""
        msg = String()
        msg.data = json.dumps(config)
        self.sm_config_pub.publish(msg)
        self.get_logger().debug(f'State machine config sent: {config.get("name", "unnamed")}')


def create_flask_app(ros_node: SpheroWebServerNode):
    """Create and configure the Flask application."""
    # Get the package share directory where templates and static files are installed
    try:
        package_share_dir = get_package_share_directory('sphero_web_interface')
        template_dir = os.path.join(package_share_dir, 'templates')
        static_dir = os.path.join(package_share_dir, 'static')

        ros_node.get_logger().info(f'Using template directory: {template_dir}')
        ros_node.get_logger().info(f'Using static directory: {static_dir}')

        app = Flask(__name__,
                   template_folder=template_dir,
                   static_folder=static_dir)
    except Exception as e:
        ros_node.get_logger().warn(f'Could not find package share directory: {e}')
        ros_node.get_logger().warn('Falling back to default Flask paths')
        app = Flask(__name__)

    app.config['SECRET_KEY'] = 'sphero_web_interface_secret_key'

    # Disable Flask development server warnings and logs
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)

    socketio = SocketIO(app, cors_allowed_origins="*", logger=False, engineio_logger=False)

    # Store socketio reference in node for callbacks
    ros_node.socketio = socketio

    @app.route('/')
    def index():
        """Serve the main web interface."""
        return render_template('index.html')

    @app.route('/state_machine')
    def state_machine():
        """Serve the state machine configuration interface."""
        return render_template('state_machine.html')

    @app.route('/api/connect', methods=['POST'])
    def connect():
        """Connect to a Sphero by name."""
        data = request.get_json()
        sphero_name = data.get('sphero_name', '')

        if not sphero_name:
            return jsonify({'success': False, 'message': 'Sphero name is required'}), 400

        result = ros_node.start_controller(sphero_name)
        return jsonify(result)

    @app.route('/api/disconnect', methods=['POST'])
    def disconnect():
        """Disconnect from the current Sphero."""
        result = ros_node.stop_controller()
        return jsonify(result)

    @app.route('/api/status', methods=['GET'])
    def status():
        """Get current connection status."""
        # Check controller health
        is_healthy = ros_node.check_controller_health()

        return jsonify({
            'connected': ros_node.controller_process is not None,
            'ready': ros_node.controller_ready,
            'healthy': is_healthy,
            'sphero_name': ros_node.sphero_name,
            'error_count': ros_node.error_count,
            'latest_state': ros_node.latest_state,
            'latest_battery': ros_node.latest_battery,
            'latest_status': ros_node.latest_status
        })

    @app.route('/api/led', methods=['POST'])
    def set_led():
        """Set LED color."""
        data = request.get_json()
        ros_node.send_led_command(
            data.get('red', 0),
            data.get('green', 0),
            data.get('blue', 0),
            data.get('led', 'main')
        )
        return jsonify({'success': True})

    @app.route('/api/motion/roll', methods=['POST'])
    def roll():
        """Send roll command."""
        data = request.get_json()
        ros_node.send_roll_command(
            data.get('heading', 0),
            data.get('speed', 100),
            data.get('duration', 0.0)
        )
        return jsonify({'success': True})

    @app.route('/api/motion/heading', methods=['POST'])
    def heading():
        """Set heading."""
        data = request.get_json()
        ros_node.send_heading_command(data.get('heading', 0))
        return jsonify({'success': True})

    @app.route('/api/motion/speed', methods=['POST'])
    def speed():
        """Set speed."""
        data = request.get_json()
        ros_node.send_speed_command(
            data.get('speed', 0),
            data.get('duration', 0.0)
        )
        return jsonify({'success': True})

    @app.route('/api/motion/stop', methods=['POST'])
    def stop():
        """Stop the Sphero."""
        ros_node.send_stop_command()
        return jsonify({'success': True})

    @app.route('/api/motion/reset', methods=['POST'])
    def reset_origin():
        """Reset Sphero to origin (heading=0, position=0,0)."""
        ros_node.send_reset_aim_command()
        return jsonify({'success': True, 'message': 'Sphero reset to origin'})

    @app.route('/api/matrix', methods=['POST'])
    def matrix():
        """Display pattern on LED matrix."""
        data = request.get_json()
        ros_node.send_matrix_command(
            data.get('pattern', 'smile'),
            data.get('red', 255),
            data.get('green', 255),
            data.get('blue', 255)
        )
        return jsonify({'success': True})

    @app.route('/api/matrix/clear', methods=['POST'])
    def clear_matrix():
        """Clear the LED matrix."""
        ros_node.clear_matrix()
        return jsonify({'success': True})

    @app.route('/api/task_controller/start', methods=['POST'])
    def start_task_controller():
        """Start the task controller."""
        result = ros_node.start_task_controller()
        return jsonify(result)

    @app.route('/api/task_controller/stop', methods=['POST'])
    def stop_task_controller():
        """Stop the task controller."""
        result = ros_node.stop_task_controller()
        return jsonify(result)

    @app.route('/api/task_controller/status', methods=['GET'])
    def task_controller_status():
        """Get task controller status."""
        return jsonify({
            'running': ros_node.task_controller_process is not None,
            'latest_task_status': ros_node.latest_task_status
        })

    @app.route('/api/task', methods=['POST'])
    def submit_task():
        """Submit a task to the task controller."""
        data = request.get_json()
        ros_node.send_task_command(data)
        return jsonify({'success': True, 'message': 'Task submitted'})

    @app.route('/api/state_machine/start', methods=['POST'])
    def start_state_machine():
        """Start the state machine controller."""
        result = ros_node.start_state_machine()
        return jsonify(result)

    @app.route('/api/state_machine/stop', methods=['POST'])
    def stop_state_machine():
        """Stop the state machine controller."""
        result = ros_node.stop_state_machine()
        return jsonify(result)

    @app.route('/api/state_machine/config', methods=['POST'])
    def configure_state_machine():
        """Send configuration to the state machine."""
        data = request.get_json()
        ros_node.send_state_machine_config(data)
        return jsonify({'success': True, 'message': 'Configuration sent'})

    @app.route('/api/state_machine/status', methods=['GET'])
    def state_machine_status():
        """Get state machine status."""
        return jsonify({
            'running': ros_node.state_machine_process is not None,
            'latest_status': ros_node.latest_sm_status,
            'recent_events': ros_node.latest_sm_events[-10:] if ros_node.latest_sm_events else []
        })

    @socketio.on('connect')
    def handle_connect():
        """Handle WebSocket connection."""
        ros_node.get_logger().debug('Web client connected')
        emit('connected', {'message': 'Connected to Sphero Web Interface'})

    @socketio.on('disconnect')
    def handle_disconnect():
        """Handle WebSocket disconnection."""
        ros_node.get_logger().debug('Web client disconnected')

    return app, socketio


def main(args=None):
    """Main entry point for the web server node."""
    rclpy.init(args=args)

    # Create the ROS2 node
    node = SpheroWebServerNode()

    # Create Flask app
    app, socketio = create_flask_app(node)

    # Create executor for ROS2
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Run ROS2 in a separate thread
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        # Run Flask app (blocking)
        node.get_logger().info('Starting web server on http://0.0.0.0:5000')
        socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down web server...')
    finally:
        # Cleanup
        if node.controller_process is not None:
            node.stop_controller()
        if node.task_controller_process is not None:
            node.stop_task_controller()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

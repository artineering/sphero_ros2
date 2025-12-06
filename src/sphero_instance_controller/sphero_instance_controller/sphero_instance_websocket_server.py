#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sphero Instance WebSocket Server Node.

This node provides a WebSocket interface for a single Sphero instance,
using namespaced topics (sphero/<sphero_name>/*) for multi-robot support.
"""

import json
import threading
import time
import subprocess
import os
import signal
from typing import Optional, Dict, Any, List

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from sphero_instance_controller.msg import SpheroSensor

from flask import Flask
from flask_socketio import SocketIO, emit
from flask_cors import CORS


class SpheroInstanceWebSocketServer(Node):
    """
    ROS2 node that provides WebSocket interface for a single Sphero instance.

    Uses namespaced topics: sphero/<sphero_name>/*
    """

    def __init__(self, sphero_name: str, websocket_port: int, external_localization: bool = False):
        """
        Initialize the websocket server node.

        Args:
            sphero_name: Name of the Sphero (e.g., 'SB-3660')
            websocket_port: Port for WebSocket server (e.g., 5001, 5002, ...)
            external_localization: Whether to use external localization (ArUco SLAM)
        """
        super().__init__(f'sphero_websocket_server_{sphero_name.replace("-", "_")}')

        self.sphero_name = sphero_name
        self.websocket_port = websocket_port
        self.external_localization = external_localization
        # Sanitize topic name: replace hyphens with underscores (ROS2 topic naming rules)
        self.topic_name_safe = sphero_name.replace("-", "_")
        self.topic_prefix = f'sphero/{self.topic_name_safe}'

        # Controller process tracking
        self.device_controller_process: Optional[subprocess.Popen] = None
        self.task_controller_process: Optional[subprocess.Popen] = None
        self.sm_controller_process: Optional[subprocess.Popen] = None
        self.controller_ready = False

        # Latest data cache
        self.latest_state: Dict[str, Any] = {}
        self.latest_sensors: Dict[str, Any] = {}
        self.latest_battery: Dict[str, Any] = {'percentage': 0, 'voltage': 0}
        self.latest_status: Dict[str, Any] = {}
        self.latest_task_status: Dict[str, Any] = {}
        self.latest_sm_status: Dict[str, Any] = {}

        # Connection tracking
        self.last_message_time = time.time()
        self.connection_timeout = 10.0

        # Create subscribers for namespaced topics
        self._create_subscribers()

        # Create publishers for namespaced topics
        self._create_publishers()

        self.get_logger().info(f'WebSocket server initialized for {sphero_name} on port {websocket_port}')
        self.get_logger().info(f'  Topic prefix: {self.topic_prefix}')

    def _create_subscribers(self):
        """Create all ROS subscribers for namespaced topics."""
        self.state_sub = self.create_subscription(
            String,
            f'{self.topic_prefix}/state',
            self.state_callback,
            10
        )

        self.sensor_sub = self.create_subscription(
            SpheroSensor,
            f'{self.topic_prefix}/sensors',
            self.sensor_callback,
            10
        )

        self.battery_sub = self.create_subscription(
            BatteryState,
            f'{self.topic_prefix}/battery',
            self.battery_callback,
            10
        )

        self.status_sub = self.create_subscription(
            String,
            f'{self.topic_prefix}/status',
            self.status_callback,
            10
        )

        self.task_status_sub = self.create_subscription(
            String,
            f'{self.topic_prefix}/task/status',
            self.task_status_callback,
            10
        )

        self.sm_status_sub = self.create_subscription(
            String,
            f'{self.topic_prefix}/state_machine/status',
            self.sm_status_callback,
            10
        )

        self.device_error_sub = self.create_subscription(
            String,
            f'{self.topic_prefix}/device_error',
            self.device_error_callback,
            10
        )

    def _create_publishers(self):
        """Create all ROS publishers for namespaced topics."""
        self.led_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/led',
            10
        )

        self.roll_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/roll',
            10
        )

        self.stop_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/stop',
            10
        )

        self.matrix_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/matrix',
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

        self.raw_motor_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/raw_motor',
            10
        )

        self.spin_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/spin',
            10
        )

        self.reset_aim_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/reset_aim',
            10
        )

        self.task_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/task',
            10
        )

        self.sm_config_pub = self.create_publisher(
            String,
            f'{self.topic_prefix}/state_machine/config',
            10
        )

    # ===== Callbacks =====

    def state_callback(self, msg: String):
        """Handle state updates."""
        try:
            self.latest_state = json.loads(msg.data)
            self.last_message_time = time.time()
            # Emit to websocket clients
            if hasattr(self, 'socketio'):
                self.socketio.emit('state_update', self.latest_state)
                self.get_logger().debug(f'Emitted state_update: {self.latest_state.get("toy_name", "unknown")}')
            else:
                self.get_logger().warn('state_callback: socketio not available yet')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse state JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in state_callback: {e}')

    def sensor_callback(self, msg: SpheroSensor):
        """Handle sensor updates."""
        try:
            # Convert SpheroSensor message to dict
            sensor_data = {
                'accelerometer': {
                    'x': msg.accel_x,
                    'y': msg.accel_y,
                    'z': msg.accel_z
                },
                'gyroscope': {
                    'x': msg.gyro_x,
                    'y': msg.gyro_y,
                    'z': msg.gyro_z
                },
                'orientation': {
                    'pitch': msg.pitch,
                    'roll': msg.roll,
                    'yaw': msg.yaw
                },
                'velocity': {
                    'x': msg.velocity_x,
                    'y': msg.velocity_y
                },
                'position': {
                    'x': msg.x,
                    'y': msg.y
                }
            }
            self.latest_sensors = sensor_data
            # Emit to websocket clients
            if hasattr(self, 'socketio'):
                self.socketio.emit('sensor_update', sensor_data)
        except Exception as e:
            self.get_logger().error(f'Error in sensor_callback: {e}')

    def battery_callback(self, msg: BatteryState):
        """Handle battery updates."""
        self.latest_battery = {
            'percentage': msg.percentage * 100,
            'voltage': msg.voltage
        }
        self.last_message_time = time.time()
        # Emit to websocket clients
        if hasattr(self, 'socketio'):
            self.socketio.emit('battery_update', self.latest_battery)

    def status_callback(self, msg: String):
        """Handle status updates."""
        try:
            self.latest_status = json.loads(msg.data)
            self.last_message_time = time.time()
            # Emit to websocket clients
            if hasattr(self, 'socketio'):
                self.socketio.emit('status_update', self.latest_status)
        except json.JSONDecodeError:
            pass

    def task_status_callback(self, msg: String):
        """Handle task status updates."""
        try:
            self.latest_task_status = json.loads(msg.data)
            # Emit to websocket clients
            if hasattr(self, 'socketio'):
                self.socketio.emit('task_status_update', self.latest_task_status)
        except json.JSONDecodeError:
            pass

    def sm_status_callback(self, msg: String):
        """Handle state machine status updates."""
        try:
            self.latest_sm_status = json.loads(msg.data)
            # Emit to websocket clients
            if hasattr(self, 'socketio'):
                self.socketio.emit('sm_status_update', self.latest_sm_status)
        except json.JSONDecodeError:
            pass

    def device_error_callback(self, msg: String):
        """Handle device error (e.g., toy not found)."""
        try:
            error_data = json.loads(msg.data)
            error_type = error_data.get('error', 'unknown')

            if error_type == 'toy_not_found':
                self._shutdown_triggered = True  # Mark that we received the error
                self.get_logger().error(f"✗ Sphero {self.sphero_name} not found - initiating shutdown")
                # Emit error to websocket clients
                if hasattr(self, 'socketio'):
                    self.socketio.emit('device_error', error_data)

                # Trigger shutdown of all controllers
                import threading
                threading.Thread(target=self._shutdown_after_delay, daemon=True).start()
        except json.JSONDecodeError:
            pass

    def _shutdown_after_delay(self):
        """Shutdown all controllers after a brief delay to allow error message to propagate."""
        time.sleep(1)  # Give time for error message to be delivered
        self.get_logger().info(f"Shutting down all controllers for {self.sphero_name} due to device error")
        self.stop_controllers()
        # Exit the process
        import os
        os._exit(1)

    # ===== Controller Management =====

    def start_controllers(self) -> bool:
        """Start all Sphero controllers for this instance."""
        try:
            self.get_logger().info('='*70)
            self.get_logger().info(f'🚀 LAUNCHING CONTROLLERS FOR {self.sphero_name}')
            self.get_logger().info('='*70)

            # Start device controller (inherit stdout/stderr to see debug logs)
            # Use preexec_fn to create new process group for proper cleanup
            import os
            self.get_logger().info('Launching Device Controller...')
            self.device_controller_process = subprocess.Popen([
                'ros2', 'run', 'sphero_instance_controller',
                'sphero_instance_device_controller_node.py',
                '--ros-args',
                '-p', f'sphero_name:={self.sphero_name}',
                '-p', f'external_localization:={str(self.external_localization).lower()}'
            ], stdout=None, stderr=None, preexec_fn=os.setpgrp)  # Create new process group

            # Monitor device controller for early exit (toy not found)
            for i in range(20):  # Check for 10 seconds (20 * 0.5s)
                time.sleep(0.5)
                if self.device_controller_process.poll() is not None:
                    # Device controller exited - likely toy not found
                    self.get_logger().error(f'Device controller exited early (exit code: {self.device_controller_process.returncode})')
                    # Wait a bit for error message to arrive via topic
                    time.sleep(1)
                    # If we didn't receive the error via topic, trigger shutdown manually
                    if not hasattr(self, '_shutdown_triggered'):
                        self.get_logger().error(f'Toy not found for {self.sphero_name} - initiating shutdown')
                        import threading
                        threading.Thread(target=self._shutdown_after_delay, daemon=True).start()
                    return False
                # After 2 seconds, break to continue if still running
                if i >= 4:
                    break

            # Start task controller (inherit stdout/stderr to see debug logs)
            self.get_logger().info('Launching Task Controller...')
            self.task_controller_process = subprocess.Popen([
                'ros2', 'run', 'sphero_instance_controller',
                'sphero_instance_task_controller_node.py',
                '--ros-args', '-p', f'sphero_name:={self.sphero_name}'
            ], stdout=None, stderr=None, preexec_fn=os.setpgrp)  # Create new process group

            # Start state machine controller (inherit stdout/stderr to see debug logs)
            self.get_logger().info('Launching State Machine Controller...')
            self.sm_controller_process = subprocess.Popen([
                'ros2', 'run', 'sphero_instance_controller',
                'sphero_instance_statemachine_controller_node.py',
                '--ros-args', '-p', f'sphero_name:={self.sphero_name}'
            ], stdout=None, stderr=None, preexec_fn=os.setpgrp)  # Create new process group

            self.controller_ready = True
            self.get_logger().info('='*70)
            self.get_logger().info(f'✅ ALL CONTROLLERS LAUNCHED FOR {self.sphero_name}')
            self.get_logger().info('='*70)
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to start controllers: {e}')
            return False

    def stop_controllers(self):
        """Stop all Sphero controllers for this instance."""
        self.get_logger().info(f'Stopping controllers for {self.sphero_name}...')

        for process in [self.device_controller_process,
                       self.task_controller_process,
                       self.sm_controller_process]:
            if process:
                try:
                    # Send SIGTERM to process group to ensure child processes are also terminated
                    import os
                    try:
                        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    except (ProcessLookupError, AttributeError):
                        # Fallback to regular terminate if process group doesn't exist
                        process.terminate()

                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.get_logger().warning(f'Controller process did not terminate in time, killing...')
                    try:
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    except (ProcessLookupError, AttributeError):
                        process.kill()
                    process.wait()
                except Exception as e:
                    self.get_logger().error(f'Error stopping controller: {e}')
                    try:
                        process.kill()
                    except:
                        pass

        self.controller_ready = False
        self.get_logger().info(f'Controllers stopped for {self.sphero_name}')

    # ===== Command Publishing =====

    def publish_led_command(self, red: int, green: int, blue: int, led_type: str = 'main'):
        """Publish LED color command."""
        msg = String()
        msg.data = json.dumps({'red': red, 'green': green, 'blue': blue, 'type': led_type})
        self.led_pub.publish(msg)

    def publish_roll_command(self, heading: int, speed: int, duration: float = 0):
        """Publish roll movement command."""
        msg = String()
        msg.data = json.dumps({'heading': heading, 'speed': speed, 'duration': duration})
        self.roll_pub.publish(msg)

    def publish_stop_command(self):
        """Publish stop command."""
        msg = String()
        msg.data = '{}'
        self.stop_pub.publish(msg)

    def publish_task_command(self, task_data: dict):
        """Publish high-level task command."""
        msg = String()
        msg.data = json.dumps(task_data)
        self.task_pub.publish(msg)

    def publish_sm_config(self, config: dict):
        """Publish state machine configuration."""
        msg = String()
        msg.data = json.dumps(config)
        self.sm_config_pub.publish(msg)

    def publish_matrix_command(self, pattern: str, red: int = 255, green: int = 255, blue: int = 255):
        """Publish matrix pattern command."""
        msg = String()
        msg.data = json.dumps({'pattern': pattern, 'red': red, 'green': green, 'blue': blue})
        self.matrix_pub.publish(msg)

    def publish_heading_command(self, heading: int):
        """Publish heading command."""
        msg = String()
        msg.data = json.dumps({'heading': heading})
        self.heading_pub.publish(msg)

    def publish_speed_command(self, speed: int, heading: int = 0):
        """Publish speed command."""
        msg = String()
        msg.data = json.dumps({'speed': speed, 'heading': heading})
        self.speed_pub.publish(msg)

    def publish_raw_motor_command(self, left_mode: str, left_speed: int, right_mode: str, right_speed: int):
        """Publish raw motor command."""
        msg = String()
        msg.data = json.dumps({
            'left_mode': left_mode,
            'left_speed': left_speed,
            'right_mode': right_mode,
            'right_speed': right_speed
        })
        self.raw_motor_pub.publish(msg)

    def publish_spin_command(self, angle: int, duration: float):
        """Publish spin command."""
        msg = String()
        msg.data = json.dumps({'angle': angle, 'duration': duration})
        self.spin_pub.publish(msg)

    def publish_reset_aim_command(self):
        """Publish reset aim command."""
        msg = String()
        msg.data = '{}'
        self.reset_aim_pub.publish(msg)

    # ===== Status Methods =====

    def get_status(self) -> Dict[str, Any]:
        """Get comprehensive status for this Sphero instance."""
        return {
            'sphero_name': self.sphero_name,
            'websocket_port': self.websocket_port,
            'controller_ready': self.controller_ready,
            'connected': (time.time() - self.last_message_time) < self.connection_timeout,
            'battery': self.latest_battery,
            'state': self.latest_state,
            'status': self.latest_status,
            'task_status': self.latest_task_status,
            'sm_status': self.latest_sm_status
        }


def create_flask_app(node: SpheroInstanceWebSocketServer):
    """Create Flask app with WebSocket support."""
    # Get the package directory for templates and static files
    import os
    import sys

    # Find the package directory in site-packages
    package_name = 'sphero_instance_controller'
    package_dir = None

    for path in sys.path:
        candidate = os.path.join(path, package_name)
        if os.path.isdir(candidate):
            templates_path = os.path.join(candidate, 'templates')
            if os.path.isdir(templates_path):
                package_dir = candidate
                break

    if package_dir is None:
        # Fallback to current file directory
        package_dir = os.path.dirname(os.path.abspath(__file__))

    template_dir = os.path.join(package_dir, 'templates')
    static_dir = os.path.join(package_dir, 'static')

    app = Flask(__name__, template_folder=template_dir, static_folder=static_dir)
    app.config['SECRET_KEY'] = 'sphero_secret_key'
    CORS(app)

    # Use threading async_mode to support emitting from ROS callback threads
    # Configure ping/pong timeouts to prevent disconnections when browser tabs are inactive
    socketio = SocketIO(
        app,
        cors_allowed_origins="*",
        async_mode='threading',
        ping_timeout=60,        # Client must respond within 60 seconds (vs 5s default)
        ping_interval=25,       # Server sends ping every 25 seconds
        engineio_logger=False,  # Reduce logging noise
        logger=False            # Reduce logging noise
    )
    node.socketio = socketio

    # HTTP routes

    @app.route('/')
    def index():
        """Serve the controller interface."""
        from flask import render_template
        return render_template('index.html', sphero_name=node.sphero_name, websocket_port=node.websocket_port)

    # REST API routes

    @app.route('/api/status', methods=['GET'])
    def api_status():
        """Get current status."""
        from flask import jsonify
        return jsonify(node.get_status())

    @app.route('/api/led', methods=['POST'])
    def api_led():
        """Set LED color."""
        from flask import request, jsonify
        data = request.get_json()
        node.publish_led_command(
            data.get('red', 0),
            data.get('green', 0),
            data.get('blue', 0),
            data.get('led', data.get('type', 'main'))  # Accept both 'led' and 'type'
        )
        return jsonify({'status': 'success'})

    @app.route('/api/matrix', methods=['POST'])
    def api_matrix():
        """Set LED matrix pattern."""
        from flask import request, jsonify
        data = request.get_json()
        node.publish_matrix_command(
            data.get('pattern', ''),
            data.get('red', 255),
            data.get('green', 255),
            data.get('blue', 255)
        )
        return jsonify({'status': 'success'})

    @app.route('/api/matrix/clear', methods=['POST'])
    def api_matrix_clear():
        """Clear LED matrix."""
        from flask import jsonify
        node.publish_matrix_command('', 0, 0, 0)
        return jsonify({'status': 'success'})

    @app.route('/api/motion/heading', methods=['POST'])
    def api_heading():
        """Set heading."""
        from flask import request, jsonify
        data = request.get_json()
        node.publish_heading_command(data.get('heading', 0))
        return jsonify({'status': 'success'})

    @app.route('/api/motion/speed', methods=['POST'])
    def api_speed():
        """Set speed."""
        from flask import request, jsonify
        data = request.get_json()
        node.publish_speed_command(data.get('speed', 0), data.get('heading', 0))
        return jsonify({'status': 'success'})

    @app.route('/api/motion/stop', methods=['POST'])
    def api_stop():
        """Stop motion."""
        from flask import jsonify
        node.publish_stop_command()
        return jsonify({'status': 'success'})

    @app.route('/api/motion/reset', methods=['POST'])
    def api_reset():
        """Reset aim."""
        from flask import jsonify
        node.publish_reset_aim_command()
        return jsonify({'status': 'success'})

    @app.route('/api/motion/roll', methods=['POST'])
    def api_roll():
        """Roll command."""
        from flask import request, jsonify
        data = request.get_json()
        node.publish_roll_command(data.get('heading', 0), data.get('speed', 0))
        return jsonify({'status': 'success'})

    @app.route('/api/motion/raw_motor', methods=['POST'])
    def api_raw_motor():
        """Raw motor control command."""
        from flask import request, jsonify
        data = request.get_json()
        node.publish_raw_motor_command(
            data.get('left_mode', 'forward'),
            data.get('left_speed', 0),
            data.get('right_mode', 'forward'),
            data.get('right_speed', 0)
        )
        return jsonify({'status': 'success'})

    @app.route('/api/task', methods=['POST'])
    def api_task():
        """Submit a task."""
        from flask import request, jsonify
        data = request.get_json()
        node.publish_task_command(data)
        return jsonify({'status': 'success'})

    # WebSocket event handlers

    @socketio.on('connect')
    def handle_connect():
        """Handle client connection."""
        node.get_logger().info(f'WebSocket client connected to {node.sphero_name}')
        # Send initial data to newly connected client
        emit('status_update', node.get_status())
        if node.latest_state:
            emit('state_update', node.latest_state)
        if node.latest_sensors:
            emit('sensor_update', node.latest_sensors)
        if node.latest_battery:
            emit('battery_update', node.latest_battery)
        if node.latest_task_status:
            emit('task_status_update', node.latest_task_status)

    @socketio.on('disconnect')
    def handle_disconnect():
        """Handle client disconnection."""
        node.get_logger().info(f'WebSocket client disconnected from {node.sphero_name}')

    @socketio.on('start_controllers')
    def handle_start_controllers():
        """Handle start controllers request."""
        success = node.start_controllers()
        emit('controllers_started', {'success': success})

    @socketio.on('stop_controllers')
    def handle_stop_controllers():
        """Handle stop controllers request."""
        node.stop_controllers()
        emit('controllers_stopped', {})

    @socketio.on('led_command')
    def handle_led_command(data):
        """Handle LED color command."""
        node.publish_led_command(
            data['red'],
            data['green'],
            data['blue'],
            data.get('type', 'main')
        )

    @socketio.on('roll_command')
    def handle_roll_command(data):
        """Handle roll movement command."""
        node.publish_roll_command(
            data['heading'],
            data['speed'],
            data.get('duration', 0)
        )

    @socketio.on('stop_command')
    def handle_stop_command():
        """Handle stop command."""
        node.publish_stop_command()

    @socketio.on('task_command')
    def handle_task_command(data):
        """Handle task command."""
        node.publish_task_command(data)

    @socketio.on('sm_config')
    def handle_sm_config(data):
        """Handle state machine configuration."""
        node.publish_sm_config(data)

    @socketio.on('matrix_command')
    def handle_matrix_command(data):
        """Handle matrix pattern command."""
        node.publish_matrix_command(
            data.get('pattern', ''),
            data.get('red', 255),
            data.get('green', 255),
            data.get('blue', 255)
        )

    @socketio.on('heading_command')
    def handle_heading_command(data):
        """Handle heading command."""
        node.publish_heading_command(data['heading'])

    @socketio.on('speed_command')
    def handle_speed_command(data):
        """Handle speed command."""
        node.publish_speed_command(data['speed'], data.get('heading', 0))

    @socketio.on('raw_motor_command')
    def handle_raw_motor_command(data):
        """Handle raw motor command."""
        node.publish_raw_motor_command(
            data.get('left_mode', 'forward'),
            data.get('left_speed', 0),
            data.get('right_mode', 'forward'),
            data.get('right_speed', 0)
        )

    @socketio.on('spin_command')
    def handle_spin_command(data):
        """Handle spin command."""
        node.publish_spin_command(data['angle'], data.get('duration', 1.0))

    @socketio.on('reset_aim_command')
    def handle_reset_aim():
        """Handle reset aim command."""
        node.publish_reset_aim_command()

    @socketio.on('get_status')
    def handle_get_status():
        """Handle get status request."""
        emit('status', node.get_status())

    return app, socketio


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    # Get parameters from command line or environment
    import sys
    if len(sys.argv) < 3:
        print("Usage: sphero_instance_websocket_server <sphero_name> <port> [external_localization]")
        print("Example: sphero_instance_websocket_server SB-3660 5001")
        print("Example: sphero_instance_websocket_server SB-3660 5001 true")
        sys.exit(1)

    sphero_name = sys.argv[1]
    websocket_port = int(sys.argv[2])
    external_localization = False
    if len(sys.argv) >= 4:
        external_localization = sys.argv[3].lower() in ['true', '1', 'yes']

    # Create node
    node = SpheroInstanceWebSocketServer(sphero_name, websocket_port, external_localization)

    # Create Flask app with WebSocket
    app, socketio = create_flask_app(node)

    # Run ROS2 node in a separate thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # Auto-start controllers after a brief delay to let ROS node initialize
    def auto_start_controllers():
        time.sleep(1)  # Give ROS node time to initialize
        node.get_logger().info(f'Auto-starting controllers for {sphero_name}...')
        node.start_controllers()

    controller_thread = threading.Thread(target=auto_start_controllers, daemon=True)
    controller_thread.start()

    # Signal handler for clean shutdown
    def signal_handler(sig, frame):
        import os
        print(f"\nShutting down WebSocket server for {sphero_name}...")

        # Stop controllers first
        node.stop_controllers()

        # Shutdown ROS
        node.destroy_node()
        rclpy.shutdown()

        # Force exit to ensure clean shutdown
        print(f"WebSocket server for {sphero_name} shutdown complete")
        os._exit(0)

    # Register signal handlers for both SIGINT and SIGTERM
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Configure logging - suppress werkzeug (Flask) HTTP request logs
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)  # Only show errors, not routine GET/POST requests

    # Run Flask-SocketIO server
    print(f"Starting WebSocket server for {sphero_name} on port {websocket_port}...")
    try:
        socketio.run(app, host='0.0.0.0', port=websocket_port, debug=False, use_reloader=False, allow_unsafe_werkzeug=True)
    except (KeyboardInterrupt, SystemExit):
        print(f"WebSocket server for {sphero_name} interrupted")
        signal_handler(signal.SIGTERM, None)


if __name__ == '__main__':
    main()

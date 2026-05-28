#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Multi-Robot Web Application.

This is a standalone Flask application (not a ROS2 node) that manages
multiple Sphero instances through their individual WebSocket servers.
"""

import json
import subprocess
import threading
import time
import signal
import sys
from typing import Dict, List, Optional
from pathlib import Path

from flask import Flask, render_template, request, jsonify
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point
from sphero_instance_controller.msg import SpheroSensor
from multirobot_msgs.msg import FleetRobot, FleetState


class FleetNode(Node):
    """Publishes /sphero_fleet/robots with per-Sphero telemetry."""

    def __init__(self):
        super().__init__('sphero_fleet_node')
        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.publisher = self.create_publisher(FleetState, '/sphero_fleet/robots', latched)
        # robots[name] = {'name_safe', 'status', 'added_at', 'last_seen',
        #                 'battery', 'x', 'y', 'heading', 'sensor_sub'}
        self.robots: Dict[str, Dict] = {}
        self.aruco_slam_running = False
        self._lock = threading.Lock()
        self.timer = self.create_timer(1.0, self._publish_fleet_state)

    def add_robot(self, name: str, name_safe: str):
        with self._lock:
            if name in self.robots:
                return
            sub = self.create_subscription(
                SpheroSensor,
                f'/sphero/{name_safe}/sensors',
                lambda msg, n=name: self._on_sensor(n, msg),
                10,
            )
            self.robots[name] = {
                'name_safe': name_safe,
                'status': 'running',
                'added_at': time.time(),
                'last_seen': 0.0,
                'battery': 0,
                'x': 0.0,
                'y': 0.0,
                'heading': 0,
                'sensor_sub': sub,
            }
        self._publish_fleet_state()

    def remove_robot(self, name: str):
        with self._lock:
            entry = self.robots.pop(name, None)
        if entry is not None:
            self.destroy_subscription(entry['sensor_sub'])
            self._publish_fleet_state()

    def set_robot_status(self, name: str, status: str):
        with self._lock:
            if name in self.robots:
                self.robots[name]['status'] = status

    def set_aruco_slam_running(self, running: bool):
        self.aruco_slam_running = running

    def _on_sensor(self, name: str, msg: SpheroSensor):
        with self._lock:
            entry = self.robots.get(name)
            if entry is None:
                return
            entry['last_seen'] = time.time()
            entry['battery'] = int(msg.battery_percentage)
            entry['x'] = float(msg.x)
            entry['y'] = float(msg.y)
            entry['heading'] = int(msg.yaw)

    def _publish_fleet_state(self):
        msg = FleetState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.aruco_slam_running = self.aruco_slam_running
        with self._lock:
            for name, entry in self.robots.items():
                robot = FleetRobot()
                robot.name = name
                robot.name_safe = entry['name_safe']
                robot.status = entry['status']
                robot.battery_percentage = entry['battery']
                robot.pose = Point(x=entry['x'], y=entry['y'], z=0.0)
                robot.heading = entry['heading']
                robot.added_at = entry['added_at']
                robot.last_seen = entry['last_seen']
                msg.robots.append(robot)
        self.publisher.publish(msg)


class SpheroInstanceManager:
    """Manages multiple Sphero instances and their WebSocket servers."""

    def __init__(self, fleet_node: Optional[FleetNode] = None):
        """Initialize the instance manager."""
        self.fleet_node = fleet_node
        self.instances: Dict[str, Dict] = {}
        # Format: {
        #     'SB-3660': {
        #         'name': 'SB-3660',
        #         'port': 5001,
        #         'process': subprocess.Popen,
        #         'status': 'running'|'stopped',
        #         'added_at': timestamp
        #     }
        # }
        self.next_port = 5001  # Starting port for WebSocket servers
        self.aruco_slam_process: Optional[subprocess.Popen] = None
        self.aruco_slam_enabled = False
        self.foxglove_bridge_process: Optional[subprocess.Popen] = None

    def add_sphero(self, sphero_name: str) -> Dict:
        """
        Add a new Sphero instance and launch its WebSocket server.

        Args:
            sphero_name: Name of the Sphero (e.g., 'SB-3660')

        Returns:
            Dictionary with instance information
        """
        if sphero_name in self.instances:
            return {
                'success': False,
                'message': f'Sphero {sphero_name} already exists',
                'instance': self.instances[sphero_name]
            }

        try:
            # Assign port
            port = self.next_port
            self.next_port += 1

            # Launch WebSocket server for this instance
            print(f"➕ Adding {sphero_name} on port {port}...")
            # Inherit stdout/stderr to see debug logs from WebSocket server and controllers
            cmd = [
                'ros2', 'run', 'sphero_instance_controller',
                'sphero_instance_websocket_server.py',
                sphero_name,
                str(port)
            ]

            # Add external_localization parameter if ArUco SLAM is enabled
            if self.aruco_slam_enabled:
                cmd.append('true')
                print(f"   External localization enabled for {sphero_name}")

            process = subprocess.Popen(cmd)

            # Store instance info
            instance_info = {
                'name': sphero_name,
                'port': port,
                'process': process,
                'status': 'starting',
                'added_at': time.time(),
                'url': f'http://localhost:{port}'
            }

            self.instances[sphero_name] = instance_info

            # Wait a bit for server to start
            time.sleep(2)

            # Check if process is still running
            if process.poll() is None:
                instance_info['status'] = 'running'
                print(f"✓ {sphero_name} added successfully on port {port}")
                if self.fleet_node is not None:
                    self.fleet_node.add_robot(sphero_name, sphero_name.replace('-', '_'))
                return {
                    'success': True,
                    'message': f'Sphero {sphero_name} added successfully',
                    'instance': {k: v for k, v in instance_info.items() if k != 'process'}
                }
            else:
                # Process died
                del self.instances[sphero_name]
                print(f"✗ Failed to start {sphero_name} - process died")
                return {
                    'success': False,
                    'message': f'Failed to start WebSocket server for {sphero_name}',
                    'instance': None
                }

        except Exception as e:
            print(f"✗ Error adding {sphero_name}: {e}")
            return {
                'success': False,
                'message': f'Error: {str(e)}',
                'instance': None
            }

    def remove_sphero(self, sphero_name: str) -> Dict:
        """
        Remove a Sphero instance and stop its WebSocket server.

        Args:
            sphero_name: Name of the Sphero

        Returns:
            Dictionary with result
        """
        if sphero_name not in self.instances:
            return {
                'success': False,
                'message': f'Sphero {sphero_name} not found'
            }

        try:
            instance = self.instances[sphero_name]
            process = instance['process']

            # Terminate the WebSocket server process gracefully
            print(f"➖ Removing {sphero_name}...")
            process.terminate()

            try:
                # Give it more time (10 seconds) to cleanly shut down all controllers
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                print(f"   ⚠️  Timeout - forcing shutdown of {sphero_name}")
                process.kill()
                process.wait()

            # Remove from instances
            del self.instances[sphero_name]
            if self.fleet_node is not None:
                self.fleet_node.remove_robot(sphero_name)

            print(f"✓ {sphero_name} removed")
            return {
                'success': True,
                'message': f'Sphero {sphero_name} removed successfully'
            }

        except Exception as e:
            print(f"✗ Error removing {sphero_name}: {e}")
            return {
                'success': False,
                'message': f'Error: {str(e)}'
            }

    def get_all_instances(self) -> List[Dict]:
        """
        Get information about all Sphero instances.

        Returns:
            List of instance dictionaries
        """
        result = []
        for name, instance in self.instances.items():
            # Check if process is still running
            if instance['process'].poll() is None:
                status = 'running'
            else:
                status = 'stopped'
                instance['status'] = status

            result.append({
                'name': instance['name'],
                'port': instance['port'],
                'status': status,
                'added_at': instance['added_at'],
                'url': instance['url']
            })

        return result

    def get_instance(self, sphero_name: str) -> Optional[Dict]:
        """Get information about a specific Sphero instance."""
        if sphero_name not in self.instances:
            return None

        instance = self.instances[sphero_name]

        # Check if process is still running
        if instance['process'].poll() is None:
            status = 'running'
        else:
            status = 'stopped'
            instance['status'] = status

        return {
            'name': instance['name'],
            'port': instance['port'],
            'status': status,
            'added_at': instance['added_at'],
            'url': instance['url']
        }

    def start_aruco_slam(self, camera_id: int = 0) -> Dict:
        """
        Start the ArUco SLAM node for external localization.

        Args:
            camera_id: Camera ID to use

        Returns:
            Dictionary with result
        """
        if self.aruco_slam_process is not None:
            return {
                'success': False,
                'message': 'ArUco SLAM is already running'
            }

        try:
            print(f"Starting ArUco SLAM node with camera {camera_id}...")
            self.aruco_slam_process = subprocess.Popen([
                'ros2', 'run', 'aruco_slam', 'aruco_slam_node.py',
                '--ros-args', '-p', f'camera_id:={camera_id}'
            ], stdout=None, stderr=None)

            time.sleep(2)

            if self.aruco_slam_process.poll() is None:
                self.aruco_slam_enabled = True
                if self.fleet_node is not None:
                    self.fleet_node.set_aruco_slam_running(True)
                print(f"ArUco SLAM node started successfully")
                return {
                    'success': True,
                    'message': f'ArUco SLAM started with camera {camera_id}'
                }
            else:
                self.aruco_slam_process = None
                print(f"Failed to start ArUco SLAM node")
                return {
                    'success': False,
                    'message': 'ArUco SLAM process died on startup'
                }

        except Exception as e:
            print(f"Error starting ArUco SLAM: {e}")
            return {
                'success': False,
                'message': f'Error: {str(e)}'
            }

    def stop_aruco_slam(self) -> Dict:
        """
        Stop the ArUco SLAM node.

        Returns:
            Dictionary with result
        """
        if self.aruco_slam_process is None:
            return {
                'success': False,
                'message': 'ArUco SLAM is not running'
            }

        try:
            print("Stopping ArUco SLAM node...")
            self.aruco_slam_process.terminate()

            try:
                self.aruco_slam_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print("   ArUco SLAM did not terminate, killing...")
                self.aruco_slam_process.kill()
                self.aruco_slam_process.wait()

            self.aruco_slam_process = None
            self.aruco_slam_enabled = False
            if self.fleet_node is not None:
                self.fleet_node.set_aruco_slam_running(False)
            print("ArUco SLAM node stopped")
            return {
                'success': True,
                'message': 'ArUco SLAM stopped successfully'
            }

        except Exception as e:
            print(f"Error stopping ArUco SLAM: {e}")
            return {
                'success': False,
                'message': f'Error: {str(e)}'
            }

    def is_aruco_slam_running(self) -> bool:
        """Check if ArUco SLAM is currently running."""
        if self.aruco_slam_process is None:
            return False
        return self.aruco_slam_process.poll() is None

    def start_foxglove_bridge(self) -> Dict:
        """Start the foxglove_bridge for Foxglove Studio monitoring."""
        if self.foxglove_bridge_process is not None and self.foxglove_bridge_process.poll() is None:
            return {'success': False, 'message': 'foxglove_bridge is already running'}

        print("Starting foxglove_bridge on port 8765...")
        self.foxglove_bridge_process = subprocess.Popen([
            'ros2', 'launch', 'multirobot_webserver', 'foxglove_bridge.launch.py',
        ])

        time.sleep(1)
        if self.foxglove_bridge_process.poll() is None:
            print("foxglove_bridge started")
            return {'success': True, 'message': 'foxglove_bridge started on ws://0.0.0.0:8765'}

        self.foxglove_bridge_process = None
        return {'success': False, 'message': 'foxglove_bridge died on startup'}

    def stop_foxglove_bridge(self):
        """Stop the foxglove_bridge subprocess."""
        if self.foxglove_bridge_process is None:
            return
        print("Stopping foxglove_bridge...")
        self.foxglove_bridge_process.terminate()
        try:
            self.foxglove_bridge_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.foxglove_bridge_process.kill()
            self.foxglove_bridge_process.wait()
        self.foxglove_bridge_process = None

    def shutdown_all(self):
        """Shutdown all Sphero instances and ArUco SLAM."""
        print("Shutting down all Sphero instances...")
        for name in list(self.instances.keys()):
            self.remove_sphero(name)

        if self.aruco_slam_process is not None:
            print("Shutting down ArUco SLAM...")
            self.stop_aruco_slam()

        if self.foxglove_bridge_process is not None:
            self.stop_foxglove_bridge()


# Create Flask app
app = Flask(__name__,
            template_folder=str(Path(get_package_share_directory('multirobot_webserver')) / 'templates'),
            static_folder=str(Path(get_package_share_directory('multirobot_webserver')) / 'static'))

# Create instance manager (fleet_node attached in main() after rclpy.init)
manager = SpheroInstanceManager()


# Routes

@app.route('/')
def index():
    """Serve the main multi-robot interface."""
    return render_template('index.html')


@app.route('/api/spheros', methods=['GET'])
def get_spheros():
    """Get list of all Sphero instances."""
    instances = manager.get_all_instances()
    return jsonify({
        'success': True,
        'spheros': instances,
        'count': len(instances)
    })


@app.route('/api/spheros/<sphero_name>', methods=['GET'])
def get_sphero(sphero_name):
    """Get information about a specific Sphero."""
    instance = manager.get_instance(sphero_name)
    if instance:
        return jsonify({
            'success': True,
            'sphero': instance
        })
    else:
        return jsonify({
            'success': False,
            'message': f'Sphero {sphero_name} not found'
        }), 404


@app.route('/api/spheros', methods=['POST'])
def add_sphero():
    """Add a new Sphero instance."""
    data = request.get_json()

    if not data or 'sphero_name' not in data:
        return jsonify({
            'success': False,
            'message': 'Missing sphero_name in request'
        }), 400

    sphero_name = data['sphero_name']
    result = manager.add_sphero(sphero_name)

    if result['success']:
        return jsonify(result), 201
    else:
        return jsonify(result), 400


@app.route('/api/spheros/<sphero_name>', methods=['DELETE'])
def remove_sphero(sphero_name):
    """Remove a Sphero instance."""
    result = manager.remove_sphero(sphero_name)

    if result['success']:
        return jsonify(result), 200
    else:
        return jsonify(result), 404


@app.route('/health', methods=['GET'])
def health():
    """Health check endpoint."""
    return jsonify({
        'status': 'healthy',
        'sphero_count': len(manager.instances),
        'aruco_slam_running': manager.is_aruco_slam_running()
    })


@app.route('/api/aruco_slam/start', methods=['POST'])
def start_aruco_slam():
    """Start ArUco SLAM node."""
    data = request.get_json() or {}
    camera_id = data.get('camera_id', 0)
    result = manager.start_aruco_slam(camera_id)

    if result['success']:
        return jsonify(result), 200
    else:
        return jsonify(result), 400


@app.route('/api/aruco_slam/stop', methods=['POST'])
def stop_aruco_slam():
    """Stop ArUco SLAM node."""
    result = manager.stop_aruco_slam()

    if result['success']:
        return jsonify(result), 200
    else:
        return jsonify(result), 400


@app.route('/api/aruco_slam/status', methods=['GET'])
def aruco_slam_status():
    """Get ArUco SLAM status."""
    return jsonify({
        'success': True,
        'running': manager.is_aruco_slam_running(),
        'enabled': manager.aruco_slam_enabled
    })


_ros_executor: Optional[MultiThreadedExecutor] = None


def signal_handler(sig, frame):
    """Handle shutdown signal."""
    print("\nShutting down multi-robot web server...")
    manager.shutdown_all()
    if _ros_executor is not None:
        _ros_executor.shutdown()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)


def main():
    """Main entry point."""
    global _ros_executor
    import logging

    # Configure logging - suppress werkzeug (Flask) HTTP request logs
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)  # Only show errors, not routine GET/POST requests

    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)

    # Bring up ROS and the fleet node
    rclpy.init()
    fleet_node = FleetNode()
    manager.fleet_node = fleet_node
    _ros_executor = MultiThreadedExecutor()
    _ros_executor.add_node(fleet_node)
    threading.Thread(target=_ros_executor.spin, daemon=True).start()

    # Prompt for ArUco SLAM startup
    print("="*60)
    print("Multi-Robot Sphero Web Server")
    print("="*60)

    response = input("Start ArUco SLAM node for external localization? (y/N): ").strip().lower()
    if response == 'y':
        camera_id_input = input("Enter camera ID (default: 0): ").strip()
        camera_id = 0
        if camera_id_input:
            try:
                camera_id = int(camera_id_input)
            except ValueError:
                print(f"Invalid camera ID '{camera_id_input}', using default: 0")
                camera_id = 0

        result = manager.start_aruco_slam(camera_id)
        if result['success']:
            print(f"ArUco SLAM started: {result['message']}")
        else:
            print(f"Failed to start ArUco SLAM: {result['message']}")
    else:
        print("ArUco SLAM will not be started (can be started later via API)")

    # Start foxglove_bridge for Foxglove Studio monitoring
    fg_result = manager.start_foxglove_bridge()
    print(fg_result['message'])

    # Run Flask app
    print("-"*60)
    print("Starting server on http://localhost:5000")
    print("Foxglove Studio: connect to ws://<host>:8765")
    print("Press Ctrl+C to shutdown")
    print("="*60)

    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Multi-Robot Web Application.

This is a standalone Flask application (not a ROS2 node) that manages
multiple Sphero instances through their individual WebSocket servers.
"""

import json
import subprocess
import time
import signal
import sys
from typing import Dict, List, Optional
from pathlib import Path

from flask import Flask, render_template, request, jsonify
from ament_index_python.packages import get_package_share_directory


class SpheroInstanceManager:
    """Manages multiple Sphero instances and their WebSocket servers."""

    def __init__(self):
        """Initialize the instance manager."""
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
            process = subprocess.Popen([
                'ros2', 'run', 'sphero_instance_controller',
                'sphero_instance_websocket_server.py',
                sphero_name,
                str(port)
            ])

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

    def shutdown_all(self):
        """Shutdown all Sphero instances."""
        print("Shutting down all Sphero instances...")
        for name in list(self.instances.keys()):
            self.remove_sphero(name)


# Create Flask app
app = Flask(__name__,
            template_folder=str(Path(get_package_share_directory('multirobot_webserver')) / 'templates'),
            static_folder=str(Path(get_package_share_directory('multirobot_webserver')) / 'static'))

# Create instance manager
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
        'sphero_count': len(manager.instances)
    })


def signal_handler(sig, frame):
    """Handle shutdown signal."""
    print("\nShutting down multi-robot web server...")
    manager.shutdown_all()
    sys.exit(0)


def main():
    """Main entry point."""
    import logging

    # Configure logging - suppress werkzeug (Flask) HTTP request logs
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)  # Only show errors, not routine GET/POST requests

    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)

    # Run Flask app
    print("="*60)
    print("Multi-Robot Sphero Web Server")
    print("="*60)
    print("Starting server on http://localhost:5000")
    print("Press Ctrl+C to shutdown")
    print("="*60)

    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)


if __name__ == '__main__':
    main()

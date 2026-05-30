#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Multi-Robot Web Application.

This is a standalone Flask application (not a ROS2 node) that manages
multiple Sphero instances through their individual WebSocket servers.
"""

import json
import os
import socket
import subprocess
import threading
import time
import signal
import sys
from typing import Dict, List, Optional
from pathlib import Path

import requests
from flask import Flask, render_template, request, jsonify
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point, PoseStamped
from sphero_instance_controller.msg import SpheroSensor
from multirobot_msgs.msg import FleetRobot, FleetState

from multirobot_webserver.worker_registry import WorkerRegistry

# UWB tag pool: 16 tags, ids 1-16.
UWB_TAG_IDS = list(range(1, 17))

# Marker pool: 16 LED-matrix markers (8 hues x {filled, ring}). Allocation
# order per the Interface Contract: filled across all 8 hues first (slots 0-7),
# then ring across all 8 hues (slots 8-15). Maximizes hue diversity for small
# fleets. (hue, fill) tuples; index in the list is the slot number.
MARKER_HUE_ORDER = [
    'Red', 'Orange', 'Yellow', 'Green', 'Cyan', 'Blue', 'Magenta', 'Purple',
]
MARKER_FILLS = ['filled', 'ring']
MARKER_POOL = [(hue, fill) for fill in MARKER_FILLS for hue in MARKER_HUE_ORDER]

# Positioning sources that publish the shared /localization/<name>/position
# contract. Exactly one may be active at a time (single-active-publisher rule).
POSITIONING_SOURCES = ('aruco', 'matrix', 'uwb')
# Selectable sources including 'none' (no localization running / camera free).
# 'none' is ordered first since it is the default.
ALL_SOURCES = ('none',) + POSITIONING_SOURCES

# Worker launcher-agent HTTP client tuning. (connect, read) seconds — every
# agent call MUST pass this so a dead/slow Pi can never hang the webserver.
AGENT_TIMEOUT = (3, 5)
# Liveness cache TTL (seconds): remote-instance status is derived from a cached
# per-worker GET /status so per-request listing can't stall on a dead agent.
AGENT_STATUS_TTL = 2.0


class TcpRelay:
    """
    A blind TCP relay: listens on a coordinator-side port and forwards every
    connection to a single remote (worker host, port). Remote Sphero WebSocket
    servers run on the worker Pi, but browsers only ever reach the coordinator;
    this relay bridges the gap so the CONSOLE link works through one host.

    Bytes are forwarded verbatim in both directions, so HTTP and the WebSocket
    upgrade both pass through unchanged. One accept thread plus two pump threads
    per connection; daemon threads, so they never block process exit.
    """

    def __init__(self, listen_port: int, target_host: str, target_port: int):
        self.listen_port = listen_port
        self.target_host = target_host
        self.target_port = target_port
        self._server: Optional[socket.socket] = None
        self._accept_thread: Optional[threading.Thread] = None
        self._running = False

    def start(self):
        """Bind, listen, and spawn the accept loop. Raises on bind failure."""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.listen_port))
        server.listen(16)
        self._server = server
        self._running = True
        self._accept_thread = threading.Thread(
            target=self._accept_loop, daemon=True,
        )
        self._accept_thread.start()

    def _accept_loop(self):
        while self._running:
            try:
                client, _ = self._server.accept()
            except OSError:
                break  # listen socket closed by stop()
            threading.Thread(
                target=self._handle, args=(client,), daemon=True,
            ).start()

    def _handle(self, client: socket.socket):
        try:
            upstream = socket.create_connection(
                (self.target_host, self.target_port), timeout=5,
            )
        except OSError:
            client.close()
            return
        threading.Thread(
            target=self._pump, args=(client, upstream), daemon=True,
        ).start()
        self._pump(upstream, client)

    @staticmethod
    def _pump(src: socket.socket, dst: socket.socket):
        try:
            while True:
                data = src.recv(65536)
                if not data:
                    break
                dst.sendall(data)
        except OSError:
            pass
        finally:
            for sock in (src, dst):
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    sock.close()
                except OSError:
                    pass

    def stop(self):
        """Stop accepting and close the listen socket. Idempotent."""
        if not self._running:
            return
        self._running = False
        if self._server is not None:
            try:
                self._server.close()
            except OSError:
                pass
            self._server = None


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
        # QoS for /localization position subscriptions: depth-10 VOLATILE to
        # match the position publishers' default PoseStamped profile.
        self._uwb_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        # robots[name] = {'name_safe', 'status', 'added_at', 'last_seen',
        #                 'battery', 'x', 'y', 'heading', 'tag_id',
        #                 'sensor_sub', 'pos_sub'}
        self.robots: Dict[str, Dict] = {}
        # tag_id -> sphero name (live assignments)
        self.tag_assignments_map: Dict[int, str] = {}
        # sphero name -> marker slot index (live assignments)
        self.marker_assignments_map: Dict[str, int] = {}
        self.aruco_slam_running = False
        self._lock = threading.Lock()
        self.timer = self.create_timer(1.0, self._publish_fleet_state)

    def add_robot(self, name: str, name_safe: str, tag_id: int = 0,
                  marker_slot: int = -1):
        with self._lock:
            if name in self.robots:
                return
            sub = self.create_subscription(
                SpheroSensor,
                f'/sphero/{name_safe}/sensors',
                lambda msg, n=name: self._on_sensor(n, msg),
                10,
            )
            # Shared localization position contract: subscribe by name, not
            # tag_id. Receives pose from whichever source is the active
            # publisher (aruco/matrix/uwb) — source-agnostic.
            pos_sub = self.create_subscription(
                PoseStamped,
                f'/localization/{name_safe}/position',
                lambda msg, n=name: self._on_localization(n, msg),
                self._uwb_qos,
            )
            if tag_id:
                self.tag_assignments_map[tag_id] = name
            if marker_slot >= 0:
                self.marker_assignments_map[name] = marker_slot
            self.robots[name] = {
                'name_safe': name_safe,
                'status': 'running',
                'added_at': time.time(),
                'last_seen': 0.0,
                'battery': 0,
                'x': 0.0,
                'y': 0.0,
                'heading': 0,
                'tag_id': tag_id,
                'sensor_sub': sub,
                'pos_sub': pos_sub,
            }
        self._publish_fleet_state()

    def remove_robot(self, name: str):
        with self._lock:
            entry = self.robots.pop(name, None)
            if entry is not None:
                tag_id = entry.get('tag_id', 0)
                if tag_id and self.tag_assignments_map.get(tag_id) == name:
                    del self.tag_assignments_map[tag_id]
                self.marker_assignments_map.pop(name, None)
        if entry is not None:
            self.destroy_subscription(entry['sensor_sub'])
            if entry.get('pos_sub') is not None:
                self.destroy_subscription(entry['pos_sub'])
            self._publish_fleet_state()

    def free_tag_ids(self) -> List[int]:
        """Tag ids (1-16) not currently assigned to a robot."""
        with self._lock:
            assigned = set(self.tag_assignments_map.keys())
        return [tid for tid in UWB_TAG_IDS if tid not in assigned]

    def tag_assignments(self) -> Dict[int, str]:
        """Snapshot of the live tag_id -> sphero name map."""
        with self._lock:
            return dict(self.tag_assignments_map)

    def free_marker_slots(self) -> List[int]:
        """Marker pool slot indices (0-15) not currently assigned to a robot."""
        with self._lock:
            assigned = set(self.marker_assignments_map.values())
        return [s for s in range(len(MARKER_POOL)) if s not in assigned]

    def allocate_marker_slot(self) -> int:
        """Reserve and return the next free marker slot, or -1 if pool full."""
        free = self.free_marker_slots()
        return free[0] if free else -1

    def marker_assignments(self) -> Dict[str, int]:
        """Snapshot of the live sphero name -> marker slot map."""
        with self._lock:
            return dict(self.marker_assignments_map)

    def set_robot_status(self, name: str, status: str):
        with self._lock:
            if name in self.robots:
                self.robots[name]['status'] = status

    def set_aruco_slam_running(self, running: bool):
        self.aruco_slam_running = running

    def _on_sensor(self, name: str, msg: SpheroSensor):
        # Pose (x/y) now comes from localization (_on_localization); sensor
        # supplies battery + heading only.
        with self._lock:
            entry = self.robots.get(name)
            if entry is None:
                return
            entry['last_seen'] = time.time()
            entry['battery'] = int(msg.battery_percentage)
            entry['heading'] = int(msg.yaw)

    def _on_localization(self, name: str, msg: PoseStamped):
        # Active positioning source publishes in cm; convert to meters once.
        with self._lock:
            entry = self.robots.get(name)
            if entry is None:
                return
            entry['x'] = float(msg.pose.position.x) / 100.0
            entry['y'] = float(msg.pose.position.y) / 100.0

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
                robot.tag_id = entry['tag_id']
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
        # Matrix (LED-matrix marker) positioning lifecycle
        self.matrix_slam_process: Optional[subprocess.Popen] = None
        self.matrix_camera_id = 0
        self.foxglove_bridge_process: Optional[subprocess.Popen] = None
        # UWB positioning (BLE node) lifecycle
        self.uwb_process: Optional[subprocess.Popen] = None
        self.uwb_fake_mode = False
        # Flat 8-float list (4 anchors x (x,y) in cm); None until configured.
        self.anchor_positions_cm: Optional[List[float]] = None
        self.uwb_tag_ids = list(UWB_TAG_IDS)
        # Active positioning source — the three real sources share
        # /localization/<name>/... so only one may publish at a time. 'none'
        # runs no localization (leaves the camera free). Initial value set via
        # env POSITIONING_SOURCE (launch-arg/param compatible), default 'none'.
        initial = os.environ.get('POSITIONING_SOURCE', 'none').lower()
        self.positioning_source = (
            initial if initial in ALL_SOURCES else 'none'
        )

        # Worker registry for distributed BLE workers (Phase 2). Loaded from
        # config/workers.yaml; provides least-loaded selection + capacity
        # accounting. Selection/accounting only for now -- add_sphero still
        # spawns locally (remote wiring is Phase 3). None if no config present,
        # which keeps the current single-host flow working unchanged.
        self.worker_registry = self._load_worker_registry()
        # Per-worker cached agent /status: name -> {'ts', 'online', 'data'}.
        # Populated lazily by _agent_status_cached(); TTL AGENT_STATUS_TTL.
        self._agent_status_cache: Dict[str, Dict] = {}

    def _load_worker_registry(self) -> Optional[WorkerRegistry]:
        """Load config/workers.yaml into a WorkerRegistry, or None if absent."""
        try:
            share = get_package_share_directory('multirobot_webserver')
            path = os.path.join(share, 'config', 'workers.yaml')
            if not os.path.exists(path):
                return None
            registry = WorkerRegistry.from_yaml(path)
            print(f'Loaded worker registry: '
                  f'{[w.name for w in registry.all()]}')
            return registry
        except Exception as exc:  # noqa: BLE001 - registry is optional in Ph2
            print(f'Worker registry not loaded: {exc}')
            return None

    # --- Worker launcher-agent HTTP client -----------------------------------

    def _agent_headers(self) -> Dict[str, str]:
        """Auth header for agent calls. Empty when no token (dev mode)."""
        token = self.worker_registry.agent_token if self.worker_registry else ''
        return {'Authorization': f'Bearer {token}'} if token else {}

    def _agent_spawn(self, worker, sphero_name: str, port: int,
                     external_localization: bool) -> Dict:
        """
        POST {base}/spawn on `worker`. The caller allocates the port; the agent
        does not. Returns the agent's JSON payload on a 2xx response. Raises
        RuntimeError on timeout, transport error, non-2xx, or a non-success
        payload so the caller can roll back cleanly.
        """
        url = f'{worker.base_url}/spawn'
        try:
            resp = requests.post(
                url,
                json={
                    'name': sphero_name,
                    'port': port,
                    'external_localization': external_localization,
                },
                headers=self._agent_headers(),
                timeout=AGENT_TIMEOUT,
            )
        except requests.RequestException as exc:
            raise RuntimeError(f'agent {worker.name} unreachable: {exc}')
        if resp.status_code // 100 != 2:
            raise RuntimeError(
                f'agent {worker.name} spawn failed (HTTP {resp.status_code})'
            )
        try:
            payload = resp.json()
        except ValueError:
            raise RuntimeError(f'agent {worker.name} returned non-JSON spawn')
        if not payload.get('success', True):
            raise RuntimeError(
                payload.get('message', f'agent {worker.name} spawn rejected')
            )
        return payload

    def _agent_remove(self, worker, sphero_name: str) -> bool:
        """
        DELETE {base}/spawn/<name> on `worker`. Best-effort: returns True on a
        2xx (or 404, already-gone) response, False if the agent is unreachable
        or errors. Never raises — remote teardown must not block local cleanup.
        """
        url = f'{worker.base_url}/spawn/{sphero_name}'
        try:
            resp = requests.delete(
                url, headers=self._agent_headers(), timeout=AGENT_TIMEOUT,
            )
        except requests.RequestException as exc:
            print(f"   ⚠️  agent {worker.name} unreachable on remove: {exc}")
            return False
        if resp.status_code == 404 or resp.status_code // 100 == 2:
            return True
        print(f"   ⚠️  agent {worker.name} remove HTTP {resp.status_code}")
        return False

    def _agent_status_cached(self, worker) -> Optional[Dict]:
        """
        Cached GET {base}/status for `worker` (TTL AGENT_STATUS_TTL). Returns
        the agent's JSON dict, or None if currently unreachable. Updates the
        registry's online flag as a side effect. Never raises.
        """
        now = time.time()
        cached = self._agent_status_cache.get(worker.name)
        if cached is not None and (now - cached['ts']) < AGENT_STATUS_TTL:
            return cached['data']
        data: Optional[Dict] = None
        online = False
        try:
            resp = requests.get(
                f'{worker.base_url}/status',
                headers=self._agent_headers(),
                timeout=AGENT_TIMEOUT,
            )
            if resp.status_code // 100 == 2:
                data = resp.json()
                online = True
        except (requests.RequestException, ValueError):
            data = None
            online = False
        self._agent_status_cache[worker.name] = {'ts': now, 'data': data}
        if self.worker_registry is not None:
            self.worker_registry.set_online(worker.name, online)
        return data

    def add_sphero(self, sphero_name: str, tag_id: int) -> Dict:
        """
        Add a new Sphero instance and launch its WebSocket server.

        Args:
            sphero_name: Name of the Sphero (e.g., 'SB-3660')
            tag_id: UWB tag id (1-16) that drives this Sphero's pose. Required.

        Returns:
            Dictionary with instance information
        """
        if sphero_name in self.instances:
            existing = self.instances[sphero_name]
            return {
                'success': False,
                'message': f'Sphero {sphero_name} already exists',
                'instance': {k: v for k, v in existing.items()
                             if k not in ('process', 'relay')}
            }

        # A Sphero requires a valid, free tag id to join.
        if tag_id not in self.uwb_tag_ids:
            return {
                'success': False,
                'message': f'tag_id {tag_id} out of range (1-16)',
                'instance': None
            }
        if self.fleet_node is not None and tag_id not in self.fleet_node.free_tag_ids():
            assigned_to = self.fleet_node.tag_assignments().get(tag_id, '?')
            return {
                'success': False,
                'message': f'tag_id {tag_id} already assigned to {assigned_to}',
                'instance': None
            }

        # Distributed path: when a worker registry is loaded, spawn on the
        # least-loaded remote worker instead of locally. Falls through to the
        # local subprocess path below when no registry is present.
        if self.worker_registry is not None:
            return self._add_sphero_remote(sphero_name, tag_id)

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
                print(f"✓ {sphero_name} added successfully on port {port} (tag {tag_id})")
                instance_info['tag_id'] = tag_id
                # Allocate a marker pool slot alongside the UWB tag_id so the
                # matrix source can address this robot. -1 if pool is full.
                marker_slot = -1
                if self.fleet_node is not None:
                    marker_slot = self.fleet_node.allocate_marker_slot()
                    self.fleet_node.add_robot(
                        sphero_name, sphero_name.replace('-', '_'),
                        tag_id, marker_slot,
                    )
                instance_info['marker_slot'] = marker_slot
                return {
                    'success': True,
                    'message': f'Sphero {sphero_name} added successfully',
                    'instance': {k: v for k, v in instance_info.items()
                                 if k not in ('process', 'relay')}
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

    def _add_sphero_remote(self, sphero_name: str, tag_id: int) -> Dict:
        """
        Spawn `sphero_name` on the least-loaded remote worker via its launcher
        agent. Capacity is accounted only after a confirmed spawn; any failure
        rolls back fully (no registry assignment, no instance entry, no fleet
        robot) and returns a failure dict.
        """
        try:
            worker = self.worker_registry.select_worker()
        except RuntimeError as exc:
            return {'success': False, 'message': str(exc), 'instance': None}

        # Coordinator allocates the port (the agent does not). Like the local
        # flow, this is a monotonic global counter that is NOT reclaimed on
        # failure — consistent and intentional.
        port = self.next_port
        self.next_port += 1

        print(f"➕ Adding {sphero_name} on worker {worker.name} "
              f"({worker.base_url}) port {port} tag {tag_id}...")
        try:
            payload = self._agent_spawn(
                worker, sphero_name, port,
                external_localization=self.aruco_slam_enabled,
            )
        except RuntimeError as exc:
            print(f"✗ Remote spawn of {sphero_name} on {worker.name} failed: {exc}")
            return {'success': False, 'message': str(exc), 'instance': None}

        # Spawn confirmed — account capacity now. From here on, any failure must
        # release this assignment as part of rollback.
        self.worker_registry.assign(worker)
        relay: Optional[TcpRelay] = None
        try:
            # Coordinator is the source of truth for the port (the agent echoes
            # it back, but we use the locally-allocated value). The remote
            # WebSocket server runs on the worker; we stand up a coordinator-side
            # relay on the same port that forwards to worker.host:port so the
            # browser only ever talks to the coordinator. CONSOLE links are then
            # served through this host (the front-end rewrites url's host).
            relay = TcpRelay(port, worker.host, port)
            relay.start()
            url = f'http://{worker.host}:{port}'
            marker_slot = -1
            if self.fleet_node is not None:
                marker_slot = self.fleet_node.allocate_marker_slot()
                self.fleet_node.add_robot(
                    sphero_name, sphero_name.replace('-', '_'),
                    tag_id, marker_slot,
                )
            instance_info = {
                'name': sphero_name,
                'port': port,
                'process': None,          # remote: no local subprocess
                'relay': relay,           # coordinator-side TCP relay
                'status': 'running',
                'added_at': time.time(),
                'url': url,
                'tag_id': tag_id,
                'marker_slot': marker_slot,
                'worker': worker.name,
                'host': worker.host,
            }
            self.instances[sphero_name] = instance_info
        except Exception as exc:  # noqa: BLE001 - roll back the remote spawn
            print(f"✗ Bookkeeping failed for {sphero_name}; rolling back: {exc}")
            if relay is not None:
                relay.stop()
            self._agent_remove(worker, sphero_name)
            self.worker_registry.release(worker.name)
            self.instances.pop(sphero_name, None)
            return {'success': False, 'message': f'Error: {exc}', 'instance': None}

        print(f"✓ {sphero_name} added on {worker.name} at {url} (tag {tag_id})")
        return {
            'success': True,
            'message': f'Sphero {sphero_name} added on worker {worker.name}',
            'instance': {k: v for k, v in instance_info.items()
                         if k not in ('process', 'relay')},
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
            worker_name = instance.get('worker')

            if worker_name is not None:
                # Remote instance: ask its worker agent to tear it down. Agent
                # unreachable is non-fatal — we still drop local bookkeeping so
                # the slot is reclaimed.
                print(f"➖ Removing {sphero_name} from worker {worker_name}...")
                worker = (self.worker_registry.get(worker_name)
                          if self.worker_registry is not None else None)
                if worker is not None:
                    ok = self._agent_remove(worker, sphero_name)
                    if not ok:
                        print(f"   ⚠️  {sphero_name} agent remove not confirmed; "
                              f"dropping local bookkeeping anyway")
                    self.worker_registry.release(worker_name)
                # Tear down the coordinator-side relay for this remote instance.
                relay = instance.get('relay')
                if relay is not None:
                    relay.stop()
            else:
                # Local instance: terminate the WebSocket server subprocess.
                process = instance['process']
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
            status = self._instance_status(instance)
            instance['status'] = status
            result.append({
                'name': instance['name'],
                'port': instance['port'],
                'status': status,
                'added_at': instance['added_at'],
                'url': instance['url'],
                'worker': instance.get('worker'),  # None for local instances
            })

        return result

    def _instance_status(self, instance: Dict) -> str:
        """
        Liveness for one instance. Local: subprocess poll. Remote: cached agent
        /status (so a dead Pi can't stall the listing). Remote status is
        'running' only when the agent is reachable AND reports this Sphero.
        """
        worker_name = instance.get('worker')
        if worker_name is None:
            return 'running' if instance['process'].poll() is None else 'stopped'

        worker = (self.worker_registry.get(worker_name)
                  if self.worker_registry is not None else None)
        if worker is None:
            return 'unknown'
        data = self._agent_status_cached(worker)
        if data is None:
            return 'unreachable'
        # Agent /status is expected to list its live spheros under 'spheros'
        # (list of names or list of dicts with 'name'). Be tolerant of shape.
        names = set()
        for item in (data.get('spheros') or []):
            if isinstance(item, dict):
                names.add(item.get('name'))
            else:
                names.add(item)
        if names and instance['name'] not in names:
            return 'stopped'
        return 'running'

    def workers_snapshot(self) -> Dict:
        """
        Snapshot of the worker registry for GET /api/workers. Refreshes each
        worker's online flag via cached agent /status, then reports per-worker
        state plus fleet totals. Empty (registry_loaded=False) with no config.
        """
        if self.worker_registry is None:
            return {'registry_loaded': False, 'workers': [],
                    'total_capacity': 0, 'total_count': 0}
        workers = []
        for worker in self.worker_registry.all():
            self._agent_status_cached(worker)  # refreshes online flag
            workers.append(worker.as_dict())
        return {
            'registry_loaded': True,
            'workers': workers,
            'total_capacity': self.worker_registry.total_capacity(),
            'total_count': self.worker_registry.total_count(),
        }

    def get_instance(self, sphero_name: str) -> Optional[Dict]:
        """Get information about a specific Sphero instance."""
        if sphero_name not in self.instances:
            return None

        instance = self.instances[sphero_name]
        status = self._instance_status(instance)
        instance['status'] = status

        return {
            'name': instance['name'],
            'port': instance['port'],
            'status': status,
            'added_at': instance['added_at'],
            'url': instance['url'],
            'worker': instance.get('worker'),  # None for local instances
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
                'ros2', 'run', 'aruco_slam', 'aruco_slam_node',
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

    def _marker_assignments_json(self) -> str:
        """Build the matrix node's marker_assignments JSON from the live
        name -> marker-slot map: [{"name", "hue", "fill"}, ...]."""
        entries = []
        if self.fleet_node is not None:
            for name, slot in self.fleet_node.marker_assignments().items():
                if 0 <= slot < len(MARKER_POOL):
                    hue, fill = MARKER_POOL[slot]
                    entries.append({'name': name, 'hue': hue, 'fill': fill})
        return json.dumps(entries)

    def start_matrix_slam(self, camera_id: int = 0) -> Dict:
        """
        Start the matrix-marker SLAM node for external localization.

        Mirrors start_aruco_slam; passes the live marker_assignments JSON
        (name -> hue/fill) plus camera_id to matrix_slam_node.
        """
        if self.matrix_slam_process is not None:
            return {
                'success': False,
                'message': 'Matrix SLAM is already running'
            }

        try:
            marker_json = self._marker_assignments_json()
            print(f"Starting Matrix SLAM node with camera {camera_id}...")
            self.matrix_slam_process = subprocess.Popen([
                'ros2', 'run', 'aruco_slam', 'matrix_slam_node',
                '--ros-args',
                '-p', f'camera_id:={camera_id}',
                # Single-quote so ros2's YAML parser treats the JSON as a string
                # (an unquoted '[]' / '[{...}]' parses as a list, breaking the param).
                '-p', f"marker_assignments:='{marker_json}'",
            ], stdout=None, stderr=None)

            time.sleep(2)

            if self.matrix_slam_process.poll() is None:
                self.aruco_slam_enabled = True
                self.matrix_camera_id = camera_id
                if self.fleet_node is not None:
                    self.fleet_node.set_aruco_slam_running(True)
                print("Matrix SLAM node started successfully")
                return {
                    'success': True,
                    'message': f'Matrix SLAM started with camera {camera_id}'
                }
            else:
                self.matrix_slam_process = None
                print("Failed to start Matrix SLAM node")
                return {
                    'success': False,
                    'message': 'Matrix SLAM process died on startup'
                }

        except Exception as e:
            print(f"Error starting Matrix SLAM: {e}")
            return {
                'success': False,
                'message': f'Error: {str(e)}'
            }

    def stop_matrix_slam(self) -> Dict:
        """Stop the matrix-marker SLAM node."""
        if self.matrix_slam_process is None:
            return {
                'success': False,
                'message': 'Matrix SLAM is not running'
            }

        try:
            print("Stopping Matrix SLAM node...")
            self.matrix_slam_process.terminate()

            try:
                self.matrix_slam_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print("   Matrix SLAM did not terminate, killing...")
                self.matrix_slam_process.kill()
                self.matrix_slam_process.wait()

            self.matrix_slam_process = None
            self.aruco_slam_enabled = False
            if self.fleet_node is not None:
                self.fleet_node.set_aruco_slam_running(False)
            print("Matrix SLAM node stopped")
            return {
                'success': True,
                'message': 'Matrix SLAM stopped successfully'
            }

        except Exception as e:
            print(f"Error stopping Matrix SLAM: {e}")
            return {
                'success': False,
                'message': f'Error: {str(e)}'
            }

    def is_matrix_slam_running(self) -> bool:
        """Check if Matrix SLAM is currently running."""
        if self.matrix_slam_process is None:
            return False
        return self.matrix_slam_process.poll() is None

    def set_positioning_source(self, source: str) -> Dict:
        """
        Select the active positioning source
        ('none' | 'aruco' | 'matrix' | 'uwb').

        The three real sources publish the shared /localization/<name>/position
        topic, so exactly one may run at a time. Starts the chosen source and
        stops the other two (single-active-publisher rule). 'none' stops all
        sources and starts nothing (leaves the camera free).
        """
        source = (source or '').lower()
        if source not in ALL_SOURCES:
            return {
                'success': False,
                'message': f"source must be one of {list(ALL_SOURCES)}"
            }

        # Stop the non-selected sources first so nothing else publishes.
        # For 'none' none of the guards match, so all three are stopped.
        if source != 'aruco' and self.is_aruco_slam_running():
            self.stop_aruco_slam()
        if source != 'matrix' and self.is_matrix_slam_running():
            self.stop_matrix_slam()
        if source != 'uwb' and self.is_uwb_running():
            self.stop_uwb()

        # Start the selected source (if not already running).
        if source == 'none':
            result = {'success': True, 'message': 'No positioning source active'}
        elif source == 'aruco':
            result = (self.start_aruco_slam(self.matrix_camera_id)
                      if not self.is_aruco_slam_running()
                      else {'success': True, 'message': 'ArUco already running'})
        elif source == 'matrix':
            result = (self.start_matrix_slam(self.matrix_camera_id)
                      if not self.is_matrix_slam_running()
                      else {'success': True, 'message': 'Matrix already running'})
        else:  # uwb
            result = (self.start_uwb(self.uwb_fake_mode)
                      if not self.is_uwb_running()
                      else {'success': True, 'message': 'UWB already running'})

        if result['success']:
            self.positioning_source = source
        return result

    def set_anchor_positions(self, coords: List[Dict]) -> Dict:
        """
        Store the 4-anchor coordinate map (A0..A3, x/y in cm).

        Args:
            coords: list of exactly 4 {'x', 'y'} dicts, in cm.

        Returns:
            Dictionary with result.
        """
        if not isinstance(coords, list) or len(coords) != 4:
            return {'success': False, 'message': 'expected exactly 4 anchors'}

        try:
            flat: List[float] = []
            for a in coords:
                flat.append(float(a['x']))
                flat.append(float(a['y']))
        except (KeyError, TypeError, ValueError):
            return {'success': False, 'message': 'each anchor needs numeric x and y'}

        self.anchor_positions_cm = flat
        if self.is_uwb_running():
            return {'success': True, 'message': 'Stored; stop/start positioning to apply'}
        return {'success': True, 'message': 'Anchor map stored'}

    def start_uwb(self, fake_mode: bool = False) -> Dict:
        """Start the BLE positioning node as a child process (Foxglove-style)."""
        if self.is_uwb_running():
            return {'success': False, 'message': 'UWB positioning already running'}
        if self.anchor_positions_cm is None:
            return {'success': False, 'message': 'anchors not configured'}

        anchors_arg = '[' + ','.join(str(v) for v in self.anchor_positions_cm) + ']'
        tag_ids_arg = '[' + ','.join(str(t) for t in self.uwb_tag_ids) + ']'
        # tag_names aligned 1:1 with tag_ids so the UWB node can publish the
        # shared /localization/<name>/position contract (name-keyed). Empty
        # string for unassigned tags.
        assignments = (
            self.fleet_node.tag_assignments() if self.fleet_node is not None else {}
        )
        tag_names = [assignments.get(t, '') for t in self.uwb_tag_ids]
        tag_names_arg = '[' + ','.join(f'"{n}"' for n in tag_names) + ']'
        cmd = [
            'ros2', 'run', 'sphero_uwb_positioning', 'ble_position_node',
            '--ros-args',
            '-p', f'anchor_positions_cm:={anchors_arg}',
            '-p', f'tag_ids:={tag_ids_arg}',
            '-p', f'tag_names:={tag_names_arg}',
        ]
        if fake_mode:
            cmd += ['-p', 'fake_mode:=true']

        print(f"Starting UWB positioning node (fake_mode={fake_mode})...")
        self.uwb_process = subprocess.Popen(cmd)
        self.uwb_fake_mode = fake_mode

        time.sleep(2)
        if self.uwb_process.poll() is None:
            print("UWB positioning node started")
            return {'success': True, 'message': 'UWB positioning started'}

        self.uwb_process = None
        self.uwb_fake_mode = False
        return {'success': False, 'message': 'UWB positioning process died on startup'}

    def stop_uwb(self) -> Dict:
        """Stop the BLE positioning node."""
        if not self.is_uwb_running():
            return {'success': False, 'message': 'UWB positioning is not running'}

        print("Stopping UWB positioning node...")
        self.uwb_process.terminate()
        try:
            self.uwb_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            print("   UWB positioning did not terminate, killing...")
            self.uwb_process.kill()
            self.uwb_process.wait()
        self.uwb_process = None
        self.uwb_fake_mode = False
        print("UWB positioning node stopped")
        return {'success': True, 'message': 'UWB positioning stopped'}

    def is_uwb_running(self) -> bool:
        """Check if the UWB positioning node is currently running."""
        if self.uwb_process is None:
            return False
        return self.uwb_process.poll() is None

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

        if self.matrix_slam_process is not None:
            print("Shutting down Matrix SLAM...")
            self.stop_matrix_slam()

        if self.uwb_process is not None:
            print("Shutting down UWB positioning...")
            self.stop_uwb()

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


@app.route('/api/workers', methods=['GET'])
def get_workers():
    """Worker registry snapshot (per-worker capacity/count/free/online + totals)."""
    return jsonify({'success': True, **manager.workers_snapshot()})


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
    """Add a new Sphero instance. Requires a valid, free tag_id (1-16)."""
    data = request.get_json()

    if not data or 'sphero_name' not in data:
        return jsonify({
            'success': False,
            'message': 'Missing sphero_name in request'
        }), 400

    if 'tag_id' not in data:
        return jsonify({
            'success': False,
            'message': 'Missing tag_id in request'
        }), 400

    try:
        tag_id = int(data['tag_id'])
    except (TypeError, ValueError):
        return jsonify({
            'success': False,
            'message': 'tag_id must be an integer (1-16)'
        }), 400

    sphero_name = data['sphero_name']
    result = manager.add_sphero(sphero_name, tag_id)

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


@app.route('/api/uwb/tags', methods=['GET'])
def uwb_tags():
    """Report the UWB tag pool: all ids, free ids, and current assignments."""
    if manager.fleet_node is None:
        free = list(UWB_TAG_IDS)
        assigned: Dict[str, str] = {}
    else:
        free = manager.fleet_node.free_tag_ids()
        assigned = {str(k): v for k, v in manager.fleet_node.tag_assignments().items()}
    return jsonify({
        'success': True,
        'all': list(UWB_TAG_IDS),
        'free': free,
        'assigned': assigned,
    })


@app.route('/api/markers', methods=['GET'])
def markers():
    """Report the LED-matrix marker pool: all slots, free slots, and current
    assignments (mirrors GET /api/uwb/tags)."""
    pool = [{'slot': i, 'hue': hue, 'fill': fill}
            for i, (hue, fill) in enumerate(MARKER_POOL)]
    if manager.fleet_node is None:
        free = list(range(len(MARKER_POOL)))
        assigned: Dict[str, int] = {}
    else:
        free = manager.fleet_node.free_marker_slots()
        assigned = manager.fleet_node.marker_assignments()
    return jsonify({
        'success': True,
        'all': pool,
        'free': free,
        'assigned': assigned,
    })


@app.route('/api/positioning_source', methods=['GET'])
def get_positioning_source():
    """Report the active positioning source and per-source running state."""
    return jsonify({
        'success': True,
        'source': manager.positioning_source,
        'sources': list(ALL_SOURCES),
        'running': {
            'aruco': manager.is_aruco_slam_running(),
            'matrix': manager.is_matrix_slam_running(),
            'uwb': manager.is_uwb_running(),
        },
    })


@app.route('/api/positioning_source', methods=['POST'])
def set_positioning_source():
    """Select the active positioning source (starts it, stops the others)."""
    data = request.get_json() or {}
    source = data.get('source')
    result = manager.set_positioning_source(source)
    if result['success']:
        return jsonify({**result, 'source': manager.positioning_source}), 200
    return jsonify(result), 400


@app.route('/api/uwb/anchors', methods=['GET'])
def get_uwb_anchors():
    """Return the currently stored anchor map (in cm), or null if unset."""
    flat = manager.anchor_positions_cm
    if flat is None:
        return jsonify({'success': True, 'anchors_cm': None, 'configured': False})
    anchors = [{'x': flat[i], 'y': flat[i + 1]} for i in range(0, len(flat), 2)]
    return jsonify({'success': True, 'anchors_cm': anchors, 'configured': True})


@app.route('/api/uwb/anchors', methods=['POST'])
def set_uwb_anchors():
    """Store the 4-anchor coordinate map (A0..A3, x/y in cm)."""
    data = request.get_json() or {}
    coords = data.get('anchors_cm')
    result = manager.set_anchor_positions(coords)

    if result['success']:
        return jsonify(result), 200
    else:
        return jsonify(result), 400


@app.route('/api/uwb/start', methods=['POST'])
def start_uwb():
    """Start the UWB positioning node (requires anchors configured)."""
    data = request.get_json() or {}
    fake_mode = bool(data.get('fake_mode', False))
    result = manager.start_uwb(fake_mode)

    if result['success']:
        return jsonify(result), 200
    else:
        return jsonify(result), 400


@app.route('/api/uwb/stop', methods=['POST'])
def stop_uwb():
    """Stop the UWB positioning node."""
    result = manager.stop_uwb()

    if result['success']:
        return jsonify(result), 200
    else:
        return jsonify(result), 400


@app.route('/api/uwb/status', methods=['GET'])
def uwb_status():
    """Get UWB positioning status (for polling)."""
    assigned_count = (
        len(manager.fleet_node.tag_assignments())
        if manager.fleet_node is not None else 0
    )
    return jsonify({
        'success': True,
        'running': manager.is_uwb_running(),
        'anchors_configured': manager.anchor_positions_cm is not None,
        'fake_mode': manager.uwb_fake_mode,
        'assigned_count': assigned_count,
    })


_ros_executor: Optional[MultiThreadedExecutor] = None
_shutting_down = False


def signal_handler(sig, frame):
    """Handle shutdown signal (idempotent — re-entrant SIGINT from ros2 launch is ignored)."""
    global _shutting_down
    if _shutting_down:
        return
    _shutting_down = True
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

    # Bring up the configured positioning source automatically (default
    # 'matrix', overridable via POSITIONING_SOURCE env / launch). The source
    # can be changed at runtime via the web UI / REST API. UWB needs anchors
    # configured first, so it is left for the operator to start via the API.
    source = manager.positioning_source
    print(f"Positioning source: {source}")
    if source == 'none':
        print("No positioning source — start one from the UI/API.")
    elif source == 'uwb':
        print("UWB selected — configure anchors then start via API/UI.")
    else:
        result = manager.set_positioning_source(source)
        if result['success']:
            print(f"{source} positioning started: {result['message']}")
        else:
            print(f"Failed to start {source} positioning: {result['message']}")

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

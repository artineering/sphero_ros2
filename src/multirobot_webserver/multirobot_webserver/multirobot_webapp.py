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
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Dict, List, Optional
from pathlib import Path

import requests
from flask import Flask, render_template, request, jsonify
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point, PoseStamped
from sphero_instance_controller.msg import SpheroSensor
from multirobot_msgs.msg import FleetRobot, FleetState

from multirobot_webserver.worker_registry import WorkerRegistry

# Marker pool: 16 LED-matrix markers (8 hues x {filled, ring}). Allocation
# order per the Interface Contract: filled across all 8 hues first (slots 0-7),
# then ring across all 8 hues (slots 8-15). Maximizes hue diversity for small
# fleets. (hue, fill) tuples; index in the list is the slot number.
MARKER_HUE_ORDER = [
    'Red', 'Orange', 'Yellow', 'Green', 'Cyan', 'Blue', 'Magenta', 'Purple',
]
MARKER_FILLS = ['filled', 'ring']
MARKER_POOL = [(hue, fill) for fill in MARKER_FILLS for hue in MARKER_HUE_ORDER]

# Worker launcher-agent HTTP client tuning. (connect, read) seconds — every
# agent call MUST pass this so a dead/slow Pi can never hang the webserver.
AGENT_TIMEOUT = (3, 5)
# Liveness cache TTL (seconds): remote-instance status is derived from a cached
# per-worker GET /status so per-request listing can't stall on a dead agent.
AGENT_STATUS_TTL = 2.0
# Local-instance health cache. Each local instance exposes GET /api/status with
# real BLE telemetry (controller_ready, connected); we cache it so the /spheros
# listing can't stall on a slow/unanswering instance. (connect, read) timeout
# keeps a hung instance from blocking the request.
INSTANCE_HEALTH_TTL = 2.0
INSTANCE_HEALTH_TIMEOUT = (2, 3)
# Grace window (seconds) after spawn during which a local instance with no BLE
# telemetry yet reads 'connecting' rather than 'failed'. Covers BLE scan +
# connect + first heartbeat.
INSTANCE_CONNECT_WINDOW = 45.0
# Heartbeat freshness (seconds): a Sphero reads 'running' only if FleetNode
# received telemetry (a /sphero/<name>/status heartbeat or a sensor message)
# within this window. The device controller's heartbeat timer + sensor stream
# only fire while the BLE link is live, so a stale last_seen means the link is
# down -- the real liveness signal, uniform for local AND remote instances.
HEARTBEAT_FRESH_SEC = 15.0

# Broadcast fan-out tuning. (connect, read) per-POST timeout so a dead/slow unit
# can never block the others; worker pool capped at the fleet ceiling (16 units /
# channel tags). Fan-out is parallel, so worst-case wall time is one read timeout.
BROADCAST_POST_TIMEOUT = (3, 5)
BROADCAST_MAX_WORKERS = 16

# Robot-to-robot IR channel pairs, one (near, far) per broadcaster. Max 4
# broadcasters; the server assigns pairs in payload order (source of truth).
IR_CHANNEL_PAIRS = [(0, 1), (2, 3), (4, 5), (6, 7)]


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
        self._loc_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        # robots[name] = {'name_safe', 'status', 'added_at', 'last_seen',
        #                 'battery', 'x', 'y', 'heading',
        #                 'sensor_sub', 'pos_sub'}
        self.robots: Dict[str, Dict] = {}
        # sphero name -> marker slot index (live assignments)
        self.marker_assignments_map: Dict[str, int] = {}
        self._lock = threading.Lock()
        self.timer = self.create_timer(1.0, self._publish_fleet_state)

    def add_robot(self, name: str, name_safe: str, marker_slot: int = -1):
        with self._lock:
            if name in self.robots:
                return
            sub = self.create_subscription(
                SpheroSensor,
                f'/sphero/{name_safe}/sensors',
                lambda msg, n=name: self._on_sensor(n, msg),
                10,
            )
            # Shared localization position contract: subscribe by name_safe.
            # Receives pose from the active localization publisher (the overhead
            # Kinect) — source-agnostic.
            pos_sub = self.create_subscription(
                PoseStamped,
                f'/localization/{name_safe}/position',
                lambda msg, n=name: self._on_localization(n, msg),
                self._loc_qos,
            )
            # Heartbeat: the device controller publishes a /status JSON on a
            # timer while the BLE link is live. Refreshing last_seen from it
            # makes connection liveness robust even when sensors aren't streaming
            # -- and works for remote instances too (topic crosses the graph).
            status_sub = self.create_subscription(
                String,
                f'/sphero/{name_safe}/status',
                lambda msg, n=name: self._on_heartbeat(n),
                10,
            )
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
                'sensor_sub': sub,
                'pos_sub': pos_sub,
                'status_sub': status_sub,
            }
        self._publish_fleet_state()

    def remove_robot(self, name: str):
        with self._lock:
            entry = self.robots.pop(name, None)
            if entry is not None:
                self.marker_assignments_map.pop(name, None)
        if entry is not None:
            self.destroy_subscription(entry['sensor_sub'])
            if entry.get('pos_sub') is not None:
                self.destroy_subscription(entry['pos_sub'])
            if entry.get('status_sub') is not None:
                self.destroy_subscription(entry['status_sub'])
            self._publish_fleet_state()

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

    def _on_heartbeat(self, name: str):
        """A /status heartbeat arrived -> the BLE link is live; refresh last_seen."""
        with self._lock:
            entry = self.robots.get(name)
            if entry is not None:
                entry['last_seen'] = time.time()

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
        # External localization (set_external_location on spawned instances) is
        # on by default: the overhead Kinect (kinect_field_tracking, launched
        # outside the webserver) publishes /localization/<name>/position for all
        # units. Override with EXTERNAL_LOCALIZATION=0/false to disable.
        self.external_localization_enabled = (
            os.environ.get('EXTERNAL_LOCALIZATION', 'true').lower()
            not in ('0', 'false', 'no')
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
        # Per-local-instance cached /api/status: name -> {'ts', 'data'}.
        # Populated lazily by _instance_health_cached(); TTL INSTANCE_HEALTH_TTL.
        self._instance_health_cache: Dict[str, Dict] = {}

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

    def _instance_health_cached(self, instance: Dict) -> Optional[Dict]:
        """
        Cached GET {instance.url}/api/status for a LOCAL instance (TTL
        INSTANCE_HEALTH_TTL). Returns the instance's status dict (with
        'controller_ready' and 'connected' BLE telemetry), or None if the
        instance isn't answering yet / is unreachable. Mirrors the
        _agent_status_cached TTL+caching pattern so a slow instance can't stall
        the /spheros listing. Never raises.
        """
        name = instance['name']
        now = time.time()
        cached = self._instance_health_cache.get(name)
        if cached is not None and (now - cached['ts']) < INSTANCE_HEALTH_TTL:
            return cached['data']
        data: Optional[Dict] = None
        try:
            resp = requests.get(
                f"{instance['url']}/api/status",
                timeout=INSTANCE_HEALTH_TIMEOUT,
            )
            if resp.status_code // 100 == 2:
                data = resp.json()
        except (requests.RequestException, ValueError):
            data = None
        self._instance_health_cache[name] = {'ts': now, 'data': data}
        return data

    def add_sphero(self, sphero_name: str) -> Dict:
        """
        Add a new Sphero instance and launch its WebSocket server.

        Args:
            sphero_name: Name of the Sphero (e.g., 'SB-3660')

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

        # Distributed path: when a worker registry is loaded, spawn on the
        # least-loaded remote worker instead of locally. Falls through to the
        # local subprocess path below when no registry is present.
        if self.worker_registry is not None:
            return self._add_sphero_remote(sphero_name)

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

            # Enable external_localization by default: the overhead Kinect
            # publishes /localization/<name>/position for all units. Overridable
            # via the EXTERNAL_LOCALIZATION env flag.
            if self.external_localization_enabled:
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

            # Check if process is still running. The shell being alive only
            # means the websocket server launched -- the BLE link comes later,
            # so seed 'connecting'. _instance_status() promotes to 'running'
            # once /api/status reports a live BLE connection.
            if process.poll() is None:
                instance_info['status'] = 'connecting'
                print(f"✓ {sphero_name} added successfully on port {port}")
                # Allocate a marker pool slot for this robot. -1 if pool is full.
                marker_slot = -1
                if self.fleet_node is not None:
                    marker_slot = self.fleet_node.allocate_marker_slot()
                    self.fleet_node.add_robot(
                        sphero_name, sphero_name.replace('-', '_'),
                        marker_slot,
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

    def _add_sphero_remote(self, sphero_name: str) -> Dict:
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
              f"({worker.base_url}) port {port}...")
        try:
            payload = self._agent_spawn(
                worker, sphero_name, port,
                external_localization=self.external_localization_enabled,
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
                    marker_slot,
                )
            instance_info = {
                'name': sphero_name,
                'port': port,
                'process': None,          # remote: no local subprocess
                'relay': relay,           # coordinator-side TCP relay
                'status': 'running',
                'added_at': time.time(),
                'url': url,
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

        print(f"✓ {sphero_name} added on {worker.name} at {url}")
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

    def add_spheros_batch(self, names: List[str]) -> Dict:
        """
        Deploy several Spheros in one call by callsign. Per-item results carry
        the assigned port on success or a failure reason.

        Args:
            names: Raw callsigns; normalized here (strip, drop blanks, dedupe
                preserving first-seen order).

        Returns: {success, deployed, failed, results:[{name, success,
                  port?, reason?}]}
        """
        seen = set()
        ordered: List[str] = []
        for raw in names:
            name = str(raw).strip()
            if not name or name in seen:
                continue
            seen.add(name)
            ordered.append(name)

        results: List[Dict] = []
        deployed = 0
        for name in ordered:
            if name in self.instances:
                results.append({'name': name, 'success': False,
                                'reason': f'{name} already deployed'})
                continue
            res = self.add_sphero(name)
            if res.get('success'):
                deployed += 1
                inst = res.get('instance') or {}
                results.append({
                    'name': name,
                    'success': True,
                    'port': inst.get('port'),
                })
            else:
                results.append({'name': name, 'success': False,
                                'reason': res.get('message', 'deploy failed')})

        return {
            'success': True,
            'deployed': deployed,
            'failed': len(results) - deployed,
            'results': results,
        }

    def remove_spheros_batch(self, names: List[str]) -> Dict:
        """
        Detach several Spheros in one call. Thin loop over remove_sphero;
        per-item results mirror add_spheros_batch.

        Returns: {success, removed, failed, results:[{name, success, reason?}]}
        """
        results: List[Dict] = []
        removed = 0
        for raw in names:
            name = str(raw).strip()
            if not name:
                continue
            res = self.remove_sphero(name)
            if res.get('success'):
                removed += 1
                results.append({'name': name, 'success': True})
            else:
                results.append({'name': name, 'success': False,
                                'reason': res.get('message', 'detach failed')})

        return {
            'success': True,
            'removed': removed,
            'failed': len(results) - removed,
            'results': results,
        }

    def broadcast_task(self, task_core: dict,
                       start_offset: float) -> Dict:
        """
        Fan ONE task (or one concurrent bundle) to every running instance in
        parallel. `task_core` is the task body sans timing: either a single
        `{task_type, parameters}` or a bundle `{tasks:[...]}`. A single
        coordinator timestamp `now` is stamped ONCE before dispatch; the SAME
        payload (incl. `now` and `start_offset`) goes to every unit so
        NTP-synced controllers fire together at `now + start_offset`. Per-POST
        timeout keeps a dead unit from blocking the others.

        Returns: {success, sent, failed, now, start_offset, start_time,
                  results:[{name, success, error?}]}
        """
        # Snapshot running targets so a concurrent add/remove can't mutate the
        # instance dict mid-fan-out.
        targets = [(inst['name'], inst['url'])
                   for inst in self.instances.values()
                   if self._instance_status(inst) == 'running']
        if not targets:
            return {'success': False, 'message': 'No units deployed'}

        # Stamp the coordinator clock ONCE; every unit receives this same value.
        now = time.time()
        payload = {
            **task_core,
            'now': now,
            'start_offset': start_offset,
        }

        def _post_one(name: str, url: str) -> Dict:
            try:
                r = requests.post(f'{url}/api/task', json=payload,
                                  timeout=BROADCAST_POST_TIMEOUT)
                if 200 <= r.status_code < 300:
                    return {'name': name, 'success': True}
                return {'name': name, 'success': False,
                        'error': f'HTTP {r.status_code}'}
            except requests.RequestException as exc:
                return {'name': name, 'success': False, 'error': str(exc)}

        results: List[Dict] = []
        with ThreadPoolExecutor(
                max_workers=min(BROADCAST_MAX_WORKERS, len(targets))) as ex:
            futures = [ex.submit(_post_one, name, url) for name, url in targets]
            # No timeout on as_completed: the per-POST timeout bounds each worker,
            # so we always collect every result while they ran concurrently.
            for fut in as_completed(futures):
                results.append(fut.result())

        sent = sum(1 for r in results if r['success'])
        return {
            'success': True,
            'sent': sent,
            'failed': len(results) - sent,
            'now': now,
            'start_offset': start_offset,
            'start_time': now + start_offset,
            'results': results,
        }

    def _post_task_to_unit(self, name: str, url: str, payload: dict) -> Dict:
        """POST one task payload to a single unit's /api/task. Per-POST timeout
        keeps a dead unit from blocking the fan-out. Returns {name, success,
        error?}."""
        try:
            r = requests.post(f'{url}/api/task', json=payload,
                              timeout=BROADCAST_POST_TIMEOUT)
            if 200 <= r.status_code < 300:
                return {'name': name, 'success': True}
            return {'name': name, 'success': False,
                    'error': f'HTTP {r.status_code}'}
        except requests.RequestException as exc:
            return {'name': name, 'success': False, 'error': str(exc)}

    def dispatch_ir_formation(self, broadcasters: list) -> Dict:
        """
        Fan DIFFERENT robot-to-robot IR tasks to DIFFERENT running units.

        `broadcasters` is a list (max 4) of
        `{name, followers:[...], evaders:[...]}`. Each broadcaster i is assigned
        IR_CHANNEL_PAIRS[i] = (near, far) in payload order (server is the source
        of truth for channel assignment). The broadcaster gets `ir_broadcast`,
        its followers get `ir_follow`, its evaders get `ir_evade`, all carrying
        that broadcaster's (near, far). Each unit receives its OWN bare
        `{task_type, parameters}` (no now/start_offset — IR is a continuous
        toggle, not a synced one-shot).

        Validation is all-or-nothing: nothing is dispatched unless every entry
        passes. Returns {success, dispatched, failed, results:[{name, role,
        task_type, success, error?}]} or {success: False, message}.
        """
        if not isinstance(broadcasters, list) or not broadcasters:
            return {'success': False,
                    'message': 'broadcasters must be a non-empty list'}
        if len(broadcasters) > len(IR_CHANNEL_PAIRS):
            return {'success': False,
                    'message': f'max {len(IR_CHANNEL_PAIRS)} broadcasters'}

        # Snapshot running targets so a concurrent add/remove can't mutate the
        # instance dict mid-fan-out. Map name -> url for per-unit dispatch.
        running = {inst['name']: inst['url']
                   for inst in self.instances.values()
                   if self._instance_status(inst) == 'running'}
        if not running:
            return {'success': False, 'message': 'No units deployed'}

        # --- Validation pass (assign roles into a plan; dispatch nothing yet).
        # plan: name -> (role, task_type, near, far)
        plan: Dict[str, tuple] = {}
        seen_broadcasters = set()
        for entry in broadcasters:
            if not isinstance(entry, dict):
                return {'success': False,
                        'message': 'each broadcaster must be an object'}
            bname = entry.get('name')
            if not isinstance(bname, str) or not bname.strip():
                return {'success': False,
                        'message': 'broadcaster name must be a non-empty '
                                   'string'}
            bname = bname.strip()
            if bname in seen_broadcasters:
                return {'success': False,
                        'message': f'{bname} listed as broadcaster twice'}
            if bname not in running:
                return {'success': False,
                        'message': f'{bname} is not a running unit'}
            seen_broadcasters.add(bname)

        # Channel pairs assigned in payload order (matches client display order).
        for i, entry in enumerate(broadcasters):
            bname = entry['name'].strip()
            near, far = IR_CHANNEL_PAIRS[i]
            # A unit can't be both broadcaster and follower/evader.
            if bname in plan:
                return {'success': False,
                        'message': f'{bname} is assigned conflicting roles'}
            plan[bname] = ('broadcaster', 'ir_broadcast', near, far)

            followers = entry.get('followers', []) or []
            evaders = entry.get('evaders', []) or []
            if not isinstance(followers, list) or not isinstance(evaders, list):
                return {'success': False,
                        'message': 'followers and evaders must be lists'}

            for role, task_type, members in (
                    ('follower', 'ir_follow', followers),
                    ('evader', 'ir_evade', evaders)):
                for member in members:
                    if not isinstance(member, str) or not member.strip():
                        return {'success': False,
                                'message': f'{role} name must be a non-empty '
                                           'string'}
                    member = member.strip()
                    if member not in running:
                        return {'success': False,
                                'message': f'{member} is not a running unit'}
                    if member in seen_broadcasters:
                        return {'success': False,
                                'message': f'{member} is both a broadcaster '
                                           'and a follower/evader'}
                    if member in plan:
                        return {'success': False,
                                'message': f'{member} is assigned to more than '
                                           'one broadcaster/role'}
                    plan[member] = (role, task_type, near, far)

        # --- Dispatch: each unit gets its OWN payload (no shared now/offset).
        def _dispatch(name: str, role: str, task_type: str,
                      near: int, far: int) -> Dict:
            payload = {'task_type': task_type,
                       'parameters': {'near': near, 'far': far}}
            res = self._post_task_to_unit(name, running[name], payload)
            res['role'] = role
            res['task_type'] = task_type
            return res

        results: List[Dict] = []
        with ThreadPoolExecutor(
                max_workers=min(BROADCAST_MAX_WORKERS, len(plan))) as ex:
            futures = [ex.submit(_dispatch, name, role, task_type, near, far)
                       for name, (role, task_type, near, far) in plan.items()]
            for fut in as_completed(futures):
                results.append(fut.result())

        dispatched = sum(1 for r in results if r['success'])
        return {
            'success': True,
            'dispatched': dispatched,
            'failed': len(results) - dispatched,
            'results': results,
        }

    def stop_all_ir(self) -> Dict:
        """
        Clear all robot-to-robot IR behavior fleet-wide. Each running unit gets
        the three IR stop tasks (`ir_broadcast_stop`, `ir_follow_stop`,
        `ir_evade_stop`); a unit not running a given behavior treats its stop as
        a harmless no-op (per task contract). Units are stopped in parallel; the
        three stops per unit are sent sequentially. A unit is `success` only if
        all three stop POSTs returned 2xx, else it carries the first error.

        Returns {success, results:[{name, success, error?}]}.
        """
        targets = [(inst['name'], inst['url'])
                   for inst in self.instances.values()
                   if self._instance_status(inst) == 'running']
        if not targets:
            return {'success': True, 'results': []}

        stop_types = ('ir_broadcast_stop', 'ir_follow_stop', 'ir_evade_stop')

        def _stop_unit(name: str, url: str) -> Dict:
            for task_type in stop_types:
                res = self._post_task_to_unit(
                    name, url, {'task_type': task_type, 'parameters': {}})
                if not res['success']:
                    return {'name': name, 'success': False,
                            'error': res.get('error', 'stop failed')}
            return {'name': name, 'success': True}

        results: List[Dict] = []
        with ThreadPoolExecutor(
                max_workers=min(BROADCAST_MAX_WORKERS, len(targets))) as ex:
            futures = [ex.submit(_stop_unit, name, url)
                       for name, url in targets]
            for fut in as_completed(futures):
                results.append(fut.result())

        return {'success': True, 'results': results}

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
        Liveness for one instance, derived from HEARTBEAT freshness.

        "Connected" truth = whether FleetNode received telemetry (a
        /sphero/<name>/status heartbeat or a sensor message) recently -- those
        only flow while the BLE link is live. This is uniform for local and
        remote (worker-agent) instances: a spawned-but-not-BLE-connected unit
        emits no heartbeat, so it never reads 'running'. The process/agent
        signals are used only to tell a still-coming-up unit
        ('connecting'/'unreachable'/'stopped') from a dead one ('failed').
        """
        name = instance['name']
        worker_name = instance.get('worker')
        now = time.time()

        # Local: a dead subprocess is definitively failed (no BLE link possible).
        if worker_name is None and instance['process'].poll() is not None:
            return 'failed'

        # Connection truth: heartbeat/telemetry freshness from FleetNode.
        last_seen = 0.0
        added_at = instance.get('added_at', now)
        if self.fleet_node is not None:
            entry = self.fleet_node.robots.get(name)
            if entry is not None:
                last_seen = entry.get('last_seen', 0.0)
                added_at = entry.get('added_at', added_at)
        if last_seen > 0.0 and (now - last_seen) <= HEARTBEAT_FRESH_SEC:
            return 'running'

        # No recent heartbeat -> not (yet) connected. Surface remote agent
        # problems if any.
        if worker_name is not None:
            worker = (self.worker_registry.get(worker_name)
                      if self.worker_registry is not None else None)
            if worker is None:
                return 'unknown'
            data = self._agent_status_cached(worker)
            if data is None:
                return 'unreachable'
            names = set()
            for item in (data.get('spheros') or []):
                names.add(item.get('name') if isinstance(item, dict) else item)
            if names and name not in names:
                return 'stopped'

        # Up at the process/agent level but no live BLE telemetry yet: still
        # connecting inside the grace window, otherwise the link is down.
        if (now - added_at) <= INSTANCE_CONNECT_WINDOW:
            return 'connecting'
        return 'failed'

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

    def shutdown_all(self):
        """Shutdown all Sphero instances."""
        print("Shutting down all Sphero instances...")
        for name in list(self.instances.keys()):
            self.remove_sphero(name)


# Create Flask app
app = Flask(__name__,
            template_folder=str(Path(get_package_share_directory('multirobot_webserver')) / 'templates'),
            static_folder=str(Path(get_package_share_directory('multirobot_webserver')) / 'static'))
# Re-read templates from disk on each render so a rebuilt index.html is picked
# up without restarting the server (Jinja otherwise caches the compiled template
# in memory for the process lifetime when debug is off).
app.config['TEMPLATES_AUTO_RELOAD'] = True
app.jinja_env.auto_reload = True

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
    """Add a new Sphero instance by callsign."""
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


@app.route('/api/spheros/batch', methods=['POST'])
def add_spheros_batch():
    """Deploy several Spheros at once by callsign.

    Request: {names: [...]}. Per-item failures are reported in `results`
    rather than as HTTP errors; 400 only if `names` is missing/not a
    list/empty after normalization.
    """
    data = request.get_json(silent=True)
    if not data or not isinstance(data.get('names'), list):
        return jsonify({
            'success': False,
            'message': 'Missing names list in request',
        }), 400

    names = [str(n).strip() for n in data['names'] if str(n).strip()]
    if not names:
        return jsonify({
            'success': False,
            'message': 'No callsigns provided',
        }), 400

    return jsonify(manager.add_spheros_batch(names)), 200


@app.route('/api/spheros/batch_delete', methods=['POST'])
def remove_spheros_batch():
    """Detach several Spheros at once.

    Request: {names: [...]}. Per-item failures are reported in `results`.
    """
    data = request.get_json(silent=True)
    if not data or not isinstance(data.get('names'), list):
        return jsonify({
            'success': False,
            'message': 'Missing names list in request',
        }), 400

    names = [str(n).strip() for n in data['names'] if str(n).strip()]
    if not names:
        return jsonify({
            'success': False,
            'message': 'No callsigns provided',
        }), 400

    return jsonify(manager.remove_spheros_batch(names)), 200


@app.route('/api/broadcast_task', methods=['POST'])
def broadcast_task():
    """Fan ONE task (or one concurrent bundle) to every running Sphero.

    Request is EITHER a single task `{task_type, parameters?, start_offset?}`
    OR a concurrent bundle `{tasks:[{task_type, parameters?}, ...],
    start_offset?}`. The manager stamps a single `now` and fans the same
    payload to each unit's /api/task. Task types are NOT whitelisted here (the
    per-instance controller is the authority on unknown types AND on bundle
    lane conflicts, matching single /api/task). 400 on bad body or zero units.
    """
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify({'success': False,
                        'message': 'Invalid JSON body'}), 400

    if 'tasks' in data:
        # Concurrent bundle: validate shape only; the per-instance controller
        # owns lane-conflict and unknown-type rejection.
        tasks = data.get('tasks')
        if not isinstance(tasks, list) or not tasks:
            return jsonify({'success': False,
                            'message': 'tasks must be a non-empty list'}), 400
        for i, item in enumerate(tasks):
            if not isinstance(item, dict):
                return jsonify({'success': False,
                                'message': f'tasks[{i}] must be an object'}), 400
            tt = item.get('task_type')
            if not isinstance(tt, str) or not tt.strip():
                return jsonify({
                    'success': False,
                    'message': f'tasks[{i}].task_type must be a '
                               'non-empty string'}), 400
        task_core = {'tasks': tasks}
    else:
        task_type = data.get('task_type')
        if not isinstance(task_type, str) or not task_type.strip():
            return jsonify({
                'success': False,
                'message': 'task_type must be a non-empty string'}), 400
        task_type = task_type.strip()

        parameters = data.get('parameters', {})
        if not isinstance(parameters, dict):
            return jsonify({'success': False,
                            'message': 'parameters must be a JSON object'}), 400
        task_core = {'task_type': task_type, 'parameters': parameters}

    start_offset = data.get('start_offset', 3.0)
    try:
        start_offset = float(start_offset)
    except (TypeError, ValueError):
        return jsonify({'success': False,
                        'message': 'start_offset must be a number'}), 400
    if start_offset < 0:
        return jsonify({'success': False,
                        'message': 'start_offset must be >= 0'}), 400

    if not manager.instances:
        return jsonify({'success': False,
                        'message': 'No units deployed'}), 400

    return jsonify(
        manager.broadcast_task(task_core, start_offset)), 200


@app.route('/api/ir_formation', methods=['POST'])
def ir_formation():
    """Dispatch a robot-to-robot IR formation across the running fleet.

    Request: `{broadcasters:[{name, followers:[...], evaders:[...]}, ...]}`
    (max 4 broadcasters). The manager assigns one IR channel pair per
    broadcaster and fans the matching ir_broadcast / ir_follow / ir_evade task
    to each unit. 400 on bad body or validation failure.
    """
    data = request.get_json(silent=True)
    if not isinstance(data, dict) or not isinstance(data.get('broadcasters'),
                                                     list):
        return jsonify({'success': False,
                        'message': 'broadcasters must be a list'}), 400

    result = manager.dispatch_ir_formation(data['broadcasters'])
    return jsonify(result), (200 if result.get('success') else 400)


@app.route('/api/ir_stop', methods=['POST'])
def ir_stop():
    """Clear all robot-to-robot IR behavior fleet-wide (no body required)."""
    return jsonify(manager.stop_all_ir()), 200


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
    })


@app.route('/api/markers', methods=['GET'])
def markers():
    """Report the LED-matrix marker pool: all slots, free slots, and current
    assignments."""
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

    print("="*60)
    print("Multi-Robot Sphero Web Server")
    print("="*60)
    # Localization is provided externally by the overhead Kinect
    # (kinect_field_tracking), launched separately from this web server.
    print(f"External localization: {manager.external_localization_enabled}")

    # Run Flask app
    print("-"*60)
    print("Starting server on http://localhost:5000")
    print("Foxglove bridge runs separately: "
          "ros2 launch multirobot_webserver foxglove_bridge.launch.py")
    print("Press Ctrl+C to shutdown")
    print("="*60)

    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)


if __name__ == '__main__':
    main()

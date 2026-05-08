#!/usr/bin/env python3
"""BLE passive-scan node that republishes UWB tag positions on ROS topics.

Tags broadcast their computed (x_cm, y_cm) position via BLE manufacturer data.
This node decodes the advertisement, maps tag_id -> sphero_name, and publishes
PoseStamped per known tag plus a per-tag liveness DiagnosticArray.
"""

import asyncio
import math
import signal
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

UWB_NAME_PREFIX = "UWB-T"
UWB_COMPANY_ID = 0xFFFF
PAYLOAD_LEN = 5  # tag_id (1) + x_cm (2) + y_cm (2) — company id is the dict key


@dataclass
class TagSample:
    tag_id: int
    x_cm: int
    y_cm: int
    rssi_dbm: int
    ble_address: str
    last_seen_monotonic: float


def _name_safe(name: str) -> str:
    return name.replace("-", "_")


class BlePositionNode(Node):
    def __init__(self) -> None:
        super().__init__("uwb_ble_position_node")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("tag_stale_timeout_s", 3.0)
        self.declare_parameter("frame_id", "sphero_arena")
        self.declare_parameter("ble_adapter", "hci0")
        self.declare_parameter("scan_active", True)
        self.declare_parameter("scan_restart_interval_s", 60.0)
        self.declare_parameter("tag_ids", [1, 2, 3, 4])
        self.declare_parameter(
            "sphero_names", ["SB-3660", "SB-74FB", "SB-3716", "SB-58EF"]
        )
        self.declare_parameter("fake_mode", False)
        self.declare_parameter("fake_tag_ids", [1, 2])

        self.publish_rate_hz: float = self.get_parameter("publish_rate_hz").value
        self.tag_stale_timeout_s: float = self.get_parameter("tag_stale_timeout_s").value
        self.frame_id: str = self.get_parameter("frame_id").value
        self.ble_adapter: str = self.get_parameter("ble_adapter").value
        self.scan_active: bool = self.get_parameter("scan_active").value
        self.scan_restart_interval_s: float = self.get_parameter(
            "scan_restart_interval_s"
        ).value
        tag_ids = list(self.get_parameter("tag_ids").value)
        sphero_names = list(self.get_parameter("sphero_names").value)
        self.fake_mode: bool = self.get_parameter("fake_mode").value
        self.fake_tag_ids = list(self.get_parameter("fake_tag_ids").value)

        if len(tag_ids) != len(sphero_names):
            raise ValueError(
                f"tag_ids ({len(tag_ids)}) and sphero_names ({len(sphero_names)}) "
                "must have the same length"
            )

        self.tag_to_sphero: dict[int, str] = {
            int(tid): str(name) for tid, name in zip(tag_ids, sphero_names)
        }

        self._lock = threading.Lock()
        self._samples: dict[int, TagSample] = {}
        self._last_published: dict[int, TagSample] = {}

        self._pose_pubs: dict[int, rclpy.publisher.Publisher] = {}
        for tid, name in self.tag_to_sphero.items():
            topic = f"sphero/{_name_safe(name)}/uwb/position"
            self._pose_pubs[tid] = self.create_publisher(PoseStamped, topic, 10)
            self.get_logger().info(f"Tag {tid} -> {name} -> {topic}")

        diag_qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.VOLATILE)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", diag_qos)

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self._publish_timer = self.create_timer(period, self._publish_callback)
        self._diag_timer = self.create_timer(1.0, self._diagnostics_callback)

        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._scanner_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        if self.fake_mode:
            self.get_logger().warning("fake_mode=true: synthesizing tag positions, no BLE scan")
            self._fake_t0 = time.monotonic()
            self._fake_timer = self.create_timer(0.1, self._fake_callback)
        else:
            self._scanner_thread = threading.Thread(
                target=self._run_scanner_loop, name="ble-scanner", daemon=True
            )
            self._scanner_thread.start()

    def _run_scanner_loop(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._scanner_main())
        finally:
            self._loop.close()

    async def _scanner_main(self) -> None:
        from bleak import BleakScanner

        scanning_mode = "active" if self.scan_active else "passive"
        adapter = self.ble_adapter or None
        self.get_logger().info(
            f"Starting BleakScanner (adapter={adapter}, mode={scanning_mode})"
        )

        while not self._stop_event.is_set():
            scanner = BleakScanner(
                detection_callback=self._detection_callback,
                adapter=adapter,
                scanning_mode=scanning_mode,
            )
            await scanner.start()
            try:
                if self.scan_restart_interval_s > 0:
                    await asyncio.sleep(self.scan_restart_interval_s)
                else:
                    while not self._stop_event.is_set():
                        await asyncio.sleep(1.0)
            finally:
                await scanner.stop()
            if self.scan_restart_interval_s > 0 and not self._stop_event.is_set():
                self.get_logger().debug("Restarting BLE scanner")

    def _detection_callback(self, device, advertisement_data) -> None:
        name = device.name or advertisement_data.local_name or ""
        if not name.startswith(UWB_NAME_PREFIX):
            return
        mfg = advertisement_data.manufacturer_data or {}
        payload = mfg.get(UWB_COMPANY_ID)
        if payload is None or len(payload) < PAYLOAD_LEN:
            return

        tag_id = payload[0]
        x_cm, y_cm = struct.unpack("<hh", payload[1:5])

        if tag_id not in self.tag_to_sphero:
            self.get_logger().debug(
                f"Ignoring unknown tag_id={tag_id} from {device.address}",
                throttle_duration_sec=5.0,
            )
            return

        sample = TagSample(
            tag_id=tag_id,
            x_cm=int(x_cm),
            y_cm=int(y_cm),
            rssi_dbm=int(advertisement_data.rssi),
            ble_address=device.address,
            last_seen_monotonic=time.monotonic(),
        )
        with self._lock:
            self._samples[tag_id] = sample

    def _fake_callback(self) -> None:
        t = time.monotonic() - self._fake_t0
        now = time.monotonic()
        with self._lock:
            for i, tid in enumerate(self.fake_tag_ids):
                if tid not in self.tag_to_sphero:
                    continue
                phase = i * (math.pi / 2.0)
                x = 150.0 + 100.0 * math.sin(0.3 * t + phase)
                y = 150.0 + 100.0 * math.cos(0.2 * t + phase)
                self._samples[tid] = TagSample(
                    tag_id=tid,
                    x_cm=int(x),
                    y_cm=int(y),
                    rssi_dbm=-50,
                    ble_address=f"FA:KE:00:00:00:{tid:02X}",
                    last_seen_monotonic=now,
                )

    def _publish_callback(self) -> None:
        with self._lock:
            samples_snapshot = dict(self._samples)
        now_mono = time.monotonic()
        ros_now = self.get_clock().now().to_msg()

        for tid in self.tag_to_sphero:
            sample = samples_snapshot.get(tid)
            if sample is not None:
                self._last_published[tid] = sample
                source = sample
            else:
                source = self._last_published.get(tid)
                if source is None:
                    continue

            pose = PoseStamped()
            pose.header.stamp = ros_now
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = float(source.x_cm)
            pose.pose.position.y = float(source.y_cm)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self._pose_pubs[tid].publish(pose)

            age = now_mono - source.last_seen_monotonic
            if age > self.tag_stale_timeout_s and sample is not None:
                self.get_logger().warning(
                    f"Tag {tid} ({self.tag_to_sphero[tid]}) is stale "
                    f"({age:.1f}s since last adv)",
                    throttle_duration_sec=5.0,
                )

    def _diagnostics_callback(self) -> None:
        with self._lock:
            samples_snapshot = dict(self._samples)
        now_mono = time.monotonic()
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        for tid, sphero_name in self.tag_to_sphero.items():
            status = DiagnosticStatus()
            status.name = f"uwb_tag_{tid}_{_name_safe(sphero_name)}"
            status.hardware_id = f"tag_{tid}"

            sample = samples_snapshot.get(tid) or self._last_published.get(tid)
            if sample is None:
                status.level = DiagnosticStatus.ERROR
                status.message = "never seen"
                status.values = [
                    KeyValue(key="tag_id", value=str(tid)),
                    KeyValue(key="sphero_name", value=sphero_name),
                ]
            else:
                age = now_mono - sample.last_seen_monotonic
                if age > self.tag_stale_timeout_s:
                    status.level = DiagnosticStatus.WARN
                    status.message = f"stale ({age:.1f}s old)"
                else:
                    status.level = DiagnosticStatus.OK
                    status.message = f"live ({age:.2f}s old)"
                status.values = [
                    KeyValue(key="tag_id", value=str(tid)),
                    KeyValue(key="sphero_name", value=sphero_name),
                    KeyValue(key="x_cm", value=str(sample.x_cm)),
                    KeyValue(key="y_cm", value=str(sample.y_cm)),
                    KeyValue(key="rssi_dbm", value=str(sample.rssi_dbm)),
                    KeyValue(key="ble_address", value=sample.ble_address),
                    KeyValue(key="age_s", value=f"{age:.3f}"),
                ]
            msg.status.append(status)
        self._diag_pub.publish(msg)

    def stop_scanner(self) -> None:
        self._stop_event.set()
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._scanner_thread is not None:
            self._scanner_thread.join(timeout=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlePositionNode()

    def _sigint(_signum, _frame):
        node.stop_scanner()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    try:
        rclpy.spin(node)
    finally:
        node.stop_scanner()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

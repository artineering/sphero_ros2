#!/usr/bin/env python3
"""
sphero_uwb_positioning_node.py
===============================
ROS2 node that:
  1. Reads UWB TWR range data from 4 WiFi-enabled anchors via UDP
  2. Performs 2D multilateration (least-squares) for each tag
  3. Maps tag IDs to Sphero names
  4. Detects arena boundary (in/out)
  5. Publishes per-Sphero PoseStamped on sphero_uwb/<sphero_name>/position
  6. Publishes per-Sphero status on sphero_uwb/<sphero_name>/status
  7. Publishes MarkerArray on /sphero_uwb/markers for RViz2 visualization
  8. Publishes diagnostics on /sphero_uwb/diagnostics

Multilateration approach:
  Given N anchor positions (x_i, y_i) and measured distances d_i,
  linearize by subtracting the last equation from all others:
    2(x_N - x_i)*x + 2(y_N - y_i)*y = (d_i² - d_N²) - (x_i² - x_N²) - (y_i² - y_N²)
  Solve the overdetermined Ax = b via least squares: x = (A^T A)^{-1} A^T b
"""

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster

# Custom message (generated at build time)
from sphero_uwb_positioning.msg import SpheroUWBStatus

# Local module
from sphero_uwb_positioning.udp_range_reader import UDPRangeReader, RangeMeasurement

# Optional: for arena boundary detection
try:
    from scipy.spatial import ConvexHull
    from shapely.geometry import Point, Polygon
    SHAPELY_AVAILABLE = True
except ImportError:
    SHAPELY_AVAILABLE = False


@dataclass
class AnchorConfig:
    """Configuration for one anchor."""
    anchor_id: int
    x: float
    y: float


@dataclass
class TagState:
    """Tracking state for one tag."""
    tag_id: int
    sphero_name: str
    # Latest range from each anchor: anchor_id -> (distance_m, timestamp)
    ranges: Dict[int, Tuple[float, float]] = field(default_factory=dict)
    # Smoothed position estimate
    x: float = 0.0
    y: float = 0.0
    initialized: bool = False
    last_update: float = 0.0


class SpheroUWBPositioningNode(Node):
    def __init__(self):
        super().__init__("sphero_uwb_positioning_node")

        # ─── Declare and read parameters ───
        self.declare_parameter("udp_port", 5000)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("range_timeout", 1.0)
        self.declare_parameter("min_anchors_for_fix", 3)
        self.declare_parameter("ema_alpha", 0.3)
        self.declare_parameter("outlier_threshold", 0.5)

        self.udp_port = self.get_parameter("udp_port").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.range_timeout = self.get_parameter("range_timeout").value
        self.min_anchors = self.get_parameter("min_anchors_for_fix").value
        self.ema_alpha = self.get_parameter("ema_alpha").value
        self.outlier_threshold = self.get_parameter("outlier_threshold").value

        # ─── Load anchor configurations ───
        self.declare_parameter("anchors", [])
        anchors_config = self.get_parameter("anchors").value

        self.anchors: List[AnchorConfig] = []
        self.anchor_map: Dict[int, AnchorConfig] = {}

        for anchor_dict in anchors_config:
            cfg = AnchorConfig(
                anchor_id=anchor_dict['id'],
                x=anchor_dict['x'],
                y=anchor_dict['y'],
            )
            self.anchors.append(cfg)
            self.anchor_map[cfg.anchor_id] = cfg
            self.get_logger().info(
                f"Anchor {cfg.anchor_id}: pos=({cfg.x}, {cfg.y})"
            )

        if len(self.anchors) < 3:
            self.get_logger().error("At least 3 anchors required for 2D positioning!")
            raise ValueError("Insufficient anchors configured")

        # ─── Load tag-to-Sphero mapping ───
        self.declare_parameter("tag_sphero_mapping", {})
        tag_mapping = self.get_parameter("tag_sphero_mapping").value

        self.tag_to_sphero: Dict[int, str] = {}
        if isinstance(tag_mapping, dict):
            for tag_id_str, sphero_name in tag_mapping.items():
                tag_id = int(tag_id_str)
                self.tag_to_sphero[tag_id] = sphero_name
                self.get_logger().info(f"Tag {tag_id} -> Sphero '{sphero_name}'")
        else:
            self.get_logger().warn("tag_sphero_mapping parameter is not a dict, no tags will be tracked")

        if not self.tag_to_sphero:
            self.get_logger().warn("No tag-to-Sphero mappings configured!")

        # ─── Initialize tag states ───
        self.tags: Dict[int, TagState] = {}
        for tag_id, sphero_name in self.tag_to_sphero.items():
            self.tags[tag_id] = TagState(tag_id=tag_id, sphero_name=sphero_name)

        # ─── Compute arena boundary ───
        self.arena_polygon = None
        if SHAPELY_AVAILABLE and len(self.anchors) >= 3:
            try:
                self._compute_arena_boundary()
                self.get_logger().info("Arena boundary computed from anchor positions")
            except Exception as e:
                self.get_logger().warn(f"Failed to compute arena boundary: {e}")
        else:
            if not SHAPELY_AVAILABLE:
                self.get_logger().warn("scipy/shapely not available, arena detection disabled")

        # ─── Shared measurement queue (thread-safe deque) ───
        self.range_queue: deque = deque(maxlen=2000)

        # ─── Start UDP reader thread ───
        self.udp_reader = UDPRangeReader(
            port=self.udp_port,
            output_queue=self.range_queue,
            max_queue_size=2000,
            logger=self.get_logger(),
        )
        self.udp_reader.start()

        # ─── QoS profiles ───
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ─── Publishers ───
        # Per-Sphero position and status
        self.position_pubs: Dict[int, rclpy.publisher.Publisher] = {}
        self.status_pubs: Dict[int, rclpy.publisher.Publisher] = {}
        for tag_id, sphero_name in self.tag_to_sphero.items():
            self.position_pubs[tag_id] = self.create_publisher(
                PoseStamped, f"sphero_uwb/{sphero_name}/position", sensor_qos
            )
            self.status_pubs[tag_id] = self.create_publisher(
                SpheroUWBStatus, f"sphero_uwb/{sphero_name}/status", 10
            )

        # RViz markers
        self.marker_pub = self.create_publisher(
            MarkerArray, "/sphero_uwb/markers", 10
        )

        # Diagnostics
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/sphero_uwb/diagnostics", 10
        )

        # ─── TF2 Static broadcaster for anchors ───
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_anchor_transforms()

        # ─── Main processing timer ───
        period = 1.0 / self.publish_rate
        self.create_timer(period, self._process_callback)

        # ─── Diagnostics timer (1 Hz) ───
        self.create_timer(1.0, self._diagnostics_callback)

        self.get_logger().info(
            f"Sphero UWB Positioning started: {len(self.anchors)} anchors, "
            f"{len(self.tags)} tags, {self.publish_rate} Hz, UDP port {self.udp_port}"
        )

    def _compute_arena_boundary(self):
        """Compute convex hull from anchor positions for boundary detection."""
        anchor_points = np.array([[a.x, a.y] for a in self.anchors])
        hull = ConvexHull(anchor_points)
        hull_points = anchor_points[hull.vertices]
        self.arena_polygon = Polygon(hull_points)

    def _is_in_arena(self, x: float, y: float) -> bool:
        """Check if point (x, y) is inside arena boundary."""
        if self.arena_polygon is None:
            return True  # Unknown, assume inside
        try:
            point = Point(x, y)
            return self.arena_polygon.contains(point)
        except:
            return True

    def _publish_anchor_transforms(self):
        """Publish static TF2 transforms for anchor positions."""
        transforms = []
        for anchor in self.anchors:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "sphero_arena"
            t.child_frame_id = f"uwb_anchor_{anchor.anchor_id}"
            t.transform.translation.x = anchor.x
            t.transform.translation.y = anchor.y
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            transforms.append(t)

        if transforms:
            self.tf_broadcaster.sendTransform(transforms)

    def _process_callback(self):
        """Main loop: drain queue, update ranges, compute positions, publish."""
        now = time.time()

        # ─── Drain measurement queue ───
        measurements_this_cycle = []
        while self.range_queue:
            try:
                m = self.range_queue.popleft()
                measurements_this_cycle.append(m)
            except IndexError:
                break

        # ─── Update per-tag range tables ───
        for m in measurements_this_cycle:
            if m.tag_id in self.tags:
                tag_state = self.tags[m.tag_id]
                tag_state.ranges[m.anchor_id] = (m.distance_m, m.timestamp)

        # ─── Compute 2D positions via multilateration ───
        markers = MarkerArray()

        for tag_id, tag_state in self.tags.items():
            # Collect fresh ranges (within timeout)
            valid_anchors = []
            valid_distances = []

            for anchor_id, (dist, ts) in tag_state.ranges.items():
                if (now - ts) < self.range_timeout:
                    if anchor_id in self.anchor_map:
                        valid_anchors.append(self.anchor_map[anchor_id])
                        valid_distances.append(dist)

            num_anchors_visible = len(valid_anchors)

            if len(valid_anchors) < self.min_anchors:
                continue  # Not enough data for this tag

            # Run multilateration
            result = self._multilaterate(valid_anchors, valid_distances)
            if result is None:
                continue

            x_raw, y_raw, residual = result

            # Outlier check
            if residual > self.outlier_threshold * len(valid_anchors):
                self.get_logger().debug(
                    f"Tag {tag_id} ({tag_state.sphero_name}): high residual {residual:.3f}, skipping"
                )
                continue

            # EMA smoothing
            if tag_state.initialized:
                tag_state.x += self.ema_alpha * (x_raw - tag_state.x)
                tag_state.y += self.ema_alpha * (y_raw - tag_state.y)
            else:
                tag_state.x = x_raw
                tag_state.y = y_raw
                tag_state.initialized = True

            tag_state.last_update = now

            # Arena boundary detection
            in_arena = self._is_in_arena(tag_state.x, tag_state.y)

            # ─── Publish PoseStamped ───
            stamp = self.get_clock().now().to_msg()

            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = "sphero_arena"
            pose_msg.pose.position.x = tag_state.x
            pose_msg.pose.position.y = tag_state.y
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            self.position_pubs[tag_id].publish(pose_msg)

            # ─── Publish SpheroUWBStatus ───
            status_msg = SpheroUWBStatus()
            status_msg.header.stamp = stamp
            status_msg.header.frame_id = "sphero_arena"
            status_msg.sphero_name = tag_state.sphero_name
            status_msg.tag_id = tag_id
            status_msg.in_arena = in_arena
            status_msg.num_anchors_visible = num_anchors_visible
            status_msg.x = tag_state.x
            status_msg.y = tag_state.y
            status_msg.quality = residual
            self.status_pubs[tag_id].publish(status_msg)

            # ─── RViz marker for tag ───
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = "sphero_arena"
            marker.ns = "sphero_uwb_tags"
            marker.id = tag_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = tag_state.x
            marker.pose.position.y = tag_state.y
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            # Color: green if in arena, red if outside
            if in_arena:
                marker.color.r = 0.2
                marker.color.g = 0.8
                marker.color.b = 0.2
            else:
                marker.color.r = 0.8
                marker.color.g = 0.2
                marker.color.b = 0.2
            marker.color.a = 1.0
            marker.lifetime.sec = 1
            markers.markers.append(marker)

            # Text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "sphero_uwb_labels"
            text_marker.id = tag_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = tag_state.x
            text_marker.pose.position.y = tag_state.y
            text_marker.pose.position.z = 0.3
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.12
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"{tag_state.sphero_name} ({tag_state.x:.2f},{tag_state.y:.2f})"
            text_marker.lifetime.sec = 1
            markers.markers.append(text_marker)

        # Add static anchor markers
        stamp = self.get_clock().now().to_msg()
        for anchor_cfg in self.anchors:
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = "sphero_arena"
            m.ns = "uwb_anchors"
            m.id = anchor_cfg.anchor_id
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = anchor_cfg.x
            m.pose.position.y = anchor_cfg.y
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.1
            m.color.r = 0.2
            m.color.g = 0.8
            m.color.b = 0.2
            m.color.a = 1.0
            m.lifetime.sec = 2
            markers.markers.append(m)

        if markers.markers:
            self.marker_pub.publish(markers)

    def _diagnostics_callback(self):
        """Publish diagnostics about anchors and tags."""
        now = time.time()
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # Per-anchor diagnostics
        for anchor_id, anchor_cfg in self.anchor_map.items():
            status = DiagnosticStatus()
            status.name = f"UWB Anchor {anchor_id}"
            status.hardware_id = f"anchor_{anchor_id}"

            # Check if anchor has sent data recently
            if anchor_id in self.udp_reader.anchor_stats:
                stats = self.udp_reader.anchor_stats[anchor_id]
                last_seen = stats['last_seen']
                rx_count = stats['rx_count']
                age = now - last_seen

                if age < 2.0:
                    status.level = DiagnosticStatus.OK
                    status.message = "Online"
                else:
                    status.level = DiagnosticStatus.WARN
                    status.message = f"Last seen {age:.1f}s ago"

                status.values.append(KeyValue(key="rx_count", value=str(rx_count)))
                status.values.append(KeyValue(key="last_seen", value=f"{age:.1f}s ago"))
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = "No data received"

            status.values.append(KeyValue(key="position", value=f"({anchor_cfg.x:.2f}, {anchor_cfg.y:.2f})"))
            diag_array.status.append(status)

        # Per-tag diagnostics
        for tag_id, tag_state in self.tags.items():
            status = DiagnosticStatus()
            status.name = f"UWB Tag {tag_id} ({tag_state.sphero_name})"
            status.hardware_id = f"tag_{tag_id}"

            if tag_state.initialized:
                age = now - tag_state.last_update
                if age < 2.0:
                    status.level = DiagnosticStatus.OK
                    status.message = "Tracking"
                else:
                    status.level = DiagnosticStatus.WARN
                    status.message = f"Stale ({age:.1f}s)"

                status.values.append(KeyValue(key="position", value=f"({tag_state.x:.2f}, {tag_state.y:.2f})"))
                status.values.append(KeyValue(key="last_update", value=f"{age:.1f}s ago"))

                # Count visible anchors
                visible = sum(1 for aid, (dist, ts) in tag_state.ranges.items()
                             if (now - ts) < self.range_timeout)
                status.values.append(KeyValue(key="anchors_visible", value=str(visible)))
            else:
                status.level = DiagnosticStatus.WARN
                status.message = "Not initialized"

            diag_array.status.append(status)

        # Overall system status
        overall = DiagnosticStatus()
        overall.name = "UWB Positioning System"
        overall.hardware_id = "sphero_uwb_positioning"

        online_anchors = sum(1 for aid in self.anchor_map.keys()
                            if aid in self.udp_reader.anchor_stats
                            and (now - self.udp_reader.anchor_stats[aid]['last_seen']) < 2.0)

        tracking_tags = sum(1 for tag in self.tags.values()
                           if tag.initialized and (now - tag.last_update) < 2.0)

        if online_anchors >= self.min_anchors:
            overall.level = DiagnosticStatus.OK
            overall.message = f"OK: {online_anchors}/{len(self.anchors)} anchors, {tracking_tags}/{len(self.tags)} tags"
        elif online_anchors > 0:
            overall.level = DiagnosticStatus.WARN
            overall.message = f"WARN: Only {online_anchors}/{len(self.anchors)} anchors online"
        else:
            overall.level = DiagnosticStatus.ERROR
            overall.message = "ERROR: No anchors online"

        overall.values.append(KeyValue(key="udp_port", value=str(self.udp_port)))
        overall.values.append(KeyValue(key="rx_count", value=str(self.udp_reader.rx_count)))
        overall.values.append(KeyValue(key="error_count", value=str(self.udp_reader.error_count)))
        diag_array.status.append(overall)

        self.diagnostics_pub.publish(diag_array)

    def _multilaterate(
        self,
        anchors: List[AnchorConfig],
        distances: List[float],
    ) -> Optional[Tuple[float, float, float]]:
        """
        2D multilateration via linearized least squares.

        Given N >= 3 anchors at positions (x_i, y_i) and distances d_i,
        we form the linear system by subtracting the last equation:

          A * [x, y]^T = b

        where:
          A[i] = [2*(x_N - x_i), 2*(y_N - y_i)]
          b[i] = (d_i² - d_N²) - (x_i² - x_N²) - (y_i² - y_N²)

        Returns (x, y, residual) or None if the system is degenerate.
        """
        n = len(anchors)
        if n < 3:
            return None

        # Use last anchor as reference
        x_ref = anchors[-1].x
        y_ref = anchors[-1].y
        d_ref = distances[-1]

        A = np.zeros((n - 1, 2))
        b = np.zeros(n - 1)

        for i in range(n - 1):
            xi, yi = anchors[i].x, anchors[i].y
            di = distances[i]

            A[i, 0] = 2.0 * (x_ref - xi)
            A[i, 1] = 2.0 * (y_ref - yi)
            b[i] = (
                (di ** 2 - d_ref ** 2)
                - (xi ** 2 - x_ref ** 2)
                - (yi ** 2 - y_ref ** 2)
            )

        try:
            # Least-squares solve
            result, residuals, rank, sv = np.linalg.lstsq(A, b, rcond=None)

            if rank < 2:
                return None

            x_est, y_est = result[0], result[1]

            # Compute residual as RMSE of distance errors
            total_error = 0.0
            for i in range(n):
                xi, yi = anchors[i].x, anchors[i].y
                di = distances[i]
                d_est = np.sqrt((x_est - xi) ** 2 + (y_est - yi) ** 2)
                total_error += (d_est - di) ** 2
            rmse = np.sqrt(total_error / n)

            return (x_est, y_est, rmse)

        except np.linalg.LinAlgError:
            return None

    def destroy_node(self):
        """Clean up UDP reader on shutdown."""
        self.udp_reader.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpheroUWBPositioningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Matrix-marker SLAM node (third positioning source, alongside ArUco and UWB).

Robots display an assigned (hue, fill) marker on their up-facing 8x8 LED
matrix. This node:
1. Reuses ArUco corner detection + FieldMapper to calibrate the field.
2. Parses the marker_assignments JSON param (dev fallback: robot_table config),
   builds name<->marker maps, assigns markers (publishes sphero/<name>/matrix),
   and optionally drives each robot's MAIN led to its hue (color boost).
3. Detects emissive markers per frame, classifies (hue, fill) -> name, and
   publishes pose on the neutral contract topic /localization/<name_safe>/position.

LED ownership: this node writes ONLY sphero/<name>/led type:"main".
front/back (aim) LEDs are owned by the state-machine layer and never touched.
"""

import json
from typing import Dict, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .boundary_detector import BoundaryDetector
from .camera_capture import CameraCapture
from .field_mapper import FieldMapper
from .matrix_detector import MatrixMarkerDetector
from .matrix_marker import HUE_PALETTE, assign_markers


def _name_safe(name: str) -> str:
    return name.replace('-', '_')


class MatrixSLAMNode(Node):
    """ROS2 node for LED-matrix active-marker positioning."""

    def __init__(self):
        super().__init__('matrix_slam_node')

        # --- Parameters (per Interface Contract) ---
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('field_width_cm', 300.0)
        self.declare_parameter('field_height_cm', 200.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('show_visualization', True)
        self.declare_parameter('drive_main_led', True)
        self.declare_parameter('color_model_path', '')
        # Blue-tape arena boundary detection (replaces ArUco corner markers).
        self.declare_parameter('blue_lower_hsv', [100, 80, 40])
        self.declare_parameter('blue_upper_hsv', [130, 255, 255])
        self.declare_parameter('boundary_min_area_frac', 0.1)
        # LED brightness scale: dims matrix + main-LED RGB to preserve hue at
        # the moderate camera exposure (avoids blooming to white).
        self.declare_parameter('led_brightness_scale', 0.4)
        # JSON: [{"name":"SB-3660","hue":"Red","fill":"filled"}, ...]
        self.declare_parameter('marker_assignments', '')
        # Dev fallback: name->(hue,fill) flattened as "name:hue:fill" entries.
        self.declare_parameter('robot_table', [])
        # Camera locks.
        self.declare_parameter('lock_white_balance', True)
        self.declare_parameter('wb_temperature', 4600)
        self.declare_parameter('lock_exposure', True)
        self.declare_parameter('exposure', 1000.0)
        self.declare_parameter('lock_autofocus', True)
        # Blob / fill thresholds.
        self.declare_parameter('min_blob_area_px', 80)
        self.declare_parameter('max_blob_area_px', 1200)
        self.declare_parameter('ring_center_frac', 0.45)
        self.declare_parameter('ring_contrast_thresh', 0.4)
        self.declare_parameter('brightness_thresh', 200)

        camera_id = self.get_parameter('camera_id').value
        field_width = self.get_parameter('field_width_cm').value
        field_height = self.get_parameter('field_height_cm').value
        publish_rate = self.get_parameter('publish_rate_hz').value
        self.show_viz = self.get_parameter('show_visualization').value
        self.drive_main_led = self.get_parameter('drive_main_led').value
        color_model_path = self.get_parameter('color_model_path').value
        self.led_brightness_scale = self.get_parameter('led_brightness_scale').value

        # --- Marker assignments: param JSON, else dev robot_table ---
        self.name_to_marker = self._load_marker_assignments()
        self.marker_to_name: Dict[Tuple[str, str], str] = {
            marker: name for name, marker in self.name_to_marker.items()
        }

        # --- Field mapper + blue-tape arena boundary detector ---
        self.field_mapper = FieldMapper(
            field_width_cm=field_width,
            field_height_cm=field_height,
        )
        self.boundary_detector = BoundaryDetector(
            blue_lower_hsv=self.get_parameter('blue_lower_hsv').value,
            blue_upper_hsv=self.get_parameter('blue_upper_hsv').value,
            min_area_frac=self.get_parameter('boundary_min_area_frac').value,
        )

        # --- Camera (shared capture with locks) ---
        self.camera = CameraCapture(
            camera_id=camera_id,
            lock_white_balance=self.get_parameter('lock_white_balance').value,
            wb_temperature=self.get_parameter('wb_temperature').value,
            lock_exposure=self.get_parameter('lock_exposure').value,
            exposure=self.get_parameter('exposure').value,
            lock_autofocus=self.get_parameter('lock_autofocus').value,
        )

        # --- Color model + matrix detector ---
        color_model = self._load_color_model(color_model_path)
        self.matrix_detector = MatrixMarkerDetector(
            color_model=color_model,
            marker_to_name=self.marker_to_name,
            min_blob_area_px=self.get_parameter('min_blob_area_px').value,
            max_blob_area_px=self.get_parameter('max_blob_area_px').value,
            ring_center_frac=self.get_parameter('ring_center_frac').value,
            ring_contrast_thresh=self.get_parameter('ring_contrast_thresh').value,
            brightness_thresh=self.get_parameter('brightness_thresh').value,
        )

        self.bridge = CvBridge()

        # --- Publishers: per-robot matrix command + optional main LED ---
        self.matrix_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        self.led_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        for name in self.name_to_marker:
            ns = _name_safe(name)
            self.matrix_pubs[name] = self.create_publisher(
                String, f'sphero/{ns}/matrix', 10)
            self.led_pubs[name] = self.create_publisher(
                String, f'sphero/{ns}/led', 10)

        # --- Pose publishers: neutral localization position contract ---
        # Shared "localization position contract": /localization/<name_safe>/position
        self.pose_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        for name in self.name_to_marker:
            topic = f'/localization/{_name_safe(name)}/position'
            self.pose_pubs[name] = self.create_publisher(PoseStamped, topic, 10)
            self.get_logger().info(f"Publishing {name} position on {topic}")

        # --- Diagnostics ---
        self.calibration_pub = self.create_publisher(String, '/aruco_slam/calibration_status', 10)
        self.image_pub = self.create_publisher(Image, '/aruco_slam/camera_feed', 10)
        self.markers_pub = self.create_publisher(String, '/aruco_slam/all_markers', 10)

        # --- Start camera + assign markers ---
        try:
            self.camera.open()
            if self.show_viz:
                cv2.namedWindow('Matrix SLAM', cv2.WINDOW_NORMAL)
            self.get_logger().info(f"Matrix SLAM node started - Camera {camera_id}")
            self.get_logger().info(f"Field size: {field_width}x{field_height} cm")
            self.get_logger().info("Arena boundary: blue-tape detection")
            self.get_logger().info(f"Markers: {self.name_to_marker}")
        except RuntimeError as e:
            self.get_logger().error(f"Failed to open camera: {e}")

        self._assign_markers()

        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.process_frame)

    # ------------------------------------------------------------------
    def _load_marker_assignments(self) -> Dict[str, Tuple[str, str]]:
        """Build name->(hue,fill) from the marker_assignments JSON param,
        falling back to the dev robot_table config when empty."""
        raw = self.get_parameter('marker_assignments').value
        name_to_marker: Dict[str, Tuple[str, str]] = {}

        if raw:
            try:
                entries = json.loads(raw)
                for e in entries:
                    name_to_marker[e['name']] = (e['hue'], e['fill'])
                return name_to_marker
            except (json.JSONDecodeError, KeyError, TypeError) as e:
                self.get_logger().error(f"Invalid marker_assignments JSON: {e}")

        # Dev fallback: robot_table as list of "name:hue:fill" strings.
        for entry in self.get_parameter('robot_table').value:
            try:
                name, hue, fill = entry.split(':')
                name_to_marker[name] = (hue, fill)
            except ValueError:
                self.get_logger().warn(f"Bad robot_table entry '{entry}' (want name:hue:fill)")

        if not name_to_marker:
            self.get_logger().warn("No marker assignments provided (empty param and config).")
        return name_to_marker

    def _load_color_model(self, path: str) -> Dict[str, dict]:
        """Load learned hue centroids; fall back to nominal palette HSV."""
        model: Dict[str, dict] = {}
        if path:
            try:
                import yaml
                with open(path, 'r') as f:
                    data = yaml.safe_load(f) or {}
                model = data.get('hues', {})
            except (OSError, ImportError) as e:
                self.get_logger().warn(f"Could not load color model '{path}': {e}")

        # Fill any missing hue from nominal palette RGB -> HSV.
        for name, rgb in HUE_PALETTE.items():
            if name in model:
                continue
            bgr = np.uint8([[[rgb[2], rgb[1], rgb[0]]]])
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[0][0]
            model[name] = {
                'h': float(hsv[0]), 's': float(hsv[1]), 'v': float(hsv[2]),
                'radius': 15.0,
            }
        return model

    def _assign_markers(self):
        """Publish each robot's matrix marker; optionally drive main LED."""
        def publish(name: str, cmd_json: str):
            msg = String()
            msg.data = cmd_json
            self.matrix_pubs[name].publish(msg)

        assign_markers(self.name_to_marker, publish,
                       brightness_scale=self.led_brightness_scale)

        if self.drive_main_led:
            scale = self.led_brightness_scale
            for name, (hue, _fill) in self.name_to_marker.items():
                r, g, b = HUE_PALETTE[hue]
                r = int(np.clip(r * scale, 0, 255))
                g = int(np.clip(g * scale, 0, 255))
                b = int(np.clip(b * scale, 0, 255))
                led_msg = String()
                # LED ownership split: matrix node writes ONLY type:"main".
                led_msg.data = json.dumps({'type': 'main', 'red': r, 'green': g, 'blue': b})
                self.led_pubs[name].publish(led_msg)

    # ------------------------------------------------------------------
    def _field_mask(self, shape) -> np.ndarray:
        """uint8 mask: 255 inside the calibrated field quad, eroded inward so
        the blue boundary band itself is excluded from robot detection.
        Returns all-255 if not calibrated."""
        mask = np.full(shape[:2], 255, dtype=np.uint8)
        if not self.field_mapper.is_calibrated():
            return mask

        quad = []
        for fc in self.field_mapper.field_corners:
            cam = self.field_mapper.field_to_camera(fc)
            if cam is None:
                return mask
            quad.append(cam)
        quad = np.array(quad, dtype=np.int32)

        mask = np.zeros(shape[:2], dtype=np.uint8)
        cv2.fillConvexPoly(mask, quad, 255)

        # Erode inward so the blue boundary band is outside the detection area.
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        mask = cv2.erode(mask, kernel, iterations=2)
        return mask

    def process_frame(self):
        """Main loop: capture, calibrate, detect markers, publish poses."""
        frame = self.camera.read()
        if frame is None:
            return

        # Calibrate from the blue-tape arena boundary (no ArUco corners).
        if not self.field_mapper.is_calibrated():
            corners = self.boundary_detector.detect(frame)
            if corners is not None and self.field_mapper.calibrate_from_corners(corners):
                self.get_logger().info("Field auto-calibration successful (blue-tape boundary)!")
            else:
                self.get_logger().warning(
                    "Arena boundary not found (blue tape)",
                    throttle_duration_sec=5.0)

        status_msg = String()
        if self.field_mapper.is_calibrated():
            status_msg.data = self.field_mapper.get_calibration_status_text()
        else:
            status_msg.data = "NOT CALIBRATED - waiting for blue-tape arena boundary"
        self.calibration_pub.publish(status_msg)

        detections: Dict[str, np.ndarray] = {}
        if self.field_mapper.is_calibrated():
            field_mask = self._field_mask(frame.shape)
            detections = self.matrix_detector.detect(frame, field_mask)

            stamp = self.get_clock().now().to_msg()
            for name, centroid in detections.items():
                field_pos = self.field_mapper.camera_to_field(centroid)
                if field_pos is None:
                    continue
                pose = PoseStamped()
                pose.header.stamp = stamp
                pose.header.frame_id = 'field'
                pose.pose.position.x = float(field_pos[0])
                pose.pose.position.y = float(field_pos[1])
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                self.pose_pubs[name].publish(pose)

        # Diagnostics: all detected markers (name + camera/field coords).
        all_markers = {'calibrated': self.field_mapper.is_calibrated(), 'markers': {}}
        for name, centroid in detections.items():
            entry = {'camera_x': float(centroid[0]), 'camera_y': float(centroid[1])}
            field_pos = self.field_mapper.camera_to_field(centroid)
            if field_pos is not None:
                entry['field_x'] = float(field_pos[0])
                entry['field_y'] = float(field_pos[1])
            all_markers['markers'][name] = entry
        markers_msg = String()
        markers_msg.data = json.dumps(all_markers)
        self.markers_pub.publish(markers_msg)

        if self.show_viz:
            for name, centroid in detections.items():
                cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 6, (0, 255, 0), -1)
                cv2.putText(frame, name, (int(centroid[0]) + 8, int(centroid[1]) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if self.field_mapper.is_calibrated():
                frame = self.field_mapper.draw_field_overlay(frame)
            cv2.imshow('Matrix SLAM', frame)
            cv2.waitKey(1)
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
            except Exception as e:
                self.get_logger().warning(f"Failed to publish image: {e}")

    def destroy_node(self):
        self.camera.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MatrixSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

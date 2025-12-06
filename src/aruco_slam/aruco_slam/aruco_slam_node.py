#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArUco SLAM Node for Soccer Field Tracking.

This ROS2 node uses ArUco markers to:
1. Calibrate a virtual soccer field using 4 corner markers
2. Detect Sphero robots with ArUco markers
3. Publish Sphero positions in field coordinates

Marker ID Assignment:
- IDs 0-3: Field corner markers (Top-Left, Top-Right, Bottom-Right, Bottom-Left)
- IDs 10+: Sphero markers (10=SB-3660, 11=SB-74FB, 12=SB-3716, 13=SB-58EF)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Pose, Point as GeomPoint, Quaternion
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
from typing import Dict, Optional
import math

from .aruco_detector import ArucoDetector
from .field_mapper import FieldMapper


class ArucoSLAMNode(Node):
    """ROS2 node for ArUco-based SLAM on a soccer field."""

    def __init__(self):
        """Initialize the ArUco SLAM node."""
        super().__init__('aruco_slam_node')

        # Declare parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('field_width_cm', 300.0)
        self.declare_parameter('field_height_cm', 200.0)
        self.declare_parameter('corner_marker_ids', [4, 5, 6, 7])
        self.declare_parameter('sphero_marker_ids', [3, 1, 2, 20])
        self.declare_parameter('sphero_names', ['SB-3660', 'SB-74FB', 'SB-3716', 'SB-58EF'])
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('auto_calibrate', True)
        self.declare_parameter('show_visualization', True)

        # Get parameters
        camera_id = self.get_parameter('camera_id').value
        field_width = self.get_parameter('field_width_cm').value
        field_height = self.get_parameter('field_height_cm').value
        corner_ids = self.get_parameter('corner_marker_ids').value
        self.sphero_marker_ids = self.get_parameter('sphero_marker_ids').value
        self.sphero_names = self.get_parameter('sphero_names').value
        publish_rate = self.get_parameter('publish_rate_hz').value
        self.auto_calibrate = self.get_parameter('auto_calibrate').value
        self.show_viz = self.get_parameter('show_visualization').value

        # Check if camera_id is valid, prompt if less than 0
        if camera_id < 0:
            self.get_logger().warn(f"Invalid camera_id: {camera_id}. Please provide a valid camera ID.")
            camera_id_input = input("Enter camera ID (e.g., 0, 1, 2...): ").strip()
            try:
                camera_id = int(camera_id_input)
                if camera_id < 0:
                    raise ValueError("Camera ID must be >= 0")
            except ValueError as e:
                self.get_logger().error(f"Invalid camera ID input: {e}")
                raise RuntimeError("Camera ID not provided or invalid")

        # Initialize detector and mapper
        self.detector = ArucoDetector(camera_id=camera_id)
        self.field_mapper = FieldMapper(
            field_width_cm=field_width,
            field_height_cm=field_height,
            corner_marker_ids=corner_ids
        )

        # CV Bridge for ROS image messages
        self.bridge = CvBridge()

        # Create publishers for each Sphero
        self.sphero_publishers: Dict[int, rclpy.publisher.Publisher] = {}
        for i, marker_id in enumerate(self.sphero_marker_ids):
            sphero_name = self.sphero_names[i] if i < len(self.sphero_names) else f"sphero_{marker_id}"
            # Replace hyphens with underscores for ROS2 topic name compliance
            topic_safe_name = sphero_name.replace('-', '_')
            topic_name = f'/aruco_slam/{topic_safe_name}/position'

            # Publish as PoseStamped (x, y in position, z=0, orientation identity)
            pub = self.create_publisher(PoseStamped, topic_name, 10)
            self.sphero_publishers[marker_id] = pub
            self.get_logger().info(f"Publishing {sphero_name} position on {topic_name}")

        # Publisher for calibration status
        self.calibration_pub = self.create_publisher(String, '/aruco_slam/calibration_status', 10)

        # Publisher for annotated video feed
        self.image_pub = self.create_publisher(Image, '/aruco_slam/camera_feed', 10)

        # Publisher for all detected markers
        self.markers_pub = self.create_publisher(String, '/aruco_slam/all_markers', 10)

        # Create timer for main processing loop
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.process_frame)

        # Start camera
        try:
            self.detector.begin_visualization()
            # Create resizable visualization window
            if self.show_viz:
                cv2.namedWindow('ArUco SLAM', cv2.WINDOW_NORMAL)
            self.get_logger().info(f"ArUco SLAM node started - Camera {camera_id}")
            self.get_logger().info(f"Field size: {field_width}x{field_height} cm")
            self.get_logger().info(f"Corner markers: {corner_ids}")
            self.get_logger().info(f"Sphero markers: {self.sphero_marker_ids}")
        except RuntimeError as e:
            self.get_logger().error(f"Failed to open camera: {e}")

    def process_frame(self):
        """Main processing loop - detect markers and publish positions."""
        # Capture frame
        frame = self.detector.get_frame()
        if frame is None:
            return

        # Detect markers
        corners, ids, rejected = self.detector.detect_markers(frame)

        # Get all detected marker centers
        marker_centers = self.detector.get_last_tag_centers()

        # Auto-calibrate if not yet calibrated
        if not self.field_mapper.is_calibrated() and self.auto_calibrate:
            if self.field_mapper.calibrate(marker_centers):
                self.get_logger().info("Field auto-calibration successful!")

        # Publish calibration status
        status_msg = String()
        status_msg.data = self.field_mapper.get_calibration_status_text()
        self.calibration_pub.publish(status_msg)

        # Process Sphero markers if field is calibrated
        if self.field_mapper.is_calibrated():
            for i, marker_id in enumerate(self.sphero_marker_ids):
                if marker_id in marker_centers:
                    # Transform to field coordinates
                    camera_pos = marker_centers[marker_id]
                    field_pos = self.field_mapper.camera_to_field(camera_pos)

                    if field_pos is not None:
                        # Create and publish PoseStamped message
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = self.get_clock().now().to_msg()
                        pose_msg.header.frame_id = 'field'

                        # Position: x and y in cm, z=0 (2D field)
                        pose_msg.pose.position.x = float(field_pos[0]) 
                        pose_msg.pose.position.y = float(field_pos[1]) 
                        pose_msg.pose.position.z = 0.0

                        # Orientation: identity quaternion (no rotation, upright)
                        pose_msg.pose.orientation.x = 0.0
                        pose_msg.pose.orientation.y = 0.0
                        pose_msg.pose.orientation.z = 0.0
                        pose_msg.pose.orientation.w = 1.0

                        self.sphero_publishers[marker_id].publish(pose_msg)

        # Publish all markers info
        current_time = self.get_clock().now()
        markers_info = {
            'timestamp': {
                'sec': current_time.seconds_nanoseconds()[0],
                'nanosec': current_time.seconds_nanoseconds()[1]
            },
            'calibrated': self.field_mapper.is_calibrated(),
            'markers': {}
        }
        for marker_id, center in marker_centers.items():
            markers_info['markers'][int(marker_id)] = {
                'camera_x': float(center[0]),
                'camera_y': float(center[1])
            }
            # Add field coordinates if calibrated
            if self.field_mapper.is_calibrated():
                field_pos = self.field_mapper.camera_to_field(center)
                if field_pos is not None:
                    markers_info['markers'][int(marker_id)]['field_x'] = float(field_pos[0])
                    markers_info['markers'][int(marker_id)]['field_y'] = float(field_pos[1])

        markers_msg = String()
        markers_msg.data = json.dumps(markers_info)
        self.markers_pub.publish(markers_msg)

        # Visualize if enabled
        if self.show_viz:
            # Draw detected markers
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                # Draw marker IDs and positions
                for i, marker_id in enumerate(ids.flatten()):
                    marker_id = int(marker_id)
                    if marker_id in marker_centers:
                        center = marker_centers[marker_id]

                        # Determine marker type and color
                        if marker_id in self.field_mapper.corner_marker_ids:
                            color = (0, 255, 255)  # Yellow for corners
                            label = f"Corner {marker_id}"
                        elif marker_id in self.sphero_marker_ids:
                            color = (0, 255, 0)  # Green for Spheros
                            sphero_idx = self.sphero_marker_ids.index(marker_id)
                            sphero_name = self.sphero_names[sphero_idx] if sphero_idx < len(self.sphero_names) else f"Sphero {marker_id}"
                            label = sphero_name
                        else:
                            color = (255, 0, 255)  # Magenta for unknown
                            label = f"ID {marker_id}"

                        # Draw center point
                        cv2.circle(frame, (int(center[0]), int(center[1])), 5, color, -1)

                        # Draw label
                        cv2.putText(
                            frame,
                            label,
                            (int(center[0]) + 10, int(center[1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            color,
                            2
                        )

                        # Draw field coordinates if calibrated and it's a Sphero
                        if self.field_mapper.is_calibrated() and marker_id in self.sphero_marker_ids:
                            field_pos = self.field_mapper.camera_to_field(center)
                            if field_pos is not None:
                                coord_text = f"({field_pos[0]:.1f}, {field_pos[1]:.1f}) cm"
                                cv2.putText(
                                    frame,
                                    coord_text,
                                    (int(center[0]) + 10, int(center[1]) + 20),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5,
                                    color,
                                    1
                                )

            # Draw field overlay if calibrated
            if self.field_mapper.is_calibrated():
                frame = self.field_mapper.draw_field_overlay(frame)

            # Draw calibration status
            status_color = (0, 255, 0) if self.field_mapper.is_calibrated() else (0, 0, 255)
            cv2.putText(
                frame,
                self.field_mapper.get_calibration_status_text(),
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                status_color,
                2
            )

            # Show frame
            cv2.imshow('ArUco SLAM', frame)
            cv2.waitKey(1)

            # Publish annotated image
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().warning(f"Failed to publish image: {e}")

    def destroy_node(self):
        """Cleanup on node shutdown."""
        self.detector.end_visualization()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = ArucoSLAMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

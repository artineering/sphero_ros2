#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Color calibrator (console tool).

Commands the fleet to cycle through each of the 8 hues (filled marker), samples
the observed emissive blob hue under the camera locks, and writes learned hue
centroids + radii to config/color_model.yaml. Warns if any two hue centroids
overlap within radius (separability guard).

Usage:
    ros2 run aruco_slam color_calibrator --ros-args \
        -p robot_names:="['SB-3660','SB-74FB']" -p camera_id:=0
"""

import json
import time
from typing import Dict, List

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .camera_capture import CameraCapture
from .matrix_marker import HUE_ORDER, HUE_PALETTE, marker_to_matrix_cmd


def _name_safe(name: str) -> str:
    return name.replace('-', '_')


class ColorCalibrator(Node):
    """Learns per-hue emissive centroids from observed blobs."""

    def __init__(self):
        super().__init__('color_calibrator')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('robot_names', [])
        self.declare_parameter('output_path', 'config/color_model.yaml')
        self.declare_parameter('settle_time_s', 2.0)
        self.declare_parameter('brightness_thresh', 200)
        self.declare_parameter('lock_white_balance', True)
        self.declare_parameter('wb_temperature', 4600)
        self.declare_parameter('lock_exposure', True)
        self.declare_parameter('exposure', 120.0)
        self.declare_parameter('lock_autofocus', True)

        self.camera_id = self.get_parameter('camera_id').value
        self.robot_names: List[str] = list(self.get_parameter('robot_names').value)
        self.output_path = self.get_parameter('output_path').value
        self.settle_time_s = self.get_parameter('settle_time_s').value
        self.brightness_thresh = self.get_parameter('brightness_thresh').value

        self.camera = CameraCapture(
            camera_id=self.camera_id,
            lock_white_balance=self.get_parameter('lock_white_balance').value,
            wb_temperature=self.get_parameter('wb_temperature').value,
            lock_exposure=self.get_parameter('lock_exposure').value,
            exposure=self.get_parameter('exposure').value,
            lock_autofocus=self.get_parameter('lock_autofocus').value,
        )

        self.matrix_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        for name in self.robot_names:
            self.matrix_pubs[name] = self.create_publisher(
                String, f'sphero/{_name_safe(name)}/matrix', 10)

    def _show_hue(self, hue: str):
        cmd = marker_to_matrix_cmd(hue, 'filled')
        msg = String()
        msg.data = cmd
        for pub in self.matrix_pubs.values():
            pub.publish(msg)

    def _sample_hue(self) -> "tuple[float, float, float] | None":
        """Mean (H, S, V) of the brightest blob in the current frame."""
        frame = self.camera.read()
        if frame is None:
            return None
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        v = hsv[:, :, 2]
        _, bright = cv2.threshold(v, self.brightness_thresh, 255, cv2.THRESH_BINARY)
        num, labels, stats, _ = cv2.connectedComponentsWithStats(bright)
        if num <= 1:
            return None
        # Largest non-background component.
        areas = stats[1:, cv2.CC_STAT_AREA]
        label = int(np.argmax(areas)) + 1
        mask = (labels == label).astype(np.uint8)
        mean = cv2.mean(hsv, mask=mask)
        return float(mean[0]), float(mean[1]), float(mean[2])

    def run(self) -> Dict[str, dict]:
        """Cycle each hue, sample, return learned centroids."""
        self.camera.open()
        learned: Dict[str, dict] = {}

        for hue in HUE_ORDER:
            self.get_logger().info(f"Calibrating hue '{hue}'...")
            self._show_hue(hue)
            time.sleep(self.settle_time_s)
            # Flush a few frames so the locked exposure settles.
            for _ in range(5):
                self.camera.read()
            sample = self._sample_hue()
            if sample is None:
                self.get_logger().warn(f"No blob seen for '{hue}'; using nominal palette.")
                rgb = HUE_PALETTE[hue]
                bgr = np.uint8([[[rgb[2], rgb[1], rgb[0]]]])
                nom = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[0][0]
                learned[hue] = {'h': float(nom[0]), 's': float(nom[1]),
                                'v': float(nom[2]), 'radius': 15.0}
                continue
            h, s, vv = sample
            learned[hue] = {'h': h, 's': s, 'v': vv, 'radius': 15.0}
            self.get_logger().info(f"  '{hue}' -> H={h:.1f} S={s:.1f} V={vv:.1f}")

        self._check_separability(learned)
        self._write(learned)
        return learned

    def _check_separability(self, learned: Dict[str, dict]):
        names = list(learned.keys())
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a, b = learned[names[i]], learned[names[j]]
                dh = abs(a['h'] - b['h'])
                dh = min(dh, 180.0 - dh)
                if dh < (a['radius'] + b['radius']):
                    self.get_logger().warn(
                        f"Hues '{names[i]}' and '{names[j]}' overlap "
                        f"(hue dist {dh:.1f} < {a['radius'] + b['radius']:.1f}). "
                        f"Lower exposure or adjust palette."
                    )

    def _write(self, learned: Dict[str, dict]):
        try:
            import yaml
            with open(self.output_path, 'w') as f:
                yaml.safe_dump({'hues': learned}, f, default_flow_style=False)
            self.get_logger().info(f"Wrote color model to {self.output_path}")
        except (OSError, ImportError) as e:
            self.get_logger().error(f"Failed to write color model: {e}")
            self.get_logger().info(json.dumps({'hues': learned}))

    def destroy_node(self):
        self.camera.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ColorCalibrator()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

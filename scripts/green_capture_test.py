#!/usr/bin/env python3
"""Diagnostic: light SB-AAAD's matrix green and capture Kinect RGB frames.

Isolates the registration "no_green" failure from the full probe: does the probe
colour actually show up in the Kinect RGB, and HOW LONG does it stay lit?

PREREQUISITES (run on rpi5-main):
  * STOP the field_tracker_node first -- it owns the Kinect; two processes cannot
    grab the device at once.
  * The SB-AAAD device controller (on its worker Pi) must be running and BLE-
    connected, so the /sphero/SB_AAAD/led command actually reaches the robot.
  * Source the workspace:  source install/setup.bash

It sends green ONCE at t0 (does NOT re-send), then grabs an RGB frame at several
offsets so a matrix that only stays lit ~1s shows up as: green at t=0.7s, dark
later. Frames are written to ~/kinect_field/test_green_SB_AAAD_t<ms>.png (saved
as the camera sees them). The matrix is blanked at the end.

Usage:
  python3 scripts/green_capture_test.py
"""
import json
import os
import time

import cv2
import numpy as np
import rclpy
from std_msgs.msg import String

from kinect_field_tracking.detection import FreenectSource

NAME_SAFE = "SB_AAAD"
OUT_DIR = os.path.expanduser("~/kinect_field")
CAPTURE_OFFSETS_S = [0.7, 1.5, 2.5, 4.0, 6.0]   # when to grab, after lighting
PROBE_RGB = (0, 255, 0)                          # green


def _led(led_type, r, g, b):
    return json.dumps({"type": led_type, "red": r, "green": g, "blue": b})


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("green_capture_test")
    pub = node.create_publisher(String, f"sphero/{NAME_SAFE}/led", 10)

    # Wait for the device controller's subscription to match (cross-host to the
    # worker Pi), or the first publish is dropped.
    print(f"waiting for a subscriber on /sphero/{NAME_SAFE}/led ...")
    deadline = time.time() + 5.0
    while time.time() < deadline and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    subs = pub.get_subscription_count()
    print(f"  subscriber count = {subs}"
          + ("" if subs else "  <-- WARNING: device controller not reachable; "
                             "LED command will go nowhere"))

    # Open the Kinect (field_tracker_node MUST be stopped) and warm up video.
    print("opening Kinect (video) ...")
    src = FreenectSource()
    src.stop()
    for _ in range(3):
        src.get_video()

    # Blank the status LEDs, then light the matrix green -- ONCE.
    pub.publish(String(data=_led("front", 0, 0, 0)))
    pub.publish(String(data=_led("back", 0, 0, 0)))
    pub.publish(String(data=_led("main", *PROBE_RGB)))
    t0 = time.time()
    print(f"green sent at t0 (rgb={PROBE_RGB}); capturing ...")

    saved = []
    for off in CAPTURE_OFFSETS_S:
        while time.time() - t0 < off:
            rclpy.spin_once(node, timeout_sec=0.02)
        rgb = src.get_video()                       # HxWx3, RGB
        # crude "is there green?" readout per frame to log alongside the image
        r, g, b = rgb[:, :, 0].astype(int), rgb[:, :, 1].astype(int), rgb[:, :, 2].astype(int)
        greenish = int(np.count_nonzero((g > 120) & (g > r + 40) & (g > b + 40)))
        path = os.path.join(OUT_DIR, f"test_green_{NAME_SAFE}_t{int(off * 1000)}.png")
        cv2.imwrite(path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        saved.append(path)
        print(f"  t={off:>4}s  green-ish px={greenish:6d}  -> {path}")

    # Blank the matrix on the way out.
    pub.publish(String(data=_led("main", 0, 0, 0)))
    rclpy.spin_once(node, timeout_sec=0.2)
    src.close()
    node.destroy_node()
    rclpy.shutdown()

    print("\nsaved frames:")
    for p in saved:
        print("  " + p)
    print("\nReadout: if 'green-ish px' is high at t=0.7s then drops to ~0 later,"
          " the matrix is being turned off after ~1s (something overrides the probe).")


if __name__ == "__main__":
    main()

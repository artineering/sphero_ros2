#!/usr/bin/env python3
"""Synchronized fleet matrix blink.

Drives all four Spheros' 8x8 LED matrices from a SINGLE node/clock so the
colour change leaves the coordinator at the same instant for every robot
(sidesteps per-Pi clock skew). Cycles red -> green -> blue.

Usage:
  python3 blink_fleet.py test            # one-shot red fill on all robots
  python3 blink_fleet.py blink [interval]  # loop R/G/B, default 1.0s per colour
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# name_safe forms (name.replace('-', '_')) of the 4 connected Spheros.
ROBOTS = ['SB_1FA8', 'SB_E12C', 'SB_3AAC', 'SB_E531', 'SB_3660', 'SB_DADB']
COLORS = [('red', 255, 0, 0), ('green', 0, 255, 0), ('blue', 0, 0, 255)]


def fill_cmd(r, g, b):
    return json.dumps({'pattern': '', 'matrix': [1] * 64,
                       'red': r, 'green': g, 'blue': b, 'duration': 0.0})


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'blink'
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

    rclpy.init()
    node = Node('blink_fleet')
    pubs = {name: node.create_publisher(String, f'sphero/{name}/matrix', 10)
            for name in ROBOTS}
    # Give DDS discovery a moment to match the remote device-controller subs.
    time.sleep(1.0)

    def broadcast(r, g, b):
        msg = String(); msg.data = fill_cmd(r, g, b)
        for p in pubs.values():          # tight loop -> all robots ~same instant
            p.publish(msg)

    if mode == 'test':
        node.get_logger().info('One-shot RED fill on all robots')
        broadcast(255, 0, 0)
        time.sleep(0.5)                  # let publishes flush before shutdown
        node.destroy_node(); rclpy.shutdown(); return

    node.get_logger().info(f'Blinking R-G-B on {ROBOTS} every {interval}s (Ctrl+C to stop)')
    try:
        i = 0
        while True:
            name, r, g, b = COLORS[i % len(COLORS)]
            broadcast(r, g, b)
            i += 1
            time.sleep(interval)
    except KeyboardInterrupt:
        broadcast(0, 0, 0)               # clear on exit
        time.sleep(0.3)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

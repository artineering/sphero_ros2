#!/usr/bin/env python3
"""Measure command latency and stopping distance for one roll + stop.

Rolls the robot at SPEED for RUN s (one command, or re-sent every 0.5 s in
'stream' mode, like a 2 Hz controller), then sends stop, and reports from the
camera + the device controller's motion_cmd echo:
  first roll -> motion starts, stop sent -> stop echoed, stop sent -> still,
  peak speed and distance travelled after the stop was sent.

Usage (ROS sourced):
  python3 scripts/stop_probe.py single|stream <cmd_heading> [speed=40] [run_s=1.5]
"""
import json
import math
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

from goto_target import Camera

ROBOT = 'SB_418F'


def main():
    mode, heading = sys.argv[1], int(sys.argv[2])
    speed = int(sys.argv[3]) if len(sys.argv) > 3 else 40
    run = float(sys.argv[4]) if len(sys.argv) > 4 else 1.5
    H = np.array(yaml.safe_load(open(
        '/home/svaghela/overhead_field/overhead_arena.yaml'))['H'])

    rclpy.init()
    node = rclpy.create_node('stop_probe')
    pub = node.create_publisher(String, f'/sphero/{ROBOT}/roll', 10)
    echo = []
    node.create_subscription(String, f'/sphero/{ROBOT}/motion_cmd',
                             lambda m: echo.append((time.time(), m.data)), 10)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(1.0)
    cam = Camera('/dev/video0', H)

    sent, track = [], []

    def roll(s):
        pub.publish(String(data=json.dumps({'speed': s, 'heading': heading})))
        sent.append((time.time(), s))

    t0 = time.time()
    stop_at, stopped, last = t0 + run, False, 0
    while time.time() < t0 + 5.0:
        _, blobs = cam.frame()
        now = time.time()
        lit = [b for b in blobs if b[2] is not None]
        if lit:
            track.append((now, lit[0][0], lit[0][1]))
        if not stopped and now >= stop_at:
            roll(0)
            stopped = True
        elif not stopped and (not sent or (mode == 'stream' and now - last >= 0.5)):
            roll(speed)
            last = now
    cam.cap.release()

    T = np.array(track)
    v = np.hypot(np.diff(T[:, 1]), np.diff(T[:, 2])) / np.diff(T[:, 0])
    vs = np.convolve(v, np.ones(5) / 5, 'same')
    t_stop = [t for t, s in sent if s == 0][0]
    t_echo = [t for t, d in echo if json.loads(d)['speed'] == 0 and t >= t_stop]
    moving = T[1:][vs > 3]
    t_start = moving[0, 0] if len(moving) else float('nan')
    t_end = moving[-1, 0] if len(moving) else float('nan')
    i = np.searchsorted(T[:, 0], t_stop)
    print(f'mode {mode}: sent {len(sent)} cmds; echoes {len(echo)}')
    print(f'  first roll -> motion starts: {t_start - sent[0][0]:.2f} s')
    print(f'  stop sent -> stop echoed:    {(t_echo[0] - t_stop) if t_echo else float("nan"):.2f} s')
    print(f'  stop sent -> motion ends:    {t_end - t_stop:.2f} s')
    print(f'  peak speed {vs.max():.1f} cm/s; distance after stop sent: '
          f'{math.hypot(T[-1, 1] - T[i, 1], T[-1, 2] - T[i, 2]):.1f} cm')
    for t, d in echo:
        print(f'    echo +{t - sent[0][0]:.2f} {d}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

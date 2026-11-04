#!/usr/bin/env python3
"""Random moves for every robot the overhead node tracks, for --duration s.

Each robot independently: roll at --speed for 1.0-2.0 s toward a random point
in the arena (inset --margin cm), stop, pause 1.0-2.5 s, repeat. Aim uses the
node's pose and an offset (field = offset - cmd) read from the node's LED
heading at cmd 0 at the start; when the node does not report the robot HEALTHY
the heading is random. No LED writes.

Usage (ROS sourced, overhead node tracking):
  python3 scripts/random_drive.py [--duration 150] [--speed 70]
"""
import argparse
import json
import math
import random
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

ARENA_W, ARENA_H = 243.84, 182.88


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=150.0)
    ap.add_argument('--speed', type=int, default=70)
    ap.add_argument('--margin', type=float, default=30.0)
    ap.add_argument('--seed', type=int, default=1)
    args = ap.parse_args()
    if args.speed < 50:
        ap.error('speed must be >= 50')
    random.seed(args.seed)

    rclpy.init()
    node = rclpy.create_node('random_drive')
    state = {}
    node.create_subscription(String, '/overhead_tracker_node/state',
                             lambda m: state.update(json.loads(m.data)), 10)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    t_end = time.time() + 5
    while 'tracks' not in state and time.time() < t_end:
        time.sleep(0.1)
    if 'tracks' not in state:
        raise SystemExit('no /overhead_tracker_node/state')
    robots = sorted(state['tracks'])
    pubs = {r: node.create_publisher(String, f'/sphero/{r.replace("-", "_")}/roll', 10)
            for r in robots}
    time.sleep(1.0)
    last = {r: 0 for r in robots}

    def roll(r, speed, h):
        last[r] = int(round(h)) % 360
        pubs[r].publish(String(data=json.dumps({'speed': speed, 'heading': last[r]})))

    for r in robots:
        roll(r, 0, 0)
    time.sleep(3.0)                      # ~/state is 1 Hz: let two samples land
    offset = {r: state['tracks'][r].get('heading') for r in robots}
    print('offsets (cmd 0):', {r: (None if h is None else round(h)) for r, h in offset.items()})

    stop = threading.Event()
    counts = {r: 0 for r in robots}

    def worker(r):
        time.sleep(random.uniform(0, 1.5))   # desynchronise
        while not stop.is_set():
            tr = state['tracks'].get(r, {})
            tx = random.uniform(args.margin, ARENA_W - args.margin)
            ty = random.uniform(args.margin, ARENA_H - args.margin)
            if offset[r] is not None and tr.get('status') == 'HEALTHY':
                phi = math.degrees(math.atan2(ty - tr['y_cm'], tx - tr['x_cm']))
                h = offset[r] - phi
            else:
                h = random.uniform(0, 360)
            roll(r, args.speed, h)
            counts[r] += 1
            stop.wait(random.uniform(1.0, 2.0))
            roll(r, 0, last[r])
            stop.wait(random.uniform(1.0, 2.5))

    threads = [threading.Thread(target=worker, args=(r,), daemon=True) for r in robots]
    t0 = time.time()
    for t in threads:
        t.start()
    try:
        while time.time() - t0 < args.duration:
            time.sleep(5)
            st = state.get('tracks', {})
            print(f'{time.time() - t0:5.0f} s  moves {sum(counts.values()):3d}  '
                  + ' '.join(f'{r[3:]}:{st.get(r, {}).get("status", "?")[:4]}' for r in robots))
    finally:
        stop.set()
        time.sleep(0.3)
        for _ in range(3):
            for r in robots:
                roll(r, 0, last[r])
            time.sleep(0.3)
        node.destroy_node()
        rclpy.shutdown()
    print('moves per robot:', counts)


if __name__ == '__main__':
    main()

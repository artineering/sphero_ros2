#!/usr/bin/env python3
"""Turn every linked robot in place (speed 0) to face a field point.

Headings come from the running overhead node's ~/state (LED pair, field deg,
CCW from +x). Each robot's offset (field = offset - cmd) is measured at cmd 0,
then refined from the heading it actually reaches; small corrections swing 30
deg away first (in-place turns under ~5 deg are not executed reliably).

Usage (ROS sourced, overhead node tracking):
  python3 scripts/face_point.py [--point 121.92 91.44] [--tol 8] [--tries 3]
"""
import argparse
import json
import math
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--point', nargs=2, type=float, default=[121.92, 91.44])
    ap.add_argument('--tol', type=float, default=8.0)
    ap.add_argument('--tries', type=int, default=3)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('face_point')
    states = []
    node.create_subscription(String, '/overhead_tracker_node/state',
                             lambda m: states.append(json.loads(m.data)), 10)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    t_end = time.time() + 5
    while not states and time.time() < t_end:
        time.sleep(0.1)
    if not states:
        raise SystemExit('no /overhead_tracker_node/state')
    robots = sorted(states[-1]['tracks'])
    pubs = {r: node.create_publisher(String, f'/sphero/{r.replace("-", "_")}/roll', 10)
            for r in robots}
    time.sleep(1.0)
    cmd = {}

    def roll0(r, h):
        cmd[r] = int(round(h)) % 360
        pubs[r].publish(String(data=json.dumps({'speed': 0, 'heading': cmd[r]})))

    def headings(n=3):
        """Circular mean of the next n ~/state samples (1 Hz) per robot."""
        k0 = len(states)
        while len(states) < k0 + n:
            time.sleep(0.1)
        out = {}
        for r in robots:
            hs = [s['tracks'][r]['heading'] for s in states[k0:]
                  if s['tracks'].get(r, {}).get('heading') is not None]
            if hs:
                out[r] = math.degrees(math.atan2(sum(math.sin(math.radians(h)) for h in hs),
                                                 sum(math.cos(math.radians(h)) for h in hs))) % 360
        return out

    px, py = args.point
    pos = {r: (states[-1]['tracks'][r]['x_cm'], states[-1]['tracks'][r]['y_cm']) for r in robots}
    want = {r: math.degrees(math.atan2(py - pos[r][1], px - pos[r][0])) % 360 for r in robots}
    for r in robots:
        roll0(r, 0)
    time.sleep(2.5)
    h = headings()
    offset = {r: h[r] for r in h}                        # cmd 0 -> field = offset
    for r in robots:
        if r not in offset:
            print(f'{r}: no LED heading, skipped')
    todo = [r for r in robots if r in offset]
    for r in todo:
        roll0(r, offset[r] - want[r])
    for k in range(args.tries):
        time.sleep(2.5)
        h = headings()
        late = []
        for r in todo:
            if r not in h:
                continue
            err = wrap(h[r] - want[r])
            print(f'try {k + 1} {r}: want {want[r]:5.1f} got {h[r]:5.1f} err {err:+5.1f}')
            offset[r] = (h[r] + cmd[r]) % 360                 # refine from what it reached
            if abs(err) > args.tol:
                late.append(r)
        if not late or k == args.tries - 1:
            todo = late
            break
        for r in late:                                    # swing away, then come back
            roll0(r, offset[r] - want[r] + 30)
        time.sleep(1.0)
        for r in late:
            roll0(r, offset[r] - want[r])
        todo = late
    print('done' + (f'; still off: {todo}' if todo else ''))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

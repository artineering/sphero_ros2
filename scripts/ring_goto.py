#!/usr/bin/env python3
"""Drive every robot at once to a ring around a field point, each on its own
line in (target = point + R * unit(robot - point)).

Robot identities and start positions come from the running overhead node's
~/state (the node is then stopped so this script can own the camera), or with
--init-track from the last rows of an earlier run's track.csv.

Ring spots are spread evenly (360/N apart) and handed out in the order the
robots sit around the point, so neighbours keep their order and paths do not
cross. A move longer than LONG_CM stops COAST_CM short of the target; a
shorter one stops after moving SHORT_FRAC of its distance (the robot never
reaches full speed, so it coasts less). Up to --passes passes, each moving
only the robots still more than --tol cm from their spot. Offsets
(field = offset - cmd) come from the LED pair at cmd 0. Speed 50 (user rule:
never below 50), re-aimed every 0.5 s, stopped COAST_CM short of the target.
Records raw.mjpeg / frames.csv / cmds.csv / events.csv / track.csv.

Usage (ROS sourced, overhead node tracking):
  python3 scripts/ring_goto.py [--point 121.92 91.44] [--radius 30]
"""
import argparse
import json
import math
import os
import subprocess
import threading
import time

import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

from crossing_record import Recorder, Tracker

SPEED = 50
COAST_CM = 42.9
LONG_CM = 60.0
SHORT_FRAC = 0.4


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--point', nargs=2, type=float, default=[121.92, 91.44])
    ap.add_argument('--radius', type=float, default=30.0)
    ap.add_argument('--init-track', help="earlier run's track.csv: start from its last rows")
    ap.add_argument('--return-to', help="earlier run's track.csv: drive each robot back to "
                    'its first row there instead of to the ring')
    ap.add_argument('--passes', type=int, default=3)
    ap.add_argument('--tol', type=float, default=10.0)
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--out', default=os.path.expanduser(
        f'~/sphero_ros2/recordings/ring_{time.strftime("%Y%m%d_%H%M%S")}'))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    H = np.array(yaml.safe_load(open(os.path.expanduser(
        '~/overhead_field/overhead_arena.yaml')))['H'])

    rclpy.init()
    node = rclpy.create_node('ring_goto')
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    if args.init_track:
        start = {}
        for line in open(args.init_track).read().splitlines()[1:]:
            _, r, x, y, _ = line.split(',')
            start[r] = (float(x), float(y))
        robots = sorted(start)
    else:
        states = []
        sub = node.create_subscription(String, '/overhead_tracker_node/state',
                                       lambda m: states.append(json.loads(m.data)), 10)
        t_end = time.time() + 5
        while not states and time.time() < t_end:
            time.sleep(0.1)
        if not states:
            raise SystemExit('no /overhead_tracker_node/state')
        st = states[-1]['tracks']
        robots = sorted(st)
        start = {r: (st[r]['x_cm'], st[r]['y_cm']) for r in robots}
        node.destroy_subscription(sub)
        subprocess.run(['pkill', '-INT', '-f', '[o]verhead_tracker_node --ros-args'])
        t_end = time.time() + 10
        while subprocess.run(['pgrep', '-f', '[o]verhead_tracker_node'],
                             capture_output=True).returncode == 0 and time.time() < t_end:
            time.sleep(0.2)

    ns = {r: r.replace('-', '_') for r in robots}
    cmds = open(os.path.join(args.out, 'cmds.csv'), 'w')
    cmds.write('t,robot,speed,heading\n')
    events = open(os.path.join(args.out, 'events.csv'), 'w')
    events.write('t,robot,event\n')
    track = open(os.path.join(args.out, 'track.csv'), 'w')
    track.write('t,robot,x,y,heading_deg\n')
    roll_pub, led_pub = {}, {}
    for r in robots:
        node.create_subscription(
            String, f'/sphero/{ns[r]}/motion_cmd',
            lambda m, r=r: cmds.write(f'{time.time():.4f},{r},{json.loads(m.data).get("speed", 0)},'
                                      f'{json.loads(m.data).get("heading", 0)}\n'), 50)
        roll_pub[r] = node.create_publisher(String, f'/sphero/{ns[r]}/roll', 10)
        led_pub[r] = node.create_publisher(String, f'/sphero/{ns[r]}/led', 10)
    time.sleep(1.0)
    last_cmd = {r: 0 for r in robots}

    def roll(r, speed, h):
        last_cmd[r] = int(round(h)) % 360
        roll_pub[r].publish(String(data=json.dumps({'heading': last_cmd[r], 'speed': speed})))

    def event(r, what):
        events.write(f'{time.time():.4f},{r},{what}\n')
        print(f'  {r}: {what}')

    def leds_on():
        for r in robots:
            for kind, rgb in (('main', (0, 0, 0)), ('front', (0, 255, 0)), ('back', (255, 0, 0))):
                led_pub[r].publish(String(data=json.dumps(
                    {'type': kind, 'red': rgb[0], 'green': rgb[1], 'blue': rgb[2]})))

    rec = Recorder(args.device, args.out)
    while rec.get() is None:
        time.sleep(0.05)
    trk = Tracker(rec, H, track)
    trk.pos = {r: (*start[r], None) for r in robots}
    vel = {r: (0.0, 0.0) for r in robots}
    t_last = [time.time()]

    def update(want_heading=False):
        t, meas = trk.measure(want_heading)
        dt = max(t - t_last[0], 1e-3)
        pred = {r: (trk.pos[r][0] + vel[r][0] * dt, trk.pos[r][1] + vel[r][1] * dt) for r in robots}
        pairs = sorted((math.hypot(m[0] - pred[r][0], m[1] - pred[r][1]), r, j)
                       for r in robots for j, m in enumerate(meas))
        used_r, used_m = set(), set()
        for d, r, j in pairs:
            if d > 25 or r in used_r or j in used_m:
                continue
            used_r.add(r)
            used_m.add(j)
            x, y, h = meas[j]
            ox, oy, _ = trk.pos[r]
            vel[r] = (0.7 * vel[r][0] + 0.3 * (x - ox) / dt, 0.7 * vel[r][1] + 0.3 * (y - oy) / dt)
            trk.pos[r] = (x, y, h)
            track.write(f'{t:.4f},{r},{x:.2f},{y:.2f},{"" if h is None else f"{h:.1f}"}\n')
        t_last[0] = t

    def settle(s):
        t_end = time.time() + s
        while time.time() < t_end:
            update()

    try:
        _, meas = trk.measure()
        for r in robots:                     # every robot must be where we think it is
            x, y, _ = trk.pos[r]
            if not meas or min(math.hypot(m[0] - x, m[1] - y) for m in meas) > 15:
                raise SystemExit(f'{r}: no blob within 15 cm of ({x:.1f},{y:.1f}) -- moved?')
        update()
        leds_on()
        settle(1.5)
        leds_on()
        for r in robots:
            roll(r, 0, 0)
        settle(2.5)
        hs = {r: [] for r in robots}
        t_end = time.time() + 3.0
        while time.time() < t_end:
            update(want_heading=True)
            for r in robots:
                if trk.pos[r][2] is not None:
                    hs[r].append(math.radians(trk.pos[r][2]))
        offset = {}
        for r in robots:
            if len(hs[r]) < 5:
                print(f'  {r}: no LED heading, stays put')
                continue
            offset[r] = math.degrees(math.atan2(np.mean(np.sin(hs[r])), np.mean(np.cos(hs[r])))) % 360
        px, py = args.point
        # even spots, handed out in angular order around the point
        ang = {r: math.atan2(trk.pos[r][1] - py, trk.pos[r][0] - px) for r in offset}
        order = sorted(offset, key=lambda r: ang[r])
        n = len(order)
        step = 2 * math.pi / n
        a0 = math.atan2(sum(math.sin(ang[r] - k * step) for k, r in enumerate(order)),
                        sum(math.cos(ang[r] - k * step) for k, r in enumerate(order)))
        target = {r: (px + args.radius * math.cos(a0 + k * step),
                      py + args.radius * math.sin(a0 + k * step)) for k, r in enumerate(order)}
        if args.return_to:
            first = {}
            for line in open(args.return_to).read().splitlines()[1:]:
                _, r, x, y, _ = line.split(',')
                first.setdefault(r, (float(x), float(y)))
            target = {r: first[r] for r in offset}

        def drive(rs, name):
            plan = {}
            for r in rs:
                x, y, _ = trk.pos[r]
                d = math.hypot(target[r][0] - x, target[r][1] - y)
                phi = math.degrees(math.atan2(target[r][1] - y, target[r][0] - x))
                roll(r, 0, offset[r] - phi)              # face the target first
                plan[r] = ((x, y), d)
            settle(1.5)
            for r in rs:
                event(r, f'{name} -> ({target[r][0]:.1f},{target[r][1]:.1f}), {plan[r][1]:.0f} cm'
                         + ('' if plan[r][1] > LONG_CM else ' (short)'))
                roll(r, SPEED, last_cmd[r])
            moving, t_aim, t_end = set(rs), {r: time.time() for r in rs}, time.time() + 15.0
            while moving and time.time() < t_end:
                update()
                now = time.time()
                for r in list(moving):
                    x, y, _ = trk.pos[r]
                    tx, ty = target[r]
                    rem = math.hypot(tx - x, ty - y)
                    (sx, sy), d = plan[r]
                    moved = math.hypot(x - sx, y - sy)
                    if (d > LONG_CM and rem <= COAST_CM) or \
                            (d <= LONG_CM and moved >= max(SHORT_FRAC * d, 2.0)):
                        roll(r, 0, last_cmd[r])
                        moving.discard(r)
                        event(r, f'{name} stop rem={rem:.1f} moved={moved:.1f}')
                    elif d > LONG_CM and now - t_aim[r] >= 0.5:
                        phi = math.degrees(math.atan2(ty - y, tx - x))
                        cmd = (offset[r] - phi) % 360
                        if abs((cmd - last_cmd[r] + 180) % 360 - 180) > 3:
                            roll(r, SPEED, cmd)
                        t_aim[r] = now
            for r in moving:
                roll(r, 0, last_cmd[r])
                event(r, f'{name} stop on timeout')
            settle(3.0)

        for k in range(args.passes):
            late = [r for r in target if math.hypot(trk.pos[r][0] - target[r][0],
                                                    trk.pos[r][1] - target[r][1]) > args.tol]
            if not late:
                break
            print(f'pass {k + 1}: {len(late)} robot(s)')
            drive(late, f'pass{k + 1}')
        print('\nresult:')
        for r in target:
            x, y, _ = trk.pos[r]
            print(f'  {r}: at ({x:6.1f},{y:6.1f})  {math.hypot(x - px, y - py):5.1f} cm from the point, '
                  f'{math.hypot(x - target[r][0], y - target[r][1]):4.1f} cm from its target')
    finally:
        for r in robots:
            roll(r, 0, last_cmd[r])
        time.sleep(0.5)
        rec.close()
        for f in (cmds, events, track):
            f.close()
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    print(f'{rec.n} frames -> {args.out}')


if __name__ == '__main__':
    main()

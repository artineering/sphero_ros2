#!/usr/bin/env python3
"""Record the whole fleet patrolling, for offline tracker tests.

Layout (field cm, from the 2026-11-04 ground-truth placement):
  corners  four robots on the tape crosses; each leg every one drives to the
           next cross around the periphery (same rotation), so after 4 legs
           each is home again
  lines    two robots on the tape-line midpoints swap sides every leg, in
           lanes 12 cm apart (a close pass in the middle of each leg)
  centre   one robot circles the arena centre while the others drive
           (heading steps of 45 deg every 0.5 s at speed 50, ~19 cm radius)

All robots first drive to their start points. Speed is 50 everywhere (user
rule: never below 50); a moving robot is re-aimed every 0.5 s and stopped
COAST_CM before its target. LEDs (main off, front green, back red) and the
heading offsets (field = offset - cmd, from the LED pair at rest) are
refreshed only between legs, never while anything rolls. A robot without a
readable LED pair (e.g. SB-3660) gets its offset from the direction it moves.

Writes raw.mjpeg, frames.csv, cmds.csv, events.csv, track.csv (as
crossing_record.py) to recordings/patrol_<time>/, and records a rosbag
(mcap) in <out>/bag with:
  /patrol/camera/compressed     every camera frame (the MJPEG bytes, 60 fps)
  /patrol/annotated/compressed  10 Hz half-res frames: names, LED heading arrows, targets
  /patrol/<ns>/pose             field cm, LED heading as yaw (identity if unknown)
  /patrol/tracks                JSON per update: t, robot, x, y, heading_deg, heading_src
  /sphero/<ns>/{motion_cmd,roll,led,status,sensors}

Usage (ROS sourced): python3 scripts/patrol_record.py [--laps 1]
"""
import argparse
import json
import math
import os
import threading
import time

import signal
import subprocess
import sys

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from crossing_record import Recorder, Tracker, ARENA_W, ARENA_H

sys.path.insert(0, os.path.expanduser('~/sphero_ros2/src/overhead_tracking'))
from overhead_tracking import blobs, color_decode  # noqa: E402

SPEED = 50
COAST_CM = 42.9            # stop -> rest at speed 50
CROSSES = [('SB-5B47', (230.7, 11.2)), ('SB-AAAD', (13.8, 13.7)),
           ('SB-E12C', (16.8, 175.0)), ('SB-E531', (233.4, 176.1))]
LINES = [('SB-BBBE', (215.9, 44.0)), ('SB-3660', (32.3, 56.0))]   # 12 cm lanes
CENTRE_ROBOT, CENTRE, CIRCLE_R = 'SB-1FA8', (123.3, 90.9), 19.0
NO_LED = {'SB-3660'}       # LEDs dead: its "LED" readings are spurious, use motion
WALL_STOP_CM = 6.0         # stop a robot whose centre gets this close to a wall


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--laps', type=int, default=1, help='4 legs per lap')
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--out', default=os.path.expanduser(
        f'~/sphero_ros2/recordings/patrol_{time.strftime("%Y%m%d_%H%M%S")}'))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    robots = [r for r, _ in CROSSES] + [r for r, _ in LINES] + [CENTRE_ROBOT]
    ns = {r: r.replace('-', '_') for r in robots}
    H = np.array(yaml.safe_load(open(os.path.expanduser(
        '~/overhead_field/overhead_arena.yaml')))['H'])

    rclpy.init()
    node = rclpy.create_node('patrol_record')
    cmds = open(os.path.join(args.out, 'cmds.csv'), 'w')
    cmds.write('t,robot,speed,heading\n')
    events = open(os.path.join(args.out, 'events.csv'), 'w')
    events.write('t,robot,event\n')
    track = open(os.path.join(args.out, 'track.csv'), 'w')
    track.write('t,robot,x,y,heading_deg\n')

    def on_cmd(msg, r):
        d = json.loads(msg.data)
        cmds.write(f'{time.time():.4f},{r},{d.get("speed", 0)},{d.get("heading", 0)}\n')

    roll_pub, led_pub = {}, {}
    for r in robots:
        node.create_subscription(String, f'/sphero/{ns[r]}/motion_cmd',
                                 lambda m, r=r: on_cmd(m, r), 50)
        roll_pub[r] = node.create_publisher(String, f'/sphero/{ns[r]}/roll', 10)
        led_pub[r] = node.create_publisher(String, f'/sphero/{ns[r]}/led', 10)
    cam_pub = node.create_publisher(CompressedImage, '/patrol/camera/compressed', 10)
    ann_pub = node.create_publisher(CompressedImage, '/patrol/annotated/compressed', 5)
    trk_pub = node.create_publisher(String, '/patrol/tracks', 10)
    pose_pub = {r: node.create_publisher(PoseStamped, f'/patrol/{ns[r]}/pose', 10)
                for r in robots}
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    bag = subprocess.Popen(
        ['ros2', 'bag', 'record', '-s', 'mcap', '-o', os.path.join(args.out, 'bag'),
         '--topics', '/patrol/camera/compressed', '/patrol/annotated/compressed',
         '/patrol/tracks', *[f'/patrol/{ns[r]}/pose' for r in robots],
         *[f'/sphero/{ns[r]}/{t}' for r in robots
           for t in ('motion_cmd', 'roll', 'led', 'status', 'sensors')]],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3.0)                          # bag subscribes before anything moves

    def stamp(msg, t):
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int((t - int(t)) * 1e9)

    def pub_frame(t, buf):
        m = CompressedImage()
        stamp(m, t)
        m.header.frame_id = 'overhead_camera'
        m.format = 'jpeg'
        m.data = buf.tobytes()
        cam_pub.publish(m)

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
        events.write(f'{time.time():.4f},all,leds\n')

    rec = Recorder(args.device, args.out, on_frame=pub_frame)
    while rec.get() is None:
        time.sleep(0.05)
    trk = Tracker(rec, H, track)
    vel = {r: (0.0, 0.0) for r in robots}
    t_last = [time.time()]
    targets_now = {}
    t_ann = [0.0]

    def measure():
        """Full-res gray for blobs, half-res colour for the LED pair (step-6 scheme)."""
        t, buf = rec.get()
        g = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        col = cv2.imdecode(buf, cv2.IMREAD_REDUCED_COLOR_2)
        out = []
        for b in blobs.detect(g, trk.roi, trk.tmap):
            u, v = b['Centroid']
            h = color_decode.led_heading(col, u, v, is_hsv=False)
            out.append((*trk.to_cm(u, v), h, u, v))
        return t, out, col

    def publish(t, col, px):
        rows = []
        for r in robots:
            x, y, h = trk.pos[r]
            m = PoseStamped()
            stamp(m, t)
            m.header.frame_id = 'field'
            m.pose.position.x, m.pose.position.y = float(x), float(y)
            if h is not None:
                m.pose.orientation.z = math.sin(math.radians(h) / 2)
                m.pose.orientation.w = math.cos(math.radians(h) / 2)
            else:
                m.pose.orientation.w = 1.0
            pose_pub[r].publish(m)
            rows.append({'robot': r, 'x': round(x, 2), 'y': round(y, 2),
                         'heading_deg': None if h is None else round(h, 1),
                         'heading_src': 'led' if h is not None else None})
        trk_pub.publish(String(data=json.dumps({'t': round(t, 4), 'tracks': rows})))
        if t - t_ann[0] < 0.1:                           # annotated frames at 10 Hz
            return
        t_ann[0] = t
        img = col.copy()
        for r in robots:
            if r not in px:
                continue
            u, v = px[r][0] / 2, px[r][1] / 2
            h = trk.pos[r][2]
            cv2.circle(img, (int(u), int(v)), 12, (0, 255, 255), 1)
            cv2.putText(img, r[3:], (int(u) + 12, int(v) - 12), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (0, 255, 255), 1, cv2.LINE_AA)
            if h is not None:                            # field heading -> image arrow
                x, y, _ = trk.pos[r]
                tip = np.linalg.inv(H) @ [x + 12 * math.cos(math.radians(h)),
                                          y + 12 * math.sin(math.radians(h)), 1]
                cv2.arrowedLine(img, (int(u), int(v)), (int(tip[0] / tip[2] / 2),
                                int(tip[1] / tip[2] / 2)), (0, 255, 0), 2, tipLength=0.35)
            if r in targets_now:
                tq = np.linalg.inv(H) @ [*targets_now[r], 1]
                cv2.drawMarker(img, (int(tq[0] / tq[2] / 2), int(tq[1] / tq[2] / 2)),
                               (255, 0, 255), cv2.MARKER_CROSS, 10, 1)
        cv2.putText(img, f'{t:.2f}', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        m = CompressedImage()
        stamp(m, t)
        m.header.frame_id = 'overhead_camera_half'
        m.format = 'jpeg'
        m.data = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
        ann_pub.publish(m)

    def update(want_heading=False):
        """Global greedy nearest assignment with constant-velocity prediction."""
        t, meas, col = measure()
        dt = max(t - t_last[0], 1e-3)
        pred = {r: (trk.pos[r][0] + vel[r][0] * dt, trk.pos[r][1] + vel[r][1] * dt)
                for r in robots}
        pairs = sorted((math.hypot(m[0] - pred[r][0], m[1] - pred[r][1]), r, j)
                       for r in robots for j, m in enumerate(meas))
        used_r, used_m, pix = set(), set(), {}
        for d, r, j in pairs:
            if d > 25 or r in used_r or j in used_m:
                continue
            used_r.add(r)
            used_m.add(j)
            x, y, h = meas[j][:3]
            px, py, _ = trk.pos[r]
            vel[r] = (0.7 * vel[r][0] + 0.3 * (x - px) / dt, 0.7 * vel[r][1] + 0.3 * (y - py) / dt)
            trk.pos[r] = (x, y, h)
            pix[r] = meas[j][3:]
            track.write(f'{t:.4f},{r},{x:.2f},{y:.2f},{"" if h is None else f"{h:.1f}"}\n')
        t_last[0] = t
        publish(t, col, pix)
        return t

    def wait_still(rs, hold=0.5, max_s=6.0):
        hist, since, t_end = [], None, time.time() + max_s
        while time.time() < t_end:
            t = update()
            hist.append((t, {r: trk.pos[r][:2] for r in rs}))
            old = next((h for h in reversed(hist) if h[0] <= t - 0.25), hist[0])
            v = max(math.hypot(hist[-1][1][r][0] - old[1][r][0], hist[-1][1][r][1] - old[1][r][1])
                    / max(t - old[0], 1e-3) for r in rs)
            if v < 2.0 and t - hist[0][0] > 0.3:
                since = since or t
                if t - since >= hold:
                    return
            else:
                since = None

    def led_headings(n=12, timeout=4.0):
        hs, t_end = {r: [] for r in robots}, time.time() + timeout
        while time.time() < t_end and min(len(v) for v in hs.values() if v is not None) < n:
            update(want_heading=True)
            for r in robots:
                if trk.pos[r][2] is not None:
                    hs[r].append(math.radians(trk.pos[r][2]))
        return {r: (math.degrees(math.atan2(np.mean(np.sin(v)), np.mean(np.cos(v)))) % 360
                    if len(v) >= n // 2 else None) for r, v in hs.items()}

    offset = {}

    def refresh_offsets():
        for r, h in led_headings().items():
            if h is not None and r not in NO_LED:
                offset[r] = (h + last_cmd[r]) % 360
                event(r, f'offset_led {offset[r]:.1f}')

    def probe_offset(r):
        """Roll at cmd 0 until it has moved 6 cm, stop at once (little speed, little coast)."""
        x0, y0, _ = trk.pos[r]
        roll(r, SPEED, 0)
        t_end = time.time() + 3.0
        while time.time() < t_end:
            update()
            if math.hypot(trk.pos[r][0] - x0, trk.pos[r][1] - y0) >= 6:
                break
        roll(r, 0, 0)
        wait_still([r])
        x1, y1, _ = trk.pos[r]
        if math.hypot(x1 - x0, y1 - y0) < 4:
            raise SystemExit(f'{r}: did not move on the offset probe')
        offset[r] = math.degrees(math.atan2(y1 - y0, x1 - x0)) % 360
        event(r, f'offset_motion {offset[r]:.1f}')

    def leg(targets, name, circle=False):
        """Drive every robot in `targets` to its point at once (re-aimed every
        0.5 s, stopped COAST_CM short); the centre robot circles meanwhile."""
        start = {r: trk.pos[r][:2] for r in targets}
        targets_now.clear()
        targets_now.update(targets)
        active = {}
        for r, (tx, ty) in targets.items():
            sx, sy = start[r]
            if math.hypot(tx - sx, ty - sy) < 8:
                continue                                  # already there
            phi = math.degrees(math.atan2(ty - sy, tx - sx))
            roll(r, 0, (offset[r] - phi) % 360)          # face first
            active[r] = phi
        time.sleep(1.5)
        for r in active:
            event(r, f'{name} -> ({targets[r][0]:.1f},{targets[r][1]:.1f})')
            roll(r, SPEED, last_cmd[r])
        c_heading = None
        if circle:
            # top of the circle, heading east, turning clockwise (cmd increases)
            c_heading = (offset[CENTRE_ROBOT] - 0.0) % 360
            roll(CENTRE_ROBOT, SPEED, c_heading)
            event(CENTRE_ROBOT, f'{name} circle')
        t_aim = {r: time.time() for r in active}
        t_circle, t_end = time.time(), time.time() + 20.0
        moving = set(active)
        while moving and time.time() < t_end:
            update()
            now = time.time()
            for r in list(moving):
                x, y, _ = trk.pos[r]
                tx, ty = targets[r]
                rem = math.hypot(tx - x, ty - y)
                near_wall = min(x, y, ARENA_W - x, ARENA_H - y) < WALL_STOP_CM
                if rem <= COAST_CM or near_wall:
                    roll(r, 0, last_cmd[r])
                    moving.discard(r)
                    event(r, f'{name} stop rem={rem:.1f}' + (' WALL' if near_wall else ''))
                elif now - t_aim[r] >= 0.5:               # device applies ~2 Hz
                    phi = math.degrees(math.atan2(ty - y, tx - x))
                    cmd = (offset[r] - phi) % 360
                    if abs((cmd - last_cmd[r] + 180) % 360 - 180) > 3:
                        roll(r, SPEED, cmd)
                    t_aim[r] = now
            if circle and now - t_circle >= 0.5:
                c_heading = (c_heading + 45) % 360
                roll(CENTRE_ROBOT, SPEED, c_heading)
                t_circle = now
        for r in moving:
            roll(r, 0, last_cmd[r])
            event(r, f'{name} stop on timeout')
        if circle:
            roll(CENTRE_ROBOT, 0, c_heading)
        wait_still(robots)
        for r in targets:
            x, y, _ = trk.pos[r]
            event(r, f'{name} end ({x:.1f},{y:.1f}) err {math.hypot(x - targets[r][0], y - targets[r][1]):.1f}')

    try:
        # current positions: robots on their marks; SB-1FA8 / SB-BBBE where the
        # crossing test left them -- snap every robot to the nearest blob
        _, meas, _ = measure()
        guess = dict(CROSSES + LINES + [(CENTRE_ROBOT, CENTRE)])
        guess.update({CENTRE_ROBOT: (163.2, 74.9), 'SB-BBBE': (170.6, 74.2),
                      'SB-3660': (32.3, 51.8)})
        free = list(range(len(meas)))
        for r in robots:
            j = min(free, key=lambda k: math.hypot(meas[k][0] - guess[r][0], meas[k][1] - guess[r][1]))
            if math.hypot(meas[j][0] - guess[r][0], meas[j][1] - guess[r][1]) > 15:
                raise SystemExit(f'{r}: no blob within 15 cm of {guess[r]}')
            trk.pos[r] = meas[j][:3]
            free.remove(j)
        leds_on()
        time.sleep(2.0)
        leds_on()
        for r in robots:
            roll(r, 0, 0)
        time.sleep(2.0)
        wait_still(robots)
        refresh_offsets()
        for r in robots:
            if r not in offset:
                probe_offset(r)

        centre_top = (CENTRE[0], CENTRE[1] + CIRCLE_R)
        home = dict(CROSSES + LINES + [(CENTRE_ROBOT, centre_top)])
        leg(home, 'to-start')
        leds_on()
        time.sleep(1.5)
        refresh_offsets()

        corner_pts = [p for _, p in CROSSES]
        corner_idx = {r: i for i, (r, _) in enumerate(CROSSES)}
        side = {r: 0 for r, _ in LINES}
        for k in range(4 * args.laps):
            tg = {}
            for r, i in corner_idx.items():
                corner_idx[r] = (i + 1) % 4
                tg[r] = corner_pts[corner_idx[r]]
            (ra, pa), (rb, pb) = LINES
            side[ra] ^= 1
            side[rb] ^= 1
            tg[ra] = (pb[0] if side[ra] else pa[0], pa[1])   # x of the other line, own lane
            tg[rb] = (pa[0] if side[rb] else pb[0], pb[1])
            print(f'leg {k + 1}/{4 * args.laps}')
            leg(tg, f'leg{k + 1}', circle=True)
            leds_on()                                    # between legs only
            time.sleep(1.5)
            refresh_offsets()
    finally:
        for r in robots:
            roll(r, 0, last_cmd[r])
        time.sleep(0.5)
        rec.close()
        bag.send_signal(signal.SIGINT)
        try:
            bag.wait(timeout=15)
        except subprocess.TimeoutExpired:
            bag.kill()
        for f in (cmds, events, track):
            f.close()
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    print(f'{rec.n} frames -> {args.out}')


if __name__ == '__main__':
    main()

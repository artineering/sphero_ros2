#!/usr/bin/env python3
"""Record two Spheros crossing, passing close and colliding, for offline tracker tests.

Per pair, from wherever the two robots stand (both must be clear of the walls
and of the other robots' paths):
  head-on   both aim at the midpoint M and drive through it (meet at M)
  pass      both aim at points 5 cm either side of M (centres ~10 cm apart)
  right     both aim at P on the circle with diameter AB, so the paths meet at
            90 deg (Thales), each overshooting P by --overshoot cm
Scenarios run in that order; robots stay where each one leaves them, so the
next scenario starts from the new positions. Speed is fixed at 50 (user rule:
never below 50) and each robot stops at D + overshoot - coast(50).

Heading offsets: field = offset - cmd. Measured at the start from the LED pair
after roll(0, 0) (as goto_target does); a robot without a readable LED pair
(e.g. SB-3660) gets it from the direction of a short roll at cmd 0.

LEDs (main off, front green, back red) are written only between moves, never
while a robot rolls -- LED writes share the BLE link and delay stops.

  <out>/raw.mjpeg   every camera frame          <out>/frames.csv  frame, t
  <out>/cmds.csv    t, robot, speed, heading    (from /sphero/<ns>/motion_cmd)
  <out>/events.csv  t, robot, event             (scenario markers, LED writes, offsets)
  <out>/track.csv   t, robot, x, y, heading_deg (this script's live tracking, field cm)

Usage (ROS sourced): python3 scripts/crossing_record.py --pair SB-AAAD SB-E12C
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

sys.path.insert(0, os.path.expanduser('~/sphero_ros2/src/overhead_tracking'))
from overhead_tracking import blobs, heading as hdg  # noqa: E402

from blob_detect_timing import ARENA_PX, open_camera  # noqa: E402

SPEED = 50
COAST_CM = 42.9            # stop -> rest at speed 50 (stop_sweep, 2026-10-05)
ARENA_W, ARENA_H = 243.84, 182.88
WALL_MARGIN = 15.0         # path end points stay this far inside the walls
CLEAR_CM = 20.0            # other robots must be this far from both paths


class Recorder:
    """Camera thread: writes every frame, keeps the latest buffer for control."""

    def __init__(self, device, out, on_frame=None):
        self.on_frame = on_frame             # called with (t, jpeg bytes) per frame
        self.cap = open_camera(device)
        for _ in range(30):
            self.cap.read()
        self.raw = open(os.path.join(out, 'raw.mjpeg'), 'wb')
        self.frames = open(os.path.join(out, 'frames.csv'), 'w')
        self.frames.write('frame,t\n')
        self.lock = threading.Lock()
        self.latest = None
        self.stop = threading.Event()
        self.n = 0
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not self.stop.is_set():
            ok, buf = self.cap.read()
            if not ok:
                continue
            t = time.time()
            self.raw.write(buf.tobytes())
            self.frames.write(f'{self.n},{t:.4f}\n')
            with self.lock:
                self.latest = (t, buf.copy())
            if self.on_frame is not None:
                self.on_frame(t, buf)
            self.n += 1

    def get(self):
        with self.lock:
            return self.latest

    def close(self):
        self.stop.set()
        time.sleep(0.2)
        self.cap.release()
        self.raw.close()
        self.frames.close()


class Tracker:
    """Nearest-blob tracking of every robot (only 1-2 move at a time)."""

    def __init__(self, rec, H, track_log):
        self.rec, self.H, self.log = rec, H, track_log
        g = cv2.imdecode(rec.get()[1], cv2.IMREAD_GRAYSCALE)
        self.roi = blobs.make_roi(ARENA_PX, g.shape)
        self.tmap = blobs.threshold_map([g], self.roi)
        self.pos = {}

    def to_cm(self, u, v):
        p = self.H @ np.array([u, v, 1.0])
        return p[0] / p[2], p[1] / p[2]

    def measure(self, want_heading=False):
        """[(x, y, heading_deg or None)] for the latest frame."""
        t, buf = self.rec.get()
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR if want_heading else cv2.IMREAD_GRAYSCALE)
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if want_heading else img
        out = []
        for b in blobs.detect(g, self.roi, self.tmap):
            u, v = b['Centroid']
            h = None
            if want_heading:
                red = hdg.find_led(img, u, v, hdg.RED_HUE, 30, 90, 60)
                green = hdg.find_led(img, u, v, hdg.GREEN_HUE, 30, 90, 60)
                if red is not None and green is not None:
                    h = hdg.heading_from_pair(red, green, 13)
            out.append((*self.to_cm(u, v), h))
        return t, out

    def update(self, want_heading=False):
        t, meas = self.measure(want_heading)
        for r, (x, y, _) in list(self.pos.items()):
            if not meas:
                break
            j = min(range(len(meas)), key=lambda k: math.hypot(meas[k][0] - x, meas[k][1] - y))
            mx, my, mh = meas[j]
            if math.hypot(mx - x, my - y) < 25.0:
                self.pos[r] = (mx, my, mh)
                self.log.write(f'{t:.4f},{r},{mx:.2f},{my:.2f},'
                               f'{"" if mh is None else f"{mh:.1f}"}\n')
        return t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pair', nargs=2, required=True)
    ap.add_argument('--start', nargs=4, type=float, metavar=('XA', 'YA', 'XB', 'YB'),
                    help='field cm of the two robots now; each snaps to the nearest blob')
    ap.add_argument('--home', nargs=4, type=float, metavar=('XA', 'YA', 'XB', 'YB'),
                    help='field cm to return the pair to between scenarios '
                         '(default: where they start)')
    ap.add_argument('--scenarios', nargs='+', default=['head-on', 'pass', 'right'])
    ap.add_argument('--overshoot', type=float, default=25.0)
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--out', default=os.path.expanduser(
        f'~/sphero_ros2/recordings/cross_{time.strftime("%Y%m%d_%H%M%S")}'))
    args = ap.parse_args()
    if args.start is None:
        ap.error('--start XA YA XB YB is required (field cm of the pair now)')
    os.makedirs(args.out, exist_ok=True)
    A, B = args.pair
    ns = {r: r.replace('-', '_') for r in args.pair}
    H = np.array(yaml.safe_load(open(os.path.expanduser(
        '~/overhead_field/overhead_arena.yaml')))['H'])

    rclpy.init()
    node = rclpy.create_node('crossing_record')
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
    for r in args.pair:
        node.create_subscription(String, f'/sphero/{ns[r]}/motion_cmd',
                                 lambda m, r=r: on_cmd(m, r), 50)
        roll_pub[r] = node.create_publisher(String, f'/sphero/{ns[r]}/roll', 10)
        led_pub[r] = node.create_publisher(String, f'/sphero/{ns[r]}/led', 10)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(1.0)

    last_cmd = {r: 0 for r in args.pair}

    def roll(r, speed, h):
        last_cmd[r] = int(round(h)) % 360
        roll_pub[r].publish(String(data=json.dumps({'heading': last_cmd[r], 'speed': speed})))

    def event(r, what):
        events.write(f'{time.time():.4f},{r},{what}\n')
        print(f'  {r}: {what}')

    def leds_on():
        for r in args.pair:
            for kind, rgb in (('main', (0, 0, 0)), ('front', (0, 255, 0)), ('back', (255, 0, 0))):
                led_pub[r].publish(String(data=json.dumps(
                    {'type': kind, 'red': rgb[0], 'green': rgb[1], 'blue': rgb[2]})))
            event(r, 'leds')

    rec = Recorder(args.device, args.out)
    while rec.get() is None:
        time.sleep(0.05)
    trk = Tracker(rec, H, track)
    trk.pos = {A: (args.start[0], args.start[1], None), B: (args.start[2], args.start[3], None)}
    _, meas = trk.measure()
    for r in args.pair:                      # snap to the nearest blob; the rest are obstacles
        x, y, _ = trk.pos[r]
        j = min(range(len(meas)), key=lambda k: math.hypot(meas[k][0] - x, meas[k][1] - y))
        if math.hypot(meas[j][0] - x, meas[j][1] - y) > 15:
            raise SystemExit(f'{r}: no blob within 15 cm of {x:.0f},{y:.0f}')
        trk.pos[r] = meas[j]
    others = [m[:2] for m in meas if all(math.hypot(m[0] - p[0], m[1] - p[1]) > 1
                                          for p in trk.pos.values())]

    def wait_still(rs, hold=0.5, max_s=5.0):
        hist, since, t_end = [], None, time.time() + max_s
        while time.time() < t_end:
            t = trk.update()
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

    def led_heading(r, n=15, timeout=4.0):
        hs, t_end = [], time.time() + timeout
        while len(hs) < n and time.time() < t_end:
            trk.update(want_heading=True)
            if trk.pos[r][2] is not None:
                hs.append(math.radians(trk.pos[r][2]))
        if len(hs) < n // 2:
            return None
        return math.degrees(math.atan2(np.mean(np.sin(hs)), np.mean(np.cos(hs)))) % 360

    offset = {}
    try:
        leds_on()
        time.sleep(2.0)
        leds_on()                            # BLE drops LED writes now and then
        for r in args.pair:
            roll(r, 0, 0)
        time.sleep(2.0)
        wait_still(args.pair)
        for r in args.pair:
            h = led_heading(r)
            if h is not None:
                offset[r] = h                # cmd 0 -> field = offset
                event(r, f'offset_led {h:.1f}')
        for r in args.pair:
            if r in offset:
                continue
            # no LED pair: roll briefly at cmd 0 toward wherever it points and
            # read the direction it moved (only if it has room in every direction)
            x0, y0, _ = trk.pos[r]
            if min(x0, y0, ARENA_W - x0, ARENA_H - y0) < 60:
                raise SystemExit(f'{r}: no LED heading and too close to a wall to probe')
            roll(r, SPEED, 0)
            time.sleep(1.0)
            roll(r, 0, 0)
            wait_still([r])
            x1, y1, _ = trk.pos[r]
            if math.hypot(x1 - x0, y1 - y0) < 10:
                raise SystemExit(f'{r}: did not move on the offset probe')
            offset[r] = math.degrees(math.atan2(y1 - y0, x1 - x0)) % 360
            event(r, f'offset_motion {offset[r]:.1f}')

        def check_identity():
            """After contact the nearest-blob tracker may have swapped A and B: each
            robot faces field = offset - last cmd, so keep the labelling whose LED
            headings fit that best."""
            hs = {r: led_heading(r, n=8, timeout=2.0) for r in args.pair}
            if any(h is None for h in hs.values()):
                return
            exp = {r: (offset[r] - last_cmd[r]) % 360 for r in args.pair}

            def err(a, b):
                return abs((a - b + 180) % 360 - 180)
            same = err(hs[A], exp[A]) + err(hs[B], exp[B])
            swap = err(hs[B], exp[A]) + err(hs[A], exp[B])
            if swap + 30 < same:
                trk.pos[A], trk.pos[B] = trk.pos[B], trk.pos[A]
                event(A, f'identity swapped back (same {same:.0f} vs swap {swap:.0f} deg)')

        def drive(targets, name, overshoot):
            """targets: {robot: (tx, ty)}; drive both at once, stop each at D + overshoot - coast."""
            start = {r: trk.pos[r][:2] for r in targets}
            plan = {}
            for r, (tx, ty) in targets.items():
                sx, sy = start[r]
                d = math.hypot(tx - sx, ty - sy)
                phi = math.degrees(math.atan2(ty - sy, tx - sx))
                ex_, ey_ = (sx + (d + overshoot) * math.cos(math.radians(phi)),
                            sy + (d + overshoot) * math.sin(math.radians(phi)))
                if not (WALL_MARGIN <= ex_ <= ARENA_W - WALL_MARGIN
                        and WALL_MARGIN <= ey_ <= ARENA_H - WALL_MARGIN):
                    print(f'  {name}: {r} would end at ({ex_:.0f},{ey_:.0f}), too near a wall; skipped')
                    return False
                for ox, oy in others:        # clear of the parked robots?
                    ux, uy = math.cos(math.radians(phi)), math.sin(math.radians(phi))
                    along = (ox - sx) * ux + (oy - sy) * uy
                    if -10 < along < d + overshoot + 10 and \
                            abs(-(ox - sx) * uy + (oy - sy) * ux) < CLEAR_CM:
                        print(f'  {name}: robot at ({ox:.0f},{oy:.0f}) is in {r}\'s path; skipped')
                        return False
                plan[r] = (phi, d + overshoot - COAST_CM)
            for r, (phi, _) in plan.items():  # face first, outside the move
                roll(r, 0, (offset[r] - phi) % 360)
            time.sleep(1.5)
            wait_still(list(plan))
            start = {r: trk.pos[r][:2] for r in plan}
            for r in plan:
                event(r, f'{name} start target=({targets[r][0]:.1f},{targets[r][1]:.1f})')
            for r, (phi, _) in plan.items():
                roll(r, SPEED, (offset[r] - phi) % 360)
            moving, t_end = set(plan), time.time() + 8.0
            while moving and time.time() < t_end:
                trk.update()
                for r in list(moving):
                    phi, stop_at = plan[r]
                    sx, sy = start[r]
                    x, y, _ = trk.pos[r]
                    cov = ((x - sx) * math.cos(math.radians(phi))
                           + (y - sy) * math.sin(math.radians(phi)))
                    if cov >= max(stop_at, 0.0):
                        roll(r, 0, last_cmd[r])
                        moving.discard(r)
                        event(r, f'{name} stop at {cov:.1f} cm')
            for r in moving:
                roll(r, 0, last_cmd[r])
                event(r, f'{name} stop on timeout')
            wait_still(list(plan))
            check_identity()
            for r in plan:
                x, y, _ = trk.pos[r]
                event(r, f'{name} end ({x:.1f},{y:.1f})')
            return True

        home = ({A: tuple(args.home[:2]), B: tuple(args.home[2:])} if args.home
                else {r: trk.pos[r][:2] for r in args.pair})
        for sc in args.scenarios:
            if min(math.hypot(trk.pos[r][0] - home[r][0], trk.pos[r][1] - home[r][1])
                   for r in args.pair) > 15:
                drive(home, 'return', 0.0)   # back apart before the next scenario
                leds_on()
                time.sleep(1.5)
            (ax, ay, _), (bx, by, _) = trk.pos[A], trk.pos[B]
            mx, my = (ax + bx) / 2, (ay + by) / 2
            dab = math.hypot(bx - ax, by - ay)
            nx, ny = -(by - ay) / dab, (bx - ax) / dab       # unit normal to AB
            print(f'{sc}: A ({ax:.0f},{ay:.0f}) B ({bx:.0f},{by:.0f}), {dab:.0f} cm apart')
            if dab < 70:
                print(f'  {sc}: robots only {dab:.0f} cm apart (need >= 70); skipped')
                continue
            if sc == 'head-on':
                tg = {A: (mx, my), B: (mx, my)}
            elif sc == 'pass':
                tg = {A: (mx + 5 * nx, my + 5 * ny), B: (mx - 5 * nx, my - 5 * ny)}
            else:
                tg = None
                for s in (1, -1):
                    px, py = mx + s * dab / 2 * nx, my + s * dab / 2 * ny
                    if WALL_MARGIN <= px <= ARENA_W - WALL_MARGIN and \
                            WALL_MARGIN <= py <= ARENA_H - WALL_MARGIN:
                        tg = {A: (px, py), B: (px, py)}
                        break
                if tg is None:
                    print('  right: the 90 deg meeting point is outside the arena; skipped')
                    continue
            drive(tg, sc, args.overshoot)
            leds_on()                        # between moves only
            time.sleep(1.5)
    finally:
        for r in args.pair:
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

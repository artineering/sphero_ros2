#!/usr/bin/env python3
"""Drive one Sphero to a field target (cm) with the overhead camera in the loop.

1. Light the robot's front (green) / back (red) LEDs; the only blob showing the
   pair is the robot. Initial position = blob centroid through the arena
   homography; initial heading = LED back->front bearing (heading.py).
2. Heading offset: command heading 0 at speed 0, measure the LED heading.
   Sphero headings are clockwise, field angles counter-clockwise, so
   field = offset - cmd, and to travel along field bearing phi: cmd = offset - phi.
   The offset is refined while driving straight.
3. Closed loop at 2 Hz (what the device controller sustains): steer at the
   target at speed <= 40, stop when the remaining distance reaches the
   stopping distance for the camera-measured speed. State from the EKF tested
   in kf_replay.py (blob position + LED heading, inputs v and a rate-limited
   omega).
4. Settle, then pulse corrections until within --tol-frac of the start-to-
   target distance (max --attempts): face the target at speed 0, roll at
   speed 25 for T s, stop, settle. T = error / gain, and the gain (cm moved per
   s of pulse) is re-learned from every pulse's measured displacement.

Stopping distance (measured 2026-10-05 on SB-418F, stop sent -> robot still):
24 cm/s -> 16.8 cm, 31 -> 20, 56 -> 78. The stop takes 0.7-1.2 s to be applied
over BLE, then the ball coasts; fitted d = 0.186 v + 0.0214 v^2 (cm, cm/s).
Speed 70 coasting 78 cm is why the approach is capped at 40.

Coordinates: targets and printed positions/headings use the USER frame --
(0,0) at the top-right corner of the camera image, (243.84, 182.88) at the
bottom-left, x leftward, y downward. See flip().

Usage (ROS sourced):
  python3 scripts/goto_target.py --robot SB-418F --target 122 91
"""
import argparse
import json
import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

from blob_detect_timing import detect, make_roi, open_camera, threshold_map
from kf_replay import EKF, hdg, wrap

SPEED_TO_CMS = 0.61
SPHERE_R_CM = 3.65                # sphere_radius_mm 36.5, overhead_tracking.yaml
MIN_SPEED = 50                    # user rule 2026-10-05: never command 0 < speed < 50

# USER convention (targets in, positions/headings out): origin at the TOP-RIGHT
# corner of the camera image, x growing leftward, y growing downward -- the
# calibration's field frame (origin bottom-left, x right, y up) rotated 180 deg.
# The conversion is its own inverse. Internals and log.csv stay in the field
# frame, which is what the homography and the tracker node use.
ARENA_W, ARENA_H = 243.84, 182.88


def flip(x, y):
    """field <-> user (cm); same formula both ways."""
    return ARENA_W - x, ARENA_H - y


def user_deg(th_field_rad):
    return (math.degrees(th_field_rad) + 180.0) % 360


def stop_dist(v):
    return 0.186 * v + 0.0214 * v * v


LAG_V, TURN_RATE = 0.45, math.radians(180)        # fitted in kf_replay.py
R_POS, R_TH, Q_POS, Q_TH = 0.05, 0.001, 1000.0, 0.1


class Camera:
    """Latest robot measurement (x_cm, y_cm, heading_rad or None) per frame."""

    def __init__(self, device, H):
        self.cap = open_camera(device)
        for _ in range(30):
            self.cap.read()
        self.H = H
        self.roi = make_roi()
        g = cv2.imdecode(self.cap.read()[1], cv2.IMREAD_GRAYSCALE)
        self.tmap = threshold_map([g], self.roi, 3.2)

    def to_cm(self, u, v):
        p = self.H @ np.array([u, v, 1.0])
        return p[0] / p[2], p[1] / p[2]

    def frame(self):
        ok, buf = self.cap.read()
        if not ok:
            return None, []
        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        out = []
        for b in detect(g, self.roi, self.tmap, 200):
            u, v = b['Centroid']
            red = hdg.find_led(bgr, u, v, hdg.RED_HUE, 30, 90, 60)
            green = hdg.find_led(bgr, u, v, hdg.GREEN_HUE, 30, 90, 60)
            h = (hdg.heading_from_pair(red, green, 13)
                 if red is not None and green is not None else None)
            out.append((*self.to_cm(u, v), None if h is None else math.radians(h), u, v))
        return bgr, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot', default='SB-418F')
    ap.add_argument('--target', nargs=2, type=float, default=[121.92, 91.44],
                    help='cm, USER frame: (0,0) top-right of the image, x left, y down')
    ap.add_argument('--tol-frac', type=float, default=0.05)
    ap.add_argument('--attempts', type=int, default=6)
    ap.add_argument('--fast', action='store_true',
                    help='minimum-time mode: one continuous loop, speed 0 only on arrival')
    ap.add_argument('--speed', type=int, default=50,
                    help='--fast: cruise speed; stop at D - measured coast(speed)')
    ap.add_argument('--face', action='store_true',
                    help='only turn in place to point at --target, then exit')
    ap.add_argument('--cmd-heading', type=float,
                    help="robot's current commanded heading, if known: skips the 2 s "
                         'heading-0 offset step (offset = LED heading + this)')
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--out', default=os.path.expanduser(
        f'~/sphero_ros2/recordings/goto_{time.strftime("%Y%m%d_%H%M%S")}'))
    args = ap.parse_args()
    if args.speed < MIN_SPEED:
        ap.error(f'--speed must be >= {MIN_SPEED}')
    os.makedirs(args.out, exist_ok=True)
    ns = args.robot.replace('-', '_')
    arena = yaml.safe_load(open(os.path.expanduser('~/overhead_field/overhead_arena.yaml')))
    H = np.array(arena['H'])
    # Usable field = arena inset by one Sphero radius: the ball's centre cannot
    # reach the wall line, and paths that graze a wall lose speed to friction and
    # stop short (2026-10-05 corner runs). The calibration itself is untouched --
    # it defines the field frame for the tracker node.
    lo = SPHERE_R_CM
    hi_x = arena['long_edge_len_cm'] - SPHERE_R_CM
    hi_y = arena['short_edge_len_cm'] - SPHERE_R_CM
    ux = float(np.clip(args.target[0], lo, hi_x))          # user frame
    uy = float(np.clip(args.target[1], lo, hi_y))
    if (ux, uy) != tuple(args.target):
        print(f'target ({args.target[0]:g}, {args.target[1]:g}) -> ({ux:.2f}, {uy:.2f}) cm: '
              f'kept {SPHERE_R_CM} cm (one Sphero radius) inside the walls')
    tx, ty = flip(ux, uy)                                   # field frame internally

    rclpy.init()
    node = rclpy.create_node('goto_target')
    roll_pub = node.create_publisher(String, f'/sphero/{ns}/roll', 10)
    led_pub = node.create_publisher(String, f'/sphero/{ns}/led', 10)
    applied = {'t': None, 'speed': 0.0, 'heading': 0.0}   # last motion_cmd echo

    def on_cmd(msg):
        d = json.loads(msg.data)
        applied.update(t=time.time(), speed=float(d.get('speed', 0)),
                       heading=float(d.get('heading', 0)))
    node.create_subscription(String, f'/sphero/{ns}/motion_cmd', on_cmd, 10)
    link = {'state': None}

    def on_status(msg):
        try:
            link['state'] = json.loads(msg.data).get('connection_state')
        except ValueError:
            pass
    node.create_subscription(String, f'/sphero/{ns}/status', on_status, 10)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    t_wait = time.time() + 8.0
    while link['state'] is None and time.time() < t_wait:
        time.sleep(0.1)
    if link['state'] != 'connected':
        raise SystemExit(f'{args.robot} is not connected (status: {link["state"]})')

    last_heading = [0]

    def roll(speed, heading):
        last_heading[0] = int(round(heading)) % 360
        roll_pub.publish(String(data=json.dumps({'speed': int(speed),
                                                 'heading': last_heading[0]})))

    def leds():
        for kind, rgb in (('main', (0, 0, 0)), ('front', (0, 255, 0)), ('back', (255, 0, 0))):
            led_pub.publish(String(data=json.dumps(
                {'type': kind, 'red': rgb[0], 'green': rgb[1], 'blue': rgb[2]})))

    stop = threading.Event()

    def keep_lit():                                        # BLE LED writes get dropped
        while not stop.is_set():
            leds()
            stop.wait(2.0)
    threading.Thread(target=keep_lit, daemon=True).start()

    cam = Camera(args.device, H)
    log = open(os.path.join(args.out, 'log.csv'), 'w')
    log.write('t,phase,meas_x,meas_y,meas_th_deg,ekf_x,ekf_y,ekf_th_deg,cmd_speed,cmd_heading\n')
    t_start = time.time()

    def lit_robot(timeout):
        """Wait for exactly one blob with an LED pair; returns it."""
        t_end = time.time() + timeout
        while time.time() < t_end:
            _, blobs = cam.frame()
            lit = [b for b in blobs if b[2] is not None]
            if len(lit) == 1:
                return lit[0]
        raise SystemExit('robot LEDs not seen (exactly one lit blob expected)')

    def median_heading(n, near):
        hs, xs = [], []
        while len(hs) < n:
            _, blobs = cam.frame()
            b = min(blobs, key=lambda b: math.hypot(b[0] - near[0], b[1] - near[1]))
            if b[2] is not None and math.hypot(b[0] - near[0], b[1] - near[1]) < 10:
                hs.append(b[2])
                xs.append(b[:2])
        mean = math.atan2(np.mean(np.sin(hs)), np.mean(np.cos(hs)))
        return mean, tuple(np.median(xs, axis=0))

    # ---- 1. initial position and heading
    try:
        b0 = lit_robot(15.0)
        th0, p0 = median_heading(30, b0[:2])
        print('initial position (%.1f, %.1f) cm, heading %.1f deg' % (*flip(*p0), user_deg(th0)))

        # ---- 2. heading offset (field = offset - cmd)
        if args.cmd_heading is not None:
            th_aim = th0
            offset = th0 + math.radians(args.cmd_heading)
            print(f'robot at commanded heading {args.cmd_heading:g} -> offset '
                  f'{math.degrees(offset) % 360:.1f} deg')
        else:
            roll(0, 0)
            time.sleep(2.0)
            th_aim, p0 = median_heading(30, p0)
            offset = th_aim
            print(f'after heading-0 command: heading {user_deg(th_aim):.1f} deg')

        if args.face:
            # turn in place (speed 0) toward the target and confirm with the LEDs
            # field = offset - cmd, so a measured error e is fixed by cmd += e.
            # Small turns in place are not executed reliably (2026-10-05: a 4.5 deg
            # correction moved 0.3 deg, a 6 deg one moved 10.5), so every
            # correction swings 30 deg away first and comes back as a large turn.
            phi = math.atan2(ty - p0[1], tx - p0[0])
            cmd = math.degrees(offset - phi) % 360
            for k in range(1, 4):
                if k > 1:
                    roll(0, cmd + 30)
                    time.sleep(1.0)
                roll(0, cmd)
                time.sleep(1.5)
                th, _ = median_heading(30, p0)
                e = math.degrees(wrap(th - phi))
                print(f'face ({ux:.1f}, {uy:.1f}) try {k}: wanted {user_deg(phi):.1f} deg, '
                      f'measured {user_deg(th):.1f} deg, error {e:+.1f} deg')
                if abs(e) < 2.0:
                    break
                cmd = (cmd + e) % 360
            raise SystemExit(0)

        dist0 = math.hypot(tx - p0[0], ty - p0[1])
        tol = args.tol_frac * dist0
        print(f'target ({ux:.1f}, {uy:.1f}) cm, distance {dist0:.1f} cm, tolerance {tol:.2f} cm')

        ekf = EKF((p0[0], p0[1], th_aim), Q_POS, Q_TH, R_POS, R_TH)
        cmd_hist = [(time.time(), 0.0, 0.0)]               # (t, v cm/s, cmd heading deg)
        state = {'last_t': time.time(), 'prev_hc': None, 'pending': 0.0}
        hist = []

        def step():
            """One camera frame through the EKF. Returns the matched measurement."""
            _, blobs = cam.frame()
            now = time.time()
            dt = now - state['last_t']
            state['last_t'] = now
            # speed input LAG_V late; heading input as applied (motion_cmd echo)
            v = next((c[1] for c in reversed(cmd_hist) if c[0] <= now - LAG_V), 0.0)
            hc = offset - math.radians(applied['heading'])
            if state['prev_hc'] is not None:
                state['pending'] += wrap(hc - state['prev_hc'])
            state['prev_hc'] = hc
            w = max(-TURN_RATE, min(TURN_RATE, state['pending'] / max(dt, 1e-3)))
            state['pending'] -= w * dt
            ekf.predict(v, w, dt)
            near = [b for b in blobs if math.hypot(b[0] - ekf.x[0], b[1] - ekf.x[1]) < 15]
            m = min(near, key=lambda b: math.hypot(b[0] - ekf.x[0], b[1] - ekf.x[1]),
                    default=None)
            if m is not None:
                ekf.update([m[0], m[1]] + ([m[2]] if m[2] is not None else []))
            hist.append((now, ekf.x[0], ekf.x[1]))
            return now, m

        def speed_meas(window=0.2):
            old = next((h for h in reversed(hist) if h[0] <= hist[-1][0] - window), hist[0])
            dt = hist[-1][0] - old[0]
            return math.hypot(hist[-1][1] - old[1], hist[-1][2] - old[2]) / dt if dt > 0 else 0.0

        def write(now, phase, m, speed, heading):
            mx = ('%.2f,%.2f,%s' % (m[0], m[1], '' if m[2] is None else '%.1f' % math.degrees(m[2]))
                  if m else ',,')
            log.write(f'{now - t_start:.3f},{phase},{mx},{ekf.x[0]:.2f},{ekf.x[1]:.2f},'
                      f'{math.degrees(ekf.x[2]):.1f},{speed},{heading}\n')

        def drive(phase, v_max, v_min, timeout):
            """Closed loop to the target; returns when stopped early or timed out."""
            nonlocal offset
            t_end, last_ctl, speed, heading = time.time() + timeout, 0.0, 0, 0
            t_heading_set = time.time()
            while time.time() < t_end:
                now, m = step()
                d = math.hypot(tx - ekf.x[0], ty - ekf.x[1])
                # stop when the remaining distance is what the robot will coast
                if d <= max(tol * 0.5, stop_dist(speed_meas())):
                    roll(0, heading)
                    cmd_hist.append((time.time(), 0.0, heading))
                    write(now, phase, m, 0, heading)
                    return
                # refine the offset while rolling straight on one command
                if (m is not None and m[2] is not None and speed > 0 and
                        now - t_heading_set > 0.8):
                    offset += 0.02 * wrap(m[2] - (offset - math.radians(heading)))
                if now - last_ctl >= 0.5:                  # 2 Hz control
                    last_ctl = now
                    phi = math.atan2(ty - ekf.x[1], tx - ekf.x[0])
                    new_heading = math.degrees(offset - phi) % 360
                    speed = int(np.clip(0.8 * d / SPEED_TO_CMS, v_min, v_max))
                    if abs(wrap(math.radians(new_heading - heading))) > math.radians(3):
                        t_heading_set = now
                    heading = new_heading
                    roll(speed, heading)
                    cmd_hist.append((now, speed * SPEED_TO_CMS, heading))
                write(now, phase, m, speed, heading)
            roll(0, heading)

        def settle(phase, secs=2.0):
            t_end = time.time() + secs
            pts = []
            while time.time() < t_end:
                now, m = step()
                write(now, phase, m, 0, 0)
                if m is not None and time.time() > t_end - 0.5:
                    pts.append(m[:2])
            return tuple(np.median(pts, axis=0)) if pts else tuple(ekf.x[:2])

        def pulse(k, gain):
            """Face the target, roll briefly, stop, settle. Returns (pos, moved_cm, T)."""
            p_before = settle(f'face{k}', 0.1)
            phi = math.atan2(ty - p_before[1], tx - p_before[0])
            heading = math.degrees(offset - phi) % 360
            err = math.hypot(tx - p_before[0], ty - p_before[1])
            roll(0, heading)                               # turn in place first
            settle(f'face{k}', 1.2)
            T = float(np.clip(err / gain, 0.1, 0.8))
            roll(MIN_SPEED, heading)
            t_end = time.time() + T
            while time.time() < t_end:
                now, m = step()
                write(now, f'pulse{k}', m, MIN_SPEED, heading)
            roll(0, heading)
            p_after = settle(f'settle{k}', 2.5)
            moved = ((p_after[0] - p_before[0]) * math.cos(phi) +
                     (p_after[1] - p_before[1]) * math.sin(phi))
            return p_after, moved, T

        def wait_still(phase, max_s=2.5, v_still=1.5, hold=0.3):
            """Step until the robot is still for `hold` s (or max_s). Returns position."""
            t_end, since = time.time() + max_s, None
            while time.time() < t_end:
                now, m = step()
                write(now, phase, m, 0, 0)
                if speed_meas(0.15) < v_still:
                    since = since or now
                    if now - since >= hold:
                        break
                else:
                    since = None
            return tuple(ekf.x[:2])

        def drive_fast(timeout):
            """Minimum time. Cruise at ONE speed (--speed) and send stop once the
            start->target line is covered up to D - coast(speed), with coast
            interpolated from the measured stopping distances. Residual error: pulses
            that allow for the ~0.6 s BLE dead time, moved = gain * (T - DEAD).
            The heading offset stays at its calibrated value."""
            DEAD = 0.6
            PULSE_SPEED = MIN_SPEED                        # nothing below 50 (user rule)
            t_end = time.time() + timeout
            sx, sy = ekf.x[0], ekf.x[1]
            D = math.hypot(tx - sx, ty - sy)
            phi = math.atan2(ty - sy, tx - sx)
            heading = math.degrees(offset - phi) % 360
            speed = args.speed
            # stop sent -> robot at rest, measured by the camera in stop_sweep.py
            # (2026-10-05, SB-418F, 100 cm moves): 30 -> 21.5 +/- 2.2 cm,
            # 50 -> 42.9 +/- 3.8, 100 -> 80.5 +/- 5.8. A fixed distance per speed,
            # not a fraction of the move.
            coast = float(np.interp(speed, [30, 50, 100], [21.5, 42.9, 80.5]))
            frac = max(0.0, (D - coast) / D)
            print(f'  cruise speed {speed}, coast {coast:.1f} cm -> stop at '
                  f'{frac * D:.1f} cm of {D:.1f} cm ({frac:.0%})')
            roll(speed, heading)
            cmd_hist.append((time.time(), speed * SPEED_TO_CMS, heading))
            checkpoints = [frac * D / 3, 2 * frac * D / 3]   # mid-course re-aims
            best, t_best = 0.0, time.time() + 1.5              # allow the BLE start delay
            while time.time() < t_end:
                now, m = step()
                covered = (ekf.x[0] - sx) * math.cos(phi) + (ekf.x[1] - sy) * math.sin(phi)
                if covered > best + 2.0:
                    best, t_best = covered, now
                elif now - t_best > 2.0:                   # no progress: link or robot gone
                    roll(0, heading)
                    print(f'  STALLED at {covered:.1f} cm: no progress for 2 s, aborting')
                    return False
                if checkpoints and covered >= checkpoints[0]:
                    checkpoints.pop(0)
                    # one heading write, well before the stop so it cannot delay it
                    bearing = math.atan2(ty - ekf.x[1], tx - ekf.x[0])
                    new_heading = math.degrees(offset - bearing) % 360
                    if abs(wrap(math.radians(new_heading - heading))) > math.radians(3):
                        heading = new_heading
                        roll(speed, heading)
                        print(f'  re-aim at {covered:.0f} cm: bearing '
                              f'{user_deg(bearing):.1f} deg')
                if covered >= frac * D:
                    roll(0, heading)
                    cmd_hist.append((now, 0.0, heading))
                    write(now, 'cruise', m, 0, heading)
                    print(f'  stop sent at +{now - t_go:.2f} s, covered {covered:.1f} cm, '
                          f'speed {speed_meas(0.2):.1f} cm/s')
                    break
                write(now, 'cruise', m, speed, heading)
            pos = wait_still('coast')
            err = math.hypot(tx - pos[0], ty - pos[1])
            print(f'  at rest +{time.time() - t_go:.2f} s: (%.1f, %.1f) cm, ' % flip(*pos) +
                  f'error {err:.2f} cm')
            gain, k, stalled = 38.0, 0, 0                  # cm per s beyond DEAD, learned
            while err > tol and time.time() < t_end and stalled < 2:
                k += 1
                phi = math.atan2(ty - pos[1], tx - pos[0])
                heading = math.degrees(offset - phi) % 360
                roll(0, heading)
                t_face = time.time() + 1.0
                while time.time() < t_face:
                    now, m = step()
                    write(now, f'face{k}', m, 0, heading)
                    if m is not None and m[2] is not None and abs(wrap(m[2] - phi)) < math.radians(15):
                        break
                T = float(np.clip(DEAD + err / gain, DEAD + 0.1, 1.4))
                roll(PULSE_SPEED, heading)
                t_p = time.time() + T
                while time.time() < t_p:
                    now, m = step()
                    write(now, f'pulse{k}', m, PULSE_SPEED, heading)
                roll(0, heading)
                new = wait_still(f'still{k}')
                moved = (new[0] - pos[0]) * math.cos(phi) + (new[1] - pos[1]) * math.sin(phi)
                if moved > 0.5:
                    gain = 0.5 * gain + 0.5 * moved / (T - DEAD)
                # pinned against a wall: pulses stop moving it, so stop pulsing
                stalled = stalled + 1 if abs(moved) < 1.0 else 0
                pos = new
                err = math.hypot(tx - pos[0], ty - pos[1])
                print(f'  pulse {k}: {T:.2f} s moved {moved:.1f} cm, error {err:.2f} cm '
                      f'at +{time.time() - t_go:.2f} s')
            return err <= tol

        if args.fast:
            t_go = time.time()
            done = drive_fast(40.0)
            elapsed = time.time() - t_go
            final = settle('confirm', 1.0)
            err = math.hypot(tx - final[0], ty - final[1])
            print(f'{"arrived" if done else "timed out"} in {elapsed:.2f} s: '
                  '(%.1f, %.1f) cm, ' % flip(*final) + f'error {err:.2f} cm')
            raise StopIteration

        # ---- 3. closed-loop drive, then 4. settle + pulse corrections
        t_go = time.time()
        drive('drive', MIN_SPEED, MIN_SPEED, 30.0)
        final = settle('settle', 2.5)
        err = math.hypot(tx - final[0], ty - final[1])
        print('after drive: (%.1f, %.1f) cm, ' % flip(*final) + f'error {err:.2f} cm')
        gain = 20.0                                        # cm per s of pulse, learned
        for k in range(args.attempts):
            if err <= tol:
                break
            final, moved, T = pulse(k + 1, gain)
            if moved > 0.5:
                gain = 0.5 * gain + 0.5 * moved / T
            err = math.hypot(tx - final[0], ty - final[1])
            print(f'pulse {k + 1}: {T:.2f} s moved {moved:.1f} cm -> '
                  '(%.1f, %.1f) cm, ' % flip(*final) +
                  f'error {err:.2f} cm (gain now {gain:.1f} cm/s)')
        elapsed = time.time() - t_go
    except StopIteration:
        pass
    finally:
        roll(0, last_heading[0])                           # stop; do not turn back to 0
        stop.set()
        time.sleep(0.3)
        log.close()

    ok = err <= tol
    print(f'\nRESULT: {"PASS" if ok else "FAIL"}  final error {err:.2f} cm = '
          f'{100 * err / dist0:.1f}% of {dist0:.1f} cm (limit {100 * args.tol_frac:.0f}% = '
          f'{tol:.2f} cm), {elapsed:.1f} s')

    bgr, _ = cam.frame()
    cam.cap.release()
    if bgr is not None:
        Hinv = np.linalg.inv(H)

        def px(x, y):
            p = Hinv @ np.array([x, y, 1.0])
            return int(p[0] / p[2]), int(p[1] / p[2])
        rows = [l.split(',') for l in open(os.path.join(args.out, 'log.csv')).read().splitlines()[1:]]
        pts = [px(float(r[5]), float(r[6])) for r in rows]
        for a, b in zip(pts, pts[1:]):
            cv2.line(bgr, a, b, (0, 255, 255), 2)
        cv2.circle(bgr, px(tx, ty), max(3, int(tol / 0.18)), (0, 0, 255), 2)
        cv2.drawMarker(bgr, px(*p0), (255, 255, 0), cv2.MARKER_CROSS, 20, 2)
        cv2.imwrite(os.path.join(args.out, 'result.jpg'), bgr)
    node.destroy_node()
    rclpy.shutdown()
    print(f'log + image: {args.out}')


if __name__ == '__main__':
    main()

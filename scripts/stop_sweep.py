#!/usr/bin/env python3
"""Find the stop fraction that lands a Sphero closest to a target D cm away.

Each trial: face along the arena's long axis (toward the side with more room),
roll at --speed, send stop once FRAC of D is covered (along the start->target
line, camera-measured), wait until still, record where it landed. No
corrections, so the landing is purely the stop fraction's doing.

Per trial: cruise speed, covered at stop, coast (stop -> rest), final along-
track error (+ = overshoot) and cross-track drift. Summary per fraction and
the fraction implied by the mean coast: 1 - coast / D.

Usage (ROS sourced):
  python3 scripts/stop_sweep.py --robot SB-418F --speed 30 --dist 100 \
      --fracs 0.70 0.80 0.90 0.75 0.85 --repeats 2
"""
import argparse
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

from goto_target import Camera
from kf_replay import wrap

ARENA_X = (0.0, 243.84)
MARGIN_X = 15.0                    # keep the landing this far off the side walls


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot', default='SB-418F')
    ap.add_argument('--speed', type=int, default=30)
    ap.add_argument('--dist', type=float, default=100.0)
    ap.add_argument('--fracs', nargs='+', type=float, default=[0.70, 0.80, 0.90, 0.75, 0.85])
    ap.add_argument('--repeats', type=int, default=2)
    ap.add_argument('--extra-room', type=float, default=0.0,
                    help='cm of free space required beyond the target (overshoot)')
    ap.add_argument('--recal', action='store_true', help='re-calibrate heading before each trial')
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--out', default=os.path.expanduser(
        f'~/sphero_ros2/recordings/stopsweep_{time.strftime("%Y%m%d_%H%M%S")}'))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    ns = args.robot.replace('-', '_')
    H = np.array(yaml.safe_load(open(os.path.expanduser(
        '~/overhead_field/overhead_arena.yaml')))['H'])

    rclpy.init()
    node = rclpy.create_node('stop_sweep')
    roll_pub = node.create_publisher(String, f'/sphero/{ns}/roll', 10)
    led_pub = node.create_publisher(String, f'/sphero/{ns}/led', 10)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(1.0)

    def roll(speed, heading):
        roll_pub.publish(String(data=json.dumps({'speed': int(speed),
                                                 'heading': int(round(heading)) % 360})))

    stop = threading.Event()

    def leds():
        for kind, rgb in (('main', (0, 0, 0)), ('front', (0, 255, 0)), ('back', (255, 0, 0))):
            led_pub.publish(String(data=json.dumps(
                {'type': kind, 'red': rgb[0], 'green': rgb[1], 'blue': rgb[2]})))

    # LED writes share the BLE link with roll/stop and can delay a stop by tenths
    # of a second, so they are re-sent only between trials, never during a move.
    def keep_lit_until_found():
        while not stop.is_set() and track['p'] is None:
            leds()
            stop.wait(2.0)

    cam = Camera(args.device, H)
    track = {'p': None}
    threading.Thread(target=keep_lit_until_found, daemon=True).start()
    log = open(os.path.join(args.out, 'log.csv'), 'w')
    log.write('trial,frac,t,x,y,heading_deg,cmd_speed\n')

    def meas(timeout=2.0):
        """(t, x, y, heading or None) of the robot: lit blob first, then nearest."""
        t_end = time.time() + timeout
        while time.time() < t_end:
            _, blobs = cam.frame()
            now = time.time()
            if track['p'] is None:
                lit = [b for b in blobs if b[2] is not None]
                if len(lit) == 1:
                    track['p'] = lit[0][:2]
                    return now, lit[0][0], lit[0][1], lit[0][2]
                continue
            near = [b for b in blobs if math.hypot(b[0] - track['p'][0], b[1] - track['p'][1]) < 15]
            if near:
                b = min(near, key=lambda b: math.hypot(b[0] - track['p'][0], b[1] - track['p'][1]))
                track['p'] = b[:2]
                return now, b[0], b[1], b[2]
        raise SystemExit('lost the robot')

    def wait_still(max_s=4.0, v_still=1.0, hold=0.4, rec=None):
        """rec = (trial, frac, t0): also log the frames (the coast after a stop)."""
        hist, since, t_end = [], None, time.time() + max_s
        while time.time() < t_end:
            t, x, y, h = meas()
            hist.append((t, x, y))
            if rec:
                log.write(f'{rec[0]},{rec[1]},{t - rec[2]:.3f},{x:.2f},{y:.2f},'
                          f'{"" if h is None else "%.1f" % math.degrees(h)},0\n')
            old = next((h for h in reversed(hist) if h[0] <= t - 0.2), hist[0])
            v = math.hypot(x - old[1], y - old[2]) / max(t - old[0], 1e-3)
            if v < v_still and t - hist[0][0] > 0.2:
                since = since or t
                if t - since >= hold:
                    break
            else:
                since = None
        pts = np.array([h[1:] for h in hist[-8:]])
        return tuple(pts.mean(axis=0))

    def mean_heading(n=20):
        hs = []
        while len(hs) < n:
            _, _, _, h = meas()
            if h is not None:
                hs.append(h)
        return math.atan2(np.mean(np.sin(hs)), np.mean(np.cos(hs)))

    results = []
    try:
        meas(15.0)                                         # find the lit robot
        roll(0, 0)
        time.sleep(2.0)
        offset = mean_heading()                            # field = offset - cmd
        print(f'heading offset {math.degrees(offset) % 360:.1f} deg; speed {args.speed}, '
              f'D = {args.dist:.0f} cm')
        order = [f for _ in range(args.repeats) for f in args.fracs]
        for k, frac in enumerate(order, 1):
            sx, sy = wait_still()
            room_e = ARENA_X[1] - MARGIN_X - sx
            room_w = sx - ARENA_X[0] - MARGIN_X
            phi = 0.0 if room_e >= room_w else math.pi
            if max(room_e, room_w) < args.dist + args.extra_room:
                print(f'  trial {k}: only {max(room_e, room_w):.0f} cm of room, stopping')
                break
            leds()                                         # between trials only
            heading = math.degrees(offset - phi) % 360
            roll(0, heading)                               # face first, outside the trial
            t_face = time.time() + 1.5
            while time.time() < t_face:
                _, _, _, h = meas()
                if h is not None and abs(wrap(h - phi)) < math.radians(10):
                    break
            wait_still()
            if args.recal:
                # re-calibrate from the heading at rest. Off by default: on
                # 2026-10-05 it doubled cross-track drift (11-18 cm vs 4-15)
                offset = mean_heading() + math.radians(heading)
                heading = math.degrees(offset - phi) % 360
                roll(0, heading)
                time.sleep(0.8)
            sx, sy = wait_still()
            ux, uy = math.cos(phi), math.sin(phi)
            t0 = time.time()
            roll(args.speed, heading)
            samples, t_stop, cov_stop, v_stop = [], None, None, None
            while time.time() - t0 < 12.0:
                t, x, y, h = meas()
                cov = (x - sx) * ux + (y - sy) * uy
                samples.append((t, cov))
                log.write(f'{k},{frac},{t - t0:.3f},{x:.2f},{y:.2f},'
                          f'{"" if h is None else "%.1f" % math.degrees(h)},{args.speed}\n')
                if cov >= frac * args.dist:
                    roll(0, heading)
                    t_stop, cov_stop = t, cov
                    old = next((s for s in reversed(samples) if s[0] <= t - 0.3), samples[0])
                    v_stop = (cov - old[1]) / max(t - old[0], 1e-3)
                    break
            if t_stop is None:
                roll(0, heading)
                print(f'  trial {k}: never covered {frac:.0%} in 12 s')
                continue
            ex_, ey_ = wait_still(rec=(k, frac, t0))
            along = (ex_ - sx) * ux + (ey_ - sy) * uy
            cross = -(ex_ - sx) * uy + (ey_ - sy) * ux
            r = dict(trial=k, frac=frac, v=v_stop, t_rest=time.time() - t0, t_stop=t_stop - t0, cov_stop=cov_stop,
                     coast=along - cov_stop, err=along - args.dist, cross=cross)
            results.append(r)
            print(f'  trial {k:2d} stop@{frac:.0%}: v {v_stop:5.1f} cm/s, stop sent at '
                  f'{cov_stop:5.1f} cm (+{r["t_stop"]:.2f} s), coast {r["coast"]:5.1f} cm, '
                  f'landed {along:6.1f} cm -> error {r["err"]:+6.1f} cm, drift {cross:+5.1f} cm, '
                  f'at rest +{r["t_rest"]:.1f} s')
    finally:
        roll(0, 0)
        stop.set()
        time.sleep(0.3)
        log.close()
        cam.cap.release()

    if results:
        print(f'\n{"stop %":>7s} {"n":>2s} {"error cm (each)":>24s} {"mean":>7s} {"coast cm":>9s}')
        for f in sorted(set(r['frac'] for r in results)):
            rs = [r for r in results if r['frac'] == f]
            errs = ' '.join(f'{r["err"]:+.1f}' for r in rs)
            print(f'{f:7.0%} {len(rs):2d} {errs:>24s} {np.mean([r["err"] for r in rs]):+7.1f} '
                  f'{np.mean([r["coast"] for r in rs]):9.1f}')
        coasts = np.array([r['coast'] for r in results])
        best = 1.0 - coasts.mean() / args.dist
        print(f'\ncoast at speed {args.speed}: mean {coasts.mean():.1f} cm, std {coasts.std(ddof=1) if len(coasts) > 1 else 0:.1f} cm, '
              f'range {coasts.min():.1f}-{coasts.max():.1f} -> best stop fraction for '
              f'{args.dist:.0f} cm: {best:.1%}')
    node.destroy_node()
    rclpy.shutdown()
    print(f'log: {args.out}')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Drive Spheros ONE AT A TIME, in a given order, to assigned field targets,
closed-loop on the LIVE overhead tracker (it does not touch the camera).

Plan file: {"order": [names...], "targets": {name: [x_cm, y_cm]}} (field frame).

Per robot:
  1. pose from /localization/<ns>/position (fresh = < 0.5 s old); wait up to
     5 s for one, else skip it.
  2. clearance of the straight start->target path from every other robot's
     CURRENT tracked position; under 8 cm -> skip it (retried once at the end).
  3. heading offset (field = offset - cmd, so cmd = offset - phi; ekf.py
     HeadingOffsetEstimator, goto_target.py step 2): roll(0, 0), then the
     node's latched /localization/<ns>/heading_offset, else LED heading at
     cmd 0, else 0 (the link-time aim zeroed cmd 0 on field +x).
  4. face the target at speed 0, then SPEED 50, re-aimed at the target every
     0.5 s; stop COAST_CM short (moves > LONG_CM) or after SHORT_FRAC of the
     distance (shorter moves) -- the ring_goto.py rule. Then up to 2
     correction passes while more than 10 cm off.
  Guards while moving: stop if within 9 cm of another robot (and closing),
  if its own pose goes stale, or if it travels > 20 deg off the commanded
  bearing (the offset is then re-learnt from the travel direction and the
  correction passes continue with it). Ctrl+C stops every robot (speed 0).
No LED writes at all. Log: ~/overhead_field/grid_goto_<stamp>/log.csv.

Usage (ROS sourced, overhead node TRACKING):
  python3 scripts/grid_goto.py --plan <grid_seq.json> [--dry-run]
"""
import argparse
import json
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from std_msgs.msg import Float32, String

# proven constants (ring_goto.py / crossing_record.py)
SPEED = 50                # user rule: never below 50
COAST_CM = 42.9           # stop -> rest at speed 50 (stop_sweep, 2026-10-05)
LONG_CM = 60.0
SHORT_FRAC = 0.4
REAIM_S = 0.5
REAIM_DEADBAND = 3.0      # deg, ring_goto.py
SETTLE_S = 3.0            # after a stop, ring_goto.py
FACE_S = 1.5              # speed-0 turn before rolling, ring_goto.py
# task rules
STALE_S = 0.5
WAIT_POSE_S = 5.0
PATH_CLEAR_CM = 8.0
PROX_STOP_CM = 9.0
TOL_CM = 10.0
CORRECTION_PASSES = 2
DIR_GUARD_DEG = 20.0      # travel vs commanded bearing
DIR_GUARD_MIN_CM = 8.0    # only judged over windows with this much travel


# ------------------------------------------------------------- pure functions
def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def bearing_deg(p, q):
    """Field bearing p -> q, deg CCW from +x."""
    return math.degrees(math.atan2(q[1] - p[1], q[0] - p[0]))


def field_to_cmd(offset_deg, phi_deg):
    """Sphero command heading (CW) to travel along field bearing phi (CCW)."""
    return int(round(offset_deg - phi_deg)) % 360


def offset_from(field_heading_deg, cmd_deg):
    """Offset from an observed field heading while commanding cmd."""
    return (field_heading_deg + cmd_deg) % 360.0


def seg_point_dist(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def path_clearance(start, target, others):
    """(min distance, name) from the segment start->target to others {name: (x, y)}."""
    best = (math.inf, None)
    for n, p in others.items():
        d = seg_point_dist(p, start, target)
        if d < best[0]:
            best = (d, n)
    return best


def should_stop(d, rem, moved):
    """ring_goto.py stop rule. d = start->target distance of this pass."""
    if d > LONG_CM:
        return rem <= COAST_CM
    return moved >= max(SHORT_FRAC * d, 2.0)


# ------------------------------------------------------------------ ROS side
class Fleet:
    def __init__(self, node, names, publish):
        self.node = node
        self.lock = threading.Lock()
        self.pose = {}        # name -> (x, y, recv_time)
        self.offset = {}      # name -> (deg, recv_time), node estimate
        self.hdg = {}         # name -> (heading_deg or None, recv_time)
        self.last_cmd = {n: 0 for n in names}
        self.roll_pub = {}
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST)
        for n in names:
            ns = n.replace('-', '_')
            node.create_subscription(PoseStamped, f'/localization/{ns}/position',
                                     lambda m, n=n: self._on_pose(n, m), 10)
            node.create_subscription(Float32, f'/localization/{ns}/heading_offset',
                                     lambda m, n=n: self._on_offset(n, m), latched)
            if publish:
                self.roll_pub[n] = node.create_publisher(String, f'/sphero/{ns}/roll', 10)
        node.create_subscription(String, '/overhead_tracker_node/detections', self._on_det, 10)

    def _on_pose(self, n, m):
        with self.lock:
            self.pose[n] = (m.pose.position.x, m.pose.position.y, time.time())

    def _on_offset(self, n, m):
        with self.lock:
            self.offset[n] = (float(m.data) % 360.0, time.time())

    def _on_det(self, m):
        try:
            robots = json.loads(m.data)['robots']
        except (ValueError, KeyError):
            return
        t = time.time()
        with self.lock:
            for r in robots:
                self.hdg[r['name']] = (r.get('heading_deg'), t)

    def get(self, n):
        """(x, y) if fresh, else None."""
        with self.lock:
            p = self.pose.get(n)
        if p is None or time.time() - p[2] > STALE_S:
            return None
        return p[0], p[1]

    def last_known(self, n):
        with self.lock:
            p = self.pose.get(n)
        return None if p is None else (p[0], p[1])

    def heading(self, n):
        with self.lock:
            h = self.hdg.get(n)
        if h is None or h[0] is None or time.time() - h[1] > STALE_S:
            return None
        return float(h[0])

    def node_offset(self, n, since=0.0):
        """Node estimate received after `since` (it may be latched from before
        an aim reset otherwise), or None."""
        with self.lock:
            o = self.offset.get(n)
        return o[0] if o is not None and o[1] > since else None

    def roll(self, n, speed, heading):
        self.last_cmd[n] = int(round(heading)) % 360
        pub = self.roll_pub.get(n)
        if pub is not None:
            pub.publish(String(data=json.dumps({'heading': self.last_cmd[n], 'speed': int(speed)})))

    def stop_all(self):
        for n in self.roll_pub:
            self.roll(n, 0, self.last_cmd[n])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--plan', required=True)
    ap.add_argument('--dry-run', action='store_true',
                    help='print the plan and command headings; publish nothing')
    args = ap.parse_args()
    plan = json.load(open(args.plan))
    order = list(plan['order'])
    targets = {n: tuple(v) for n, v in plan['targets'].items()}
    missing = [n for n in order if n not in targets]
    if missing:
        raise SystemExit(f'no target for {missing}')
    names = sorted(set(order) | set(targets))

    # no rclpy SIGINT handler: it would shut the context down before the
    # finally block can publish the speed-0 stops
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('grid_goto')
    fleet = Fleet(node, names, publish=not args.dry_run)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    def close_ros():
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin.join(timeout=2.0)
    t_end = time.time() + 3.0
    while time.time() < t_end and len(fleet.pose) < len(names):
        time.sleep(0.1)
    time.sleep(0.5)                              # detections + latched offsets

    if args.dry_run:
        try:
            dry_run(fleet, order, targets)
        finally:
            close_ros()
        return

    out = os.path.expanduser(f'~/overhead_field/grid_goto_{time.strftime("%Y%m%d_%H%M%S")}')
    os.makedirs(out, exist_ok=True)
    log = open(os.path.join(out, 'log.csv'), 'w')
    log.write('t,robot,x,y,speed,cmd_heading,event\n')
    log_lock = threading.Lock()

    def row(n, speed='', cmd='', event=''):
        p = fleet.last_known(n) or ('', '')
        with log_lock:
            log.write(f'{time.time():.4f},{n},'
                      f'{p[0] if p[0] == "" else f"{p[0]:.2f}"},'
                      f'{p[1] if p[1] == "" else f"{p[1]:.2f}"},{speed},{cmd},{event}\n')

    def say(n, msg):
        print(f'[{time.strftime("%H:%M:%S")}] {n}: {msg}', flush=True)
        row(n, event=msg.replace(',', ';'))

    def roll(n, speed, heading):
        fleet.roll(n, speed, heading)
        row(n, speed, fleet.last_cmd[n])

    def wait_pose(n):
        t_end = time.time() + WAIT_POSE_S
        while time.time() < t_end:
            p = fleet.get(n)
            if p is not None:
                return p
            time.sleep(0.05)
        return None

    def others_now(n):
        return {m: p for m in names if m != n and (p := fleet.last_known(m)) is not None}

    def acquire_offset(n):
        t_cmd = time.time()
        roll(n, 0, 0)
        time.sleep(2.5)       # node estimator: 1.5 s settle + 20 LED samples
        o = fleet.node_offset(n, since=t_cmd)
        if o is not None:
            return o, 'node'
        h = fleet.heading(n)
        if h is not None:
            return offset_from(h, 0), 'led@cmd0'
        return 0.0, 'link-aim(0)'

    def drive(n, offset, name):
        """One pass. Returns (status, offset); status 'ok', 'prox', 'stale',
        'dir', 'timeout'."""
        start = fleet.get(n)
        if start is None:
            return 'stale', offset
        tgt = targets[n]
        d = math.hypot(tgt[0] - start[0], tgt[1] - start[1])
        phi = bearing_deg(start, tgt)
        roll(n, 0, field_to_cmd(offset, phi))          # face the target first
        time.sleep(FACE_S)
        start = fleet.get(n) or start
        others0 = others_now(n)
        d0 = {m: math.hypot(p[0] - start[0], p[1] - start[1]) for m, p in others0.items()}
        say(n, f'{name} -> ({tgt[0]:.1f},{tgt[1]:.1f}) {d:.0f} cm cmd {fleet.last_cmd[n]}'
               + ('' if d > LONG_CM else ' (short)'))
        roll(n, SPEED, fleet.last_cmd[n])
        t0 = t_aim = time.time()
        win_p, win_phi = start, phi
        t_lim = t0 + max(15.0, d / 15.0 + 5.0)
        while True:
            time.sleep(0.05)
            p = fleet.get(n)
            if p is None:
                roll(n, 0, fleet.last_cmd[n])
                say(n, f'{name} STOP: own pose stale')
                return 'stale', offset
            row(n, SPEED, fleet.last_cmd[n])
            for m, q in others_now(n).items():
                dd = math.hypot(q[0] - p[0], q[1] - p[1])
                if dd < PROX_STOP_CM and dd < d0.get(m, math.inf) - 0.5:
                    roll(n, 0, fleet.last_cmd[n])
                    say(n, f'{name} STOP: {dd:.1f} cm from {m}')
                    return 'prox', offset
            rem = math.hypot(tgt[0] - p[0], tgt[1] - p[1])
            moved = math.hypot(p[0] - start[0], p[1] - start[1])
            if should_stop(d, rem, moved):
                roll(n, 0, fleet.last_cmd[n])
                say(n, f'{name} stop rem={rem:.1f} moved={moved:.1f}')
                return 'ok', offset
            now = time.time()
            if now > t_lim:
                roll(n, 0, fleet.last_cmd[n])
                say(n, f'{name} STOP: timeout rem={rem:.1f}')
                return 'timeout', offset
            if now - t_aim >= REAIM_S:
                step = math.hypot(p[0] - win_p[0], p[1] - win_p[1])
                if step >= DIR_GUARD_MIN_CM:
                    travel = bearing_deg(win_p, p)
                    err = wrap180(travel - win_phi)
                    if abs(err) > DIR_GUARD_DEG:
                        roll(n, 0, fleet.last_cmd[n])
                        new = offset_from(travel, fleet.last_cmd[n])
                        say(n, f'{name} STOP: travelling {err:+.0f} deg off; '
                               f'offset {offset:.0f} -> {new:.0f} (from travel)')
                        return 'dir', new
                win_p, t_aim = p, now
                if d > LONG_CM:                  # ring_goto re-aims long moves only
                    win_phi = bearing_deg(p, tgt)
                    cmd = field_to_cmd(offset, win_phi)
                    if abs(wrap180(cmd - fleet.last_cmd[n])) > REAIM_DEADBAND:
                        roll(n, SPEED, cmd)

    def run_robot(n):
        """Returns 'done', 'skip-clear', 'skip-pose', or 'aborted'."""
        p = wait_pose(n)
        if p is None:
            say(n, f'SKIP: no fresh pose within {WAIT_POSE_S:.0f} s')
            return 'skip-pose'
        tgt = targets[n]
        clr, who = path_clearance(p, tgt, others_now(n))
        if clr < PATH_CLEAR_CM:
            say(n, f'WAIT: path clearance {clr:.1f} cm < {PATH_CLEAR_CM:.0f} (from {who}); not driving')
            return 'skip-clear'
        t0 = time.time()
        offset, src = acquire_offset(n)
        t_off = time.time() - 1.0
        p = fleet.get(n) or p
        # clearance again: the speed-0 turn took 2 s, others may have been nudged
        clr, who = path_clearance(p, tgt, others_now(n))
        if clr < PATH_CLEAR_CM:
            say(n, f'WAIT: path clearance {clr:.1f} cm < {PATH_CLEAR_CM:.0f} (from {who}); not driving')
            return 'skip-clear'
        say(n, f'START ({p[0]:.1f},{p[1]:.1f}) -> ({tgt[0]:.1f},{tgt[1]:.1f}) '
               f'{math.hypot(tgt[0] - p[0], tgt[1] - p[1]):.0f} cm, clearance {clr:.1f} cm ({who}), '
               f'offset {offset:.0f} [{src}]')
        status, offset = drive(n, offset, 'main')
        time.sleep(SETTLE_S)
        k = 0
        while status in ('ok', 'dir', 'timeout') and k < CORRECTION_PASSES:
            q = fleet.get(n)
            if q is None or math.hypot(tgt[0] - q[0], tgt[1] - q[1]) <= TOL_CM:
                break
            clr, who = path_clearance(q, tgt, others_now(n))
            if clr < PATH_CLEAR_CM:
                say(n, f'no correction: clearance {clr:.1f} cm (from {who})')
                break
            k += 1
            o = fleet.node_offset(n, since=t_off)
            if status != 'dir' and o is not None:
                offset = o
            status, offset = drive(n, offset, f'fix{k}')
            time.sleep(SETTLE_S)
        q = fleet.get(n) or fleet.last_known(n)
        err = math.hypot(tgt[0] - q[0], tgt[1] - q[1]) if q else float('nan')
        say(n, f'DONE start ({p[0]:.1f},{p[1]:.1f}) target ({tgt[0]:.1f},{tgt[1]:.1f}) '
               f'final ({q[0]:.1f},{q[1]:.1f}) err {err:.1f} cm, {time.time() - t0:.1f} s '
               f'[{status}, {k} fix]')
        return 'done' if status not in ('prox', 'stale') else 'aborted'

    results = {}
    try:
        for n in order:
            results[n] = run_robot(n)
        retry = [n for n in order if results[n] in ('skip-clear', 'skip-pose')]
        if retry:
            print(f'retrying skipped: {retry}', flush=True)
            for n in retry:
                results[n] = run_robot(n)
        print('\nsummary:')
        for n in order:
            q = fleet.last_known(n)
            e = math.hypot(targets[n][0] - q[0], targets[n][1] - q[1]) if q else float('nan')
            print(f'  {n}: {results[n]:10s} err {e:5.1f} cm')
    except KeyboardInterrupt:
        print('\nCtrl+C: stopping every robot', flush=True)
    finally:
        for _ in range(3):
            fleet.stop_all()
            time.sleep(0.15)
        for n in names:
            row(n, 0, fleet.last_cmd[n], 'final stop (speed 0) sent')
        with log_lock:
            log.close()
        close_ros()
    print(f'log -> {out}/log.csv')


def dry_run(fleet, order, targets):
    """Print the plan as it stands NOW (every robot still at its current pose;
    the live run re-checks clearance before each move from then-current poses)."""
    found = {n: fleet.get(n) for n in targets}
    print(f'poses found: {sum(p is not None for p in found.values())}/{len(targets)}')
    print('offset = node heading_offset if latched, else LED heading assuming last cmd 0, '
          'else 0 (link aim). cmd = (offset - bearing) % 360')
    sim = {n: p for n, p in found.items() if p is not None}   # parked as we go
    print(f'{"#":>2} {"robot":8} {"start":>14} {"target":>14} {"dist":>5} {"brg":>6} '
          f'{"offset":>7} {"src":9} {"cmd":>4} {"clr_now":>8} {"clr_seq":>8}  rule')
    for i, n in enumerate(order, 1):
        p, t = found[n], targets[n]
        if p is None:
            print(f'{i:2d} {n:8} NO FRESH POSE -> would wait 5 s then skip')
            continue
        o = fleet.node_offset(n)
        src = 'node'
        if o is None:
            h = fleet.heading(n)
            o, src = (offset_from(h, 0), 'led@cmd0') if h is not None else (0.0, 'link-aim')
        d = math.hypot(t[0] - p[0], t[1] - p[1])
        phi = bearing_deg(p, t)
        now_others = {m: q for m, q in found.items() if m != n and q is not None}
        c_now, w_now = path_clearance(p, t, now_others)
        seq_others = {m: q for m, q in sim.items() if m != n}
        c_seq, w_seq = path_clearance(p, t, seq_others)
        rule = f'stop at rem {COAST_CM}' if d > LONG_CM else f'stop after {SHORT_FRAC * d:.1f} cm'
        flag = '' if c_seq >= PATH_CLEAR_CM else '  <-- WOULD WAIT'
        print(f'{i:2d} {n:8} ({p[0]:5.1f},{p[1]:5.1f}) ({t[0]:5.1f},{t[1]:5.1f}) {d:5.0f} '
              f'{phi:6.1f} {o:7.1f} {src:9} {field_to_cmd(o, phi):4d} '
              f'{c_now:5.1f}/{(w_now or "")[-4:]} {c_seq:5.1f}/{(w_seq or "")[-4:]}  {rule}{flag}')
        sim[n] = t                                  # assume it arrives
    print('clr_now: clearance vs everyone where they stand now; clr_seq: vs the '
          'sequence (robots already moved assumed at their targets) -- what the '
          'live run will check.')


if __name__ == '__main__':
    main()

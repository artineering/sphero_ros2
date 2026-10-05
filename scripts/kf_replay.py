#!/usr/bin/env python3
"""Offline EKF on a kf_record.py recording: blob position + LED heading.

State x = [px, py, theta]  (cm, cm, rad; FIELD frame, theta CCW from +x)
Input u = [v, omega]       v = speed * 0.61 cm/s (measured, overhead_tracking.yaml)
                           omega = d(commanded heading)/dt, sign flipped: Sphero
                           headings are clockwise, field angles counter-clockwise.
                           Only the CHANGE is used, so the robot's unknown aim
                           offset cancels. Commands are applied `lag` s late.
Measure z = [px, py, theta] when both LEDs are found (heading.heading_from_pair),
            [px, py] otherwise (H is the first two rows of I).

Prediction / update exactly as in the formulation: A = df/dx, P = A P A' + Q,
S = H P H' + R, K = P H' S^-1, with the heading residual wrapped to [-pi, pi].

Reports, for the EKF with inputs and for a no-input baseline (v = omega = 0):
innovation RMS, NIS, prediction error after hiding the camera for 0.5 s, and
per-step cost. Writes <rec>/kf_tracks.png.

Usage: python3 scripts/kf_replay.py recordings/kf_<stamp>
"""
import argparse
import csv
import math
import os
import pickle
import sys
import time

import cv2
import numpy as np
import yaml

from blob_detect_timing import detect, make_roi, replay_frames, threshold_map

sys.path.insert(0, os.path.expanduser('~/sphero_ros2/src/overhead_tracking'))
from overhead_tracking import heading as hdg  # noqa: E402

SPEED_TO_CMS = 0.61
LED_PAIR_MIN_SEP_PX = 13          # point_sep_px / 2, as the node uses
GATE_CM = 15.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ------------------------------------------------------------------ loading
def load(rec):
    t_frames = np.array([float(r['t']) for r in csv.DictReader(open(f'{rec}/frames.csv'))])
    cmds = {}
    for r in csv.DictReader(open(f'{rec}/cmds.csv')):
        cmds.setdefault(r['robot'], []).append(
            (float(r['t']), float(r['speed']), float(r['heading'])))
    nudge = {r['robot']: float(r['t']) for r in csv.DictReader(open(f'{rec}/leds.csv'))
             if r['event'] == 'nudge'}
    H = np.array(yaml.safe_load(open(os.path.expanduser(
        '~/overhead_field/overhead_arena.yaml')))['H'])
    return t_frames, cmds, nudge, H


def to_cm(H, u, v):
    p = H @ np.array([u, v, 1.0])
    return p[0] / p[2], p[1] / p[2]


def measure(rec, n_frames):
    """Per frame: list of (u, v, heading_deg or None) for every blob."""
    roi, tmap, out = make_roi(), None, []
    for i, buf in enumerate(replay_frames(f'{rec}/raw.mjpeg')):
        if i >= n_frames:
            break
        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if tmap is None:
            tmap = threshold_map([g], roi, 3.2)           # first frame: all dark, still
        row = []
        for b in detect(g, roi, tmap, 200):
            u, v = b['Centroid']
            red = hdg.find_led(bgr, u, v, hdg.RED_HUE, 30, 90, 60)
            green = hdg.find_led(bgr, u, v, hdg.GREEN_HUE, 30, 90, 60)
            h = (hdg.heading_from_pair(red, green, LED_PAIR_MIN_SEP_PX)
                 if red is not None and green is not None else None)
            row.append((u, v, h))
        out.append(row)
    return out


def identify(meas, t_frames, nudge, robots):
    """Blob of each robot: the only one that moved after that robot's solo nudge
    (commands show up as motion ~1 s late, so compare nudge vs nudge + 2 s)."""
    start = {}
    for r in robots:
        i0 = int(np.searchsorted(t_frames, nudge[r]))
        i1 = int(np.searchsorted(t_frames, nudge[r] + 2.0))
        before = [(u, v) for u, v, _ in meas[i0]]
        after = np.array([(u, v) for u, v, _ in meas[i1]])
        moved = [(u, v) for u, v in before
                 if np.hypot(*(after - (u, v)).T).min() > 5.0]
        if len(moved) != 1:
            raise SystemExit(f'{r}: {len(moved)} blobs moved after its nudge')
        start[r] = (i0, moved[0])
    return start


# ------------------------------------------------------------------ filter
class EKF:
    def __init__(self, x0, q_pos, q_th, r_pos, r_th):
        self.x = np.array(x0, float)
        self.P = np.diag([1.0, 1.0, 0.5])
        self.q = np.array([q_pos, q_pos, q_th])
        self.R = np.diag([r_pos, r_pos, r_th])

    def predict(self, v, w, dt):
        th = self.x[2]
        self.x = self.x + np.array([v * math.cos(th) * dt, v * math.sin(th) * dt, w * dt])
        self.x[2] = wrap(self.x[2])
        A = np.array([[1, 0, -v * math.sin(th) * dt],
                      [0, 1, v * math.cos(th) * dt],
                      [0, 0, 1]])
        self.P = A @ self.P @ A.T + np.diag(self.q * dt)

    def update(self, z):
        m = len(z)
        H = np.eye(3)[:m]
        y = np.asarray(z, float) - H @ self.x
        if m == 3:
            y[2] = wrap(y[2])
        S = H @ self.P @ H.T + self.R[:m, :m]
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2] = wrap(self.x[2])
        self.P = (np.eye(3) - K @ H) @ self.P
        return y, float(y @ np.linalg.solve(S, y))


def inputs_at(cmd, t):
    """(v cm/s, field heading rad) of the last command at or before t."""
    v, h = 0.0, None
    for tc, s, hd in cmd:
        if tc > t:
            break
        v, h = s * SPEED_TO_CMS, math.radians(-hd)
    return v, h


def run(robot_meas, t_frames, cmd, H, p, use_inputs, drop=None):
    """robot_meas: per-frame (u, v, h_deg) or None. Returns stats dict."""
    i0 = next(i for i, m in enumerate(robot_meas) if m is not None and m[2] is not None)
    u, v, h = robot_meas[i0]
    ekf = EKF((*to_cm(H, u, v), math.radians(h)), p['q_pos'], p['q_th'], p['r_pos'], p['r_th'])
    prev_h, pending = None, 0.0
    rate = math.radians(p.get('turn_rate', math.inf))
    pos_res, th_res, nis, cost, drop_err, track = [], [], [], [], [], []
    for i in range(i0 + 1, len(robot_meas)):
        dt = t_frames[i] - t_frames[i - 1]
        vin, w = 0.0, 0.0
        if use_inputs:
            vin, _ = inputs_at(cmd, t_frames[i] - p['lag'])
            _, hc = inputs_at(cmd, t_frames[i] - p.get('lag_w', p['lag']))
            if hc is not None and prev_h is not None:
                pending += wrap(hc - prev_h)              # commanded turn still to do
            prev_h = hc if hc is not None else prev_h
            # turn_rate = inf is the formulation as given (whole step in one dt)
            w = max(-rate, min(rate, pending / dt))
            pending -= w * dt
        a = time.perf_counter()
        ekf.predict(vin, w, dt)
        m = robot_meas[i]
        hidden = drop is not None and drop(i - i0)
        if m is not None:
            z_xy = np.array(to_cm(H, m[0], m[1]))
            if hidden:
                if drop(i - i0 + 1) is False:              # last hidden frame
                    drop_err.append(float(np.linalg.norm(z_xy - ekf.x[:2])))
            else:
                z = list(z_xy) + ([math.radians(m[2])] if m[2] is not None else [])
                y, d2 = ekf.update(z)
                pos_res.append(np.linalg.norm(y[:2]))
                if len(y) == 3:
                    th_res.append(abs(y[2]))
                nis.append(d2 / len(y))
        cost.append(time.perf_counter() - a)
        track.append((*ekf.x, m is not None and not hidden))
    return {'pos_rms': float(np.sqrt(np.mean(np.square(pos_res)))) if pos_res else float('nan'),
            'th_rms_deg': math.degrees(float(np.sqrt(np.mean(np.square(th_res))))) if th_res else float('nan'),
            'nis': float(np.mean(nis)) if nis else float('nan'),
            'drop_err': float(np.mean(drop_err)) if drop_err else float('nan'),
            'drop_p90': float(np.percentile(drop_err, 90)) if drop_err else float('nan'),
            'cost_us': float(np.mean(cost) * 1e6), 'track': np.array(track)}


def associate(meas, t_frames, start, H):
    """Nearest-blob association from each robot's ID frame, gated at GATE_CM.
    Simple on purpose: the EKF is what is being tested, not data association."""
    out = {}
    for r, (i0, (u0, v0)) in start.items():
        seq, last = [None] * len(meas), (u0, v0)
        for i in range(i0, len(meas)):
            best = min(meas[i], key=lambda b: math.hypot(b[0] - last[0], b[1] - last[1]),
                       default=None)
            if best is not None:
                d = np.subtract(to_cm(H, *best[:2]), to_cm(H, *last))
                if np.hypot(*d) < GATE_CM:
                    seq[i], last = best, best[:2]
        out[r] = seq
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('rec')
    ap.add_argument('--robots', nargs='+', help='only these (default: all nudged)')
    args = ap.parse_args()
    rec = args.rec.rstrip('/')
    t_frames, cmds, nudge, H = load(rec)
    robots = [r for r in nudge if r in cmds and (not args.robots or r in args.robots)]
    print(f'robots with commands: {robots}')

    cache = f'{rec}/meas.pkl'
    if os.path.exists(cache):
        meas = pickle.load(open(cache, 'rb'))
    else:
        t0 = time.perf_counter()
        meas = measure(rec, len(t_frames))
        pickle.dump(meas, open(cache, 'wb'))
        print(f'measured {len(meas)} frames in {time.perf_counter() - t0:.0f} s')
    start = identify(meas, t_frames, nudge, [r for r in nudge if r in cmds])
    start = {r: start[r] for r in robots}
    seqs = associate(meas, t_frames, start, H)

    # Still-period noise -> R (LEDs settled, before the first nudge)
    first_cmd = min(nudge.values())
    r_pos, r_th = [], []
    for r in robots:
        # associate() starts at the nudge, so walk back from there for still frames
        i0, (u0, v0) = start[r]
        still = []
        for i in range(i0, -1, -1):
            if t_frames[i] < first_cmd - 2.0:
                break
            near = [b for b in meas[i] if math.hypot(b[0] - u0, b[1] - v0) < 5]
            still += near[:1]
        xy = np.array([to_cm(H, m[0], m[1]) for m in still])
        hs = np.radians([m[2] for m in still if m[2] is not None])
        r_pos.append(xy.var(axis=0).mean())
        if len(hs):
            r_th.append(np.var(np.unwrap(hs)))
        print(f'{r}: still frames {len(still)}, pos std {math.sqrt(r_pos[-1]):.3f} cm, '
              f'heading std {math.degrees(math.sqrt(r_th[-1])) if len(hs) else float("nan"):.2f} deg, '
              f'LED heading in {100 * np.mean([m[2] is not None for m in seqs[r] if m]):.0f}% of frames')
    # floor the measured (sub-pixel) noise: centroids also shift as the shell turns
    base = {'r_pos': max(float(np.mean(r_pos)), 0.05), 'r_th': max(float(np.mean(r_th)), 1e-3)}

    def total(p, use_inputs, drop=None):
        rs = [run(seqs[r], t_frames, cmds[r], H, p, use_inputs, drop) for r in robots]
        keys = ['pos_rms', 'th_rms_deg', 'nis', 'drop_err', 'drop_p90', 'cost_us']
        return {k: float(np.nanmean([x[k] for x in rs])) for k in keys}, rs

    # Tune: grid over lag and process noise, scored on 0.5 s blind prediction
    drop = lambda k: (k % 120) >= 90                       # hide 0.5 s of every 2 s
    best = {}
    for key, use_inputs in (('given', True), ('none', False)):
        grid = []
        for lag in (np.arange(0, 1.25, 0.05) if use_inputs else [0.0]):
            for q_pos in (1, 10, 100, 1000):
                for q_th in (0.1, 1, 10):
                    p = dict(base, lag=float(lag), q_pos=q_pos, q_th=q_th)
                    s, _ = total(p, use_inputs, drop)
                    grid.append((s['drop_err'], p))
        best[key] = min(grid, key=lambda g: g[0])[1]
    # Variant: own lag for omega + finite turn rate, scored on heading innovation
    grid = []
    for lag_w in np.arange(0, 0.65, 0.05):
        for rate in (math.inf, 720, 360, 180):
            for q_th in (0.1, 1, 10):
                p = dict(best['given'], lag_w=float(lag_w), turn_rate=rate, q_th=q_th)
                s, _ = total(p, True)
                grid.append((s['th_rms_deg'], p))
    best['split'] = min(grid, key=lambda g: g[0])[1]

    print(f'\nR from still frames: pos var {base["r_pos"]:.3f} cm^2, heading var {base["r_th"]:.4f} rad^2')
    print(f'{"":30s} {"lag":>5s} {"lag_w":>5s} {"turn":>5s} {"q_pos":>6s} {"q_th":>5s} {"innov cm":>9s} '
          f'{"innov deg":>9s} {"NIS":>5s} {"0.5s blind cm":>13s} {"p90":>6s} {"us/step":>7s}')
    results = {}
    for key, use_inputs, label in (('given', True, 'EKF, formulation as given'),
                                   ('split', True, 'EKF, omega lag + turn rate'),
                                   ('none', False, 'no inputs (v=w=0)')):
        p = best[key]
        s_blind, _ = total(p, use_inputs, drop)
        s_full, rs = total(p, use_inputs)
        results[key] = rs
        print(f'{label:30s} {p["lag"]:5.2f} {p.get("lag_w", p["lag"]):5.2f} '
              f'{p.get("turn_rate", math.inf):5g} {p["q_pos"]:6g} {p["q_th"]:5g} '
              f'{s_full["pos_rms"]:9.2f} {s_full["th_rms_deg"]:9.2f} {s_full["nis"]:5.2f} '
              f'{s_blind["drop_err"]:13.2f} {s_blind["drop_p90"]:6.2f} {s_full["cost_us"]:7.1f}')
    results[True], results[False] = results['split'], results['none']

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, len(robots), figsize=(7 * len(robots), 6), squeeze=False)
    for ax, r, rw, rn in zip(axes[0], robots, results[True], results[False]):
        z = np.array([to_cm(H, m[0], m[1]) for m in seqs[r] if m is not None])
        ax.plot(z[:, 0], z[:, 1], '.', ms=2, color='0.6', label='blob (measured)')
        ax.plot(rw['track'][:, 0], rw['track'][:, 1], '-', lw=1, label='EKF with inputs')
        ax.plot(rn['track'][:, 0], rn['track'][:, 1], '-', lw=1, label='no inputs')
        tr = rw['track'][::30]
        ax.quiver(tr[:, 0], tr[:, 1], np.cos(tr[:, 2]), np.sin(tr[:, 2]), width=0.003,
                  scale=30, color='C0', alpha=0.6)
        ax.set_title(r)
        ax.set_aspect('equal')
        ax.set_xlabel('field x (cm)')
        ax.set_ylabel('field y (cm)')
        ax.legend(loc='best', fontsize=8)
    fig.tight_layout()
    fig.savefig(f'{rec}/kf_tracks.png', dpi=110)
    print(f'\nplot: {rec}/kf_tracks.png')


if __name__ == '__main__':
    main()

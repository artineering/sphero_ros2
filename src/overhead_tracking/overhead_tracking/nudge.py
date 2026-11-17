#!/usr/bin/env python3
"""Nudge linking helpers (pure, no ROS).

Identity by motion: in a window where exactly ONE robot is commanded to move,
the one blob that moved is that robot. Port of `scripts/kf_replay.py`
`identify()`. The same watcher runs live (the node issues each nudge and reads
the /sphero/<ns>/motion_cmd echoes) and on a replay (the recorded cmds.csv
echoes), so both are identified by one mechanism.

Command times and frame stamps must share a clock: wall time live, the
re-based recording time on a replay.
"""

import math

import cv2
import numpy as np


class NudgeWatch:
    """Finds solo moves in a motion command stream.

    Feed every command in time order with `command()`, then call `frame()` once
    per frame. A window opens when a robot is commanded to move while no other
    robot is; it is spoiled if any other robot is commanded to move before it
    closes. It closes `settle` seconds after that robot's stop command, which
    has to cover the BLE command latency (~1 s) plus the roll-out.

    frame() returns
      'before' on the first frame at/after the move command (the robot has not
               started yet: motion lags the echo), so the caller snapshots blobs,
      'after'  once the window has settled, so the caller snapshots again and
               calls `take()`,
      None     otherwise.
    """

    def __init__(self, settle):
        self.settle = float(settle)
        self.moving = {}                 # name -> commanded speed > 0
        self.cand = None                 # {'name', 't_stop', 'before'}

    @property
    def name(self):
        return self.cand['name'] if self.cand else None

    def command(self, t, name, speed):
        if speed > 0:
            if self.cand is None:
                if not any(m for n, m in self.moving.items() if n != name):
                    self.cand = {'name': name, 't_stop': None, 'before': None}
            elif self.cand['name'] != name:
                self.cand = None         # not solo any more
            else:
                self.cand['t_stop'] = None
            self.moving[name] = True
        else:
            self.moving[name] = False
            if self.cand is not None and self.cand['name'] == name:
                self.cand['t_stop'] = t

    def frame(self, stamp):
        c = self.cand
        if c is None:
            return None
        if c['before'] is None:
            return 'before'
        if c['t_stop'] is not None and stamp >= c['t_stop'] + self.settle \
                and not any(self.moving.values()):
            return 'after'
        return None

    def set_before(self, data):
        self.cand['before'] = data

    def take(self):
        """(name, before data) of the closed window; clears it."""
        c, self.cand = self.cand, None
        return c['name'], c['before']


def moved_blob(before, after, min_move_px):
    """The one blob that moved between two snapshots of (u, v) points.

    Returns (old_uv, new_uv, reason): reason '' on success, else 'moved=<n>'
    (not exactly one blob left its place) or 'new=<n>' (not exactly one blob
    appeared somewhere new).
    """
    b = np.asarray(before, float).reshape(-1, 2)
    a = np.asarray(after, float).reshape(-1, 2)
    if len(b) == 0 or len(a) == 0:
        return None, None, f'moved={len(b)}' if len(b) == 0 else f'new={len(a)}'
    d = np.hypot(b[:, None, 0] - a[None, :, 0], b[:, None, 1] - a[None, :, 1])
    gone = np.flatnonzero(d.min(axis=1) > min_move_px)
    new = np.flatnonzero(d.min(axis=0) > min_move_px)
    if len(gone) != 1:
        return None, None, f'moved={len(gone)}'
    if len(new) != 1:
        return None, None, f'new={len(new)}'
    return tuple(b[gone[0]]), tuple(a[new[0]]), ''


def free_heading(cands, others, poly, reach, step_deg=30):
    """Field heading (deg, CCW from +x) with the most room for a move of
    `reach` cm from ANY of `cands` (the robot is one of them, not yet known
    which). Room = the end point's clearance to the arena edge and to every
    blob in `others` other than the candidate itself, worst case over the
    candidates. All in field cm. Returns (heading_deg, room_cm).
    """
    poly = np.asarray(poly, np.float32).reshape(-1, 1, 2)
    others = np.asarray(others, float).reshape(-1, 2)
    best, best_room = 0.0, -math.inf
    for deg in range(0, 360, step_deg):
        c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        room = math.inf
        for p in cands:
            q = (p[0] + reach * c, p[1] + reach * s)
            r = cv2.pointPolygonTest(poly, (float(q[0]), float(q[1])), True)
            if len(others):
                d_self = np.hypot(*(others - p).T)
                rest = others[d_self > 1.0]          # not the candidate itself
                if len(rest):
                    r = min(r, float(np.hypot(*(rest - q).T).min()))
            room = min(room, r)
        if room > best_room:
            best, best_room = float(deg), room
    return best, best_room

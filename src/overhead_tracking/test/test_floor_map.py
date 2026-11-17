#!/usr/bin/env python3
"""Floor map: drift detection with hold, background rebuild, recovery."""

import os
import threading
import time

import cv2
import numpy as np

from overhead_tracking import blobs as B
from overhead_tracking import floor_map as F

H, W = 600, 800
ARENA = np.array([[100.0, 80.0], [700.0, 80.0], [700.0, 520.0], [100.0, 520.0]])
CENTRES = [(140, 120), (660, 120), (660, 480), (140, 480), (400, 200), (300, 400)]
DT = 1 / 60


def floor():
    """Dim, uneven floor: ~20 in the middle falling to ~8 in the corners."""
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    r2 = ((xx - W / 2) / (W / 2)) ** 2 + ((yy - H / 2) / (H / 2)) ** 2
    return 20.0 - 6.0 * r2


def scene(centres=CENTRES, scale=1.0, gain=4.0):
    """Robots at 4x the local floor and tape at 2.6x, all lit by `scale`."""
    base = floor()
    img = base.copy()
    tape = np.zeros((H, W), np.uint8)
    cv2.line(tape, (110, 300), (690, 300), 1, 6)
    img[tape > 0] = 2.6 * base[tape > 0]
    for cx, cy in centres:
        m = np.zeros((H, W), np.uint8)
        cv2.circle(m, (int(cx), int(cy)), 16, 1, -1)
        img[m > 0] = gain * base[m > 0]
    return np.clip(np.round(img * scale), 0, 255).astype(np.uint8)


def wait_idle(fm, timeout=10.0):
    end = time.monotonic() + timeout
    while fm.busy and time.monotonic() < end:
        time.sleep(0.01)
    assert not fm.busy


def run_drift(fm, frame, t0, secs):
    """Feed `frame` at 60 Hz; return the time of the first rebuild flag (or None)."""
    t = t0
    while t < t0 + secs:
        if fm.observe(frame, t):
            return t
        t += DT
    return None


def built():
    roi = B.make_roi(ARENA, (H, W))
    fm = F.FloorMap(roi)
    fm.rebuild([scene()])
    return roi, fm


def check_drift_recovers(scale):
    roi, fm = built()
    assert len(B.detect(scene(), roi, fm.tmap)) == len(CENTRES)
    dim = scene(scale=scale)
    t = run_drift(fm, dim, 0.0, 3.0)
    assert t is not None and 2.0 <= t < 2.0 + 3 * DT          # fires only after the hold
    assert len(B.detect(dim, roi, fm.tmap)) != len(CENTRES)    # stale map is wrong
    assert fm.start_rebuild()
    wait_idle(fm)
    assert fm.rebuild_count == 2 and fm.last_rebuild_sec > 0
    assert abs(fm.ref_level - fm.level) <= 1
    assert len(B.detect(dim, roi, fm.tmap)) == len(CENTRES)
    assert not fm.observe(dim, t + DT)


def test_dimmer_room_robots_missed_until_rebuild():
    check_drift_recovers(0.7)


def test_brighter_room_tape_false_blob_until_rebuild():
    check_drift_recovers(1.3)


def test_transient_shadow_does_not_trigger():
    _, fm = built()
    assert run_drift(fm, scene(scale=0.5), 0.0, 0.5) is None
    assert run_drift(fm, scene(), 0.5, 3.0) is None
    assert run_drift(fm, scene(scale=0.5), 3.5, 1.9) is None   # hold restarts


def test_moving_robots_do_not_trigger_or_bias_level():
    _, fm = built()
    empty_level = fm._level(scene(centres=[])[fm.roi['sl']])
    rng = np.random.default_rng(0)
    t = 0.0
    for _ in range(60):                                   # 12 robots wandering
        pts = rng.uniform((130, 110), (670, 490), size=(12, 2))
        f = scene(centres=pts)
        assert not fm.observe(f, t)
        assert abs(fm.level - empty_level) <= 1
        t += 0.1


def test_background_rebuild_keeps_old_map_until_swap(monkeypatch):
    roi, fm = built()
    old = fm.tmap
    bright = scene(scale=1.3)
    run_drift(fm, bright, 0.0, 3.0)
    gate = threading.Event()
    real = F.threshold_map

    def slow(*a, **k):
        gate.wait(5)
        return real(*a, **k)
    monkeypatch.setattr(F, 'threshold_map', slow)
    assert fm.start_rebuild()
    assert fm.busy and not fm.start_rebuild()             # one build at a time
    assert fm.tmap is old                                  # old map still serves
    assert len(B.detect(scene(), roi, fm.tmap)) == len(CENTRES)
    assert not fm.observe(bright, 10.0)                    # no re-flag while building
    gate.set()
    wait_idle(fm)
    assert fm.tmap is not old and fm.rebuild_count == 2
    assert len(B.detect(bright, roi, fm.tmap)) == len(CENTRES)


# Real frame: see test_blobs.py. Robots there sit at 3.4-5.6x, so a 0.7x dim
# only shrinks them (~25% less area); 0.5x loses robots outright.
FIXTURE = os.path.join(os.path.dirname(__file__), 'data', 'arena_12_robots.png')
FIXTURE_OFFSET = np.array([275, 35])
ARENA_PX = np.array([[318.5, 60.0], [1660.0, 83.5], [1680.5, 1056.0], [300.0, 1065.0]])


def fixture_rescaled(scale):
    """Blobs on the rescaled frame with the stale map, then after the rebuild."""
    g = cv2.imread(FIXTURE, cv2.IMREAD_GRAYSCALE)
    roi = B.make_roi(ARENA_PX - FIXTURE_OFFSET, g.shape)
    fm = F.FloorMap(roi)
    fm.rebuild([g])
    assert len(B.detect(g, roi, fm.tmap)) == 12
    h = np.clip(np.round(g * scale), 0, 255).astype(np.uint8)
    before = B.detect(h, roi, fm.tmap)
    t = run_drift(fm, h, 0.0, 3.0)
    assert t is not None and t >= 2.0
    assert fm.start_rebuild()
    wait_idle(fm)
    after = B.detect(h, roi, fm.tmap)
    return before, after


def test_recorded_frame_dimmed_recovers_after_rebuild():
    before, after = fixture_rescaled(0.7)
    assert len(after) == 12
    area = np.median([b['Area'] for b in before])
    assert np.median([b['Area'] for b in after]) > 1.15 * area
    before, after = fixture_rescaled(0.5)
    assert len(before) < 12 and len(after) == 12


def test_recorded_frame_brightened_false_blobs_gone_after_rebuild():
    before, after = fixture_rescaled(1.3)
    assert len(before) > 12 and len(after) == 12

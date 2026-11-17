#!/usr/bin/env python3
"""TrackManager on simulated robots: crossings, touching, hiding, rate independence.

Truth robots obey the filter's own motion model (speed applied 0.45 s late,
field heading = offset - cmd turned at <= 180 deg/s), integrated at 600 Hz and
sampled at the processing rate. The camera model mirrors blobs.detect: two
robots closer than a ball diameter form one blob of their union area, split
(Split=True, one blob per robot) when >= 1.6x the reference area, otherwise
reported as one blob at the pair's centroid with no LED heading.
"""

import math

import cv2
import numpy as np

from overhead_tracking import blobs as blobs_mod
from overhead_tracking.ekf import wrap
from overhead_tracking.track_manager import (COASTING, HEALTHY, LOST, MERGED,
                                             SUSPECT, UNRESOLVED, Blob,
                                             TrackManager)

K = 0.61                    # speed -> cm/s
LAG = 0.45
RATE = math.radians(180.0)
BALL_D = 7.3
REF = 400.0                 # px, ball area at link time
SUB = 600.0                 # truth integration rate


class Robot:
    def __init__(self, name, x, y, theta_deg, offset_deg=0.0):
        self.name, self.x, self.y = name, float(x), float(y)
        self.th = math.radians(theta_deg)
        self.offset = offset_deg
        self.cmds = [(-10.0, 0.0, (offset_deg - theta_deg) % 360.0)]

    def cmd(self, t, speed, field_deg):
        self.cmds.append((t, speed, (self.offset - field_deg) % 360.0))

    def _at(self, t):
        last = None
        for c in self.cmds:
            if c[0] <= t:
                last = c
        return last

    def advance(self, t, dt):
        c = self._at(t - LAG)
        v = c[1] * K
        target = math.radians(self.offset - self._at(t)[2])
        self.th = wrap(self.th + max(-RATE * dt, min(RATE * dt, wrap(target - self.th))))
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt


def union_frac(d):
    """Area of two touching/overlapping unit-diameter discs / one disc."""
    r = BALL_D / 2
    if d >= 2 * r:
        return 2.0
    lens = 2 * r * r * math.acos(d / (2 * r)) - 0.5 * d * math.sqrt(4 * r * r - d * d)
    return 2.0 - lens / (math.pi * r * r)


def camera(robots, rng, leds=True, hidden=(), pos_noise=0.2, th_noise=0.03):
    vis = [r for r in robots if r.name not in hidden]
    out, done = [], set()
    for i, a in enumerate(vis):
        if a.name in done:
            continue
        for b in vis[i + 1:]:
            if b.name in done:
                continue
            d = math.hypot(a.x - b.x, a.y - b.y)
            if d < BALL_D:
                done |= {a.name, b.name}
                f = union_frac(d)
                if f >= 1.6:
                    for r in (a, b):
                        out.append(Blob(r.x + rng.normal(0, pos_noise),
                                        r.y + rng.normal(0, pos_noise),
                                        REF * f / 2, True, None))
                else:
                    out.append(Blob(0.5 * (a.x + b.x), 0.5 * (a.y + b.y),
                                    REF * f, False, None))
                break
        else:
            done.add(a.name)
            out.append(Blob(a.x + rng.normal(0, pos_noise), a.y + rng.normal(0, pos_noise),
                            REF * rng.uniform(0.9, 1.1), False,
                            wrap(a.th + rng.normal(0, th_noise)) if leds else None))
    rng.shuffle(out)
    return out


def run(robots, t_end, fps=60, leds=True, hide=lambda t: (), seed=0, mgr=None,
        known_theta=True):
    """Simulate, feeding the manager. Returns (mgr, history) with history a list
    of (t, {name: (status, x, y)}, {name: (true x, true y)})."""
    rng = np.random.default_rng(seed)
    mgr = mgr or TrackManager()
    for r in robots:
        mgr.add_track(r.name, 0.0, (r.x, r.y), r.th if known_theta else None, REF)
        for c in r.cmds[1:]:
            mgr.add_cmd(r.name, *c)
    n_sub = int(SUB / fps)
    hist = []
    frame = 0
    t = 0.0
    while t < t_end - 1e-9:
        frame += 1
        for k in range(n_sub):
            ts = (frame - 1) / fps + (k + 1) / SUB
            for r in robots:
                r.advance(ts, 1.0 / SUB)
        t = frame / fps
        mgr.step(t, camera(robots, rng, leds, hide(t)))
        hist.append((t, {n: (tr.status, *tr.pos_cm) for n, tr in mgr.tracks.items()},
                     {r.name: (r.x, r.y) for r in robots}))
    return mgr, hist


def swaps(hist, tol=3.0):
    """Frames where a track that claims to be tracked (HEALTHY / SUSPECT) is
    more than `tol` cm off its own robot and nearer another one. (Mid-overlap
    the two robots are < tol apart and 'nearest' means nothing.)"""
    bad = 0
    for _t, est, truth in hist:
        for n, (st, x, y) in est.items():
            if st not in (HEALTHY, SUSPECT):
                continue
            d = {m: math.hypot(truth[m][0] - x, truth[m][1] - y) for m in truth}
            bad += d[n] > tol and min(d, key=d.get) != n
    return bad


def statuses(hist, name):
    return {e[name][0] for _t, e, _tr in hist}


def final_err(mgr, robots):
    return max(math.hypot(mgr.tracks[r.name].pos_cm[0] - r.x,
                          mgr.tracks[r.name].pos_cm[1] - r.y) for r in robots)


# ---------------------------------------------------------------- scenarios
def head_on():
    a, b = Robot('A', 60, 90, 0), Robot('B', 180, 90, 180, offset_deg=37)
    a.cmd(0.1, 50, 0)
    b.cmd(0.1, 50, 180)
    return [a, b]


def right_angle():
    # A along +x, B along +y; both reach (120, 90) at the same time
    a, b = Robot('A', 60, 90, 0), Robot('B', 120, 30, 90, offset_deg=200)
    a.cmd(0.1, 50, 0)
    b.cmd(0.1, 50, 90)
    return [a, b]


def close_pass():
    # opposite lanes 10 cm apart (2.7 cm of air between the shells)
    a, b = Robot('A', 60, 85, 0), Robot('B', 180, 95, 180)
    a.cmd(0.1, 50, 0)
    b.cmd(0.1, 50, 180)
    return [a, b]


def test_head_on_crossing_keeps_identities_with_leds():
    robots = head_on()
    mgr, hist = run(robots, 5.0)
    assert swaps(hist) == 0
    assert MERGED in statuses(hist, 'A')            # the overlap was seen as one blob
    for n in 'AB':
        assert not mgr.tracks[n].suspect
        assert mgr.tracks[n].status == HEALTHY
    assert final_err(mgr, robots) < 3.0


def test_right_angle_crossing_keeps_identities_with_leds():
    robots = right_angle()
    mgr, hist = run(robots, 5.0)
    assert swaps(hist) == 0
    assert not any(tr.suspect for tr in mgr.tracks.values())
    assert final_err(mgr, robots) < 3.0


def test_identical_robots_without_heading_are_flagged_suspect():
    # No LED heading ever: theta unknown, so the EKF cannot apply the speed and
    # both predictions sit still. Once the robots pass through each other the
    # two assignments cost the same -> the manager picks one and flags both.
    robots = head_on()
    mgr, hist = run(robots, 5.0, leds=False, known_theta=False)
    assert mgr.tracks['A'].suspect and mgr.tracks['B'].suspect
    assert mgr.tracks['A'].status == SUSPECT


def test_close_pass_is_not_ambiguous():
    robots = close_pass()
    mgr, hist = run(robots, 5.0)
    assert swaps(hist) == 0
    for n in 'AB':
        assert statuses(hist, n) == {HEALTHY}
    assert final_err(mgr, robots) < 3.0


def test_close_pass_without_heading_is_still_followed():
    # every frame updates both tracks, so continuity alone keeps them apart
    robots = close_pass()
    mgr, hist = run(robots, 5.0, leds=False, known_theta=False)
    assert swaps(hist) == 0
    assert not any(tr.suspect for tr in mgr.tracks.values())


def test_touch_then_separate_through_blobs_detect():
    """Rendered frames through blobs.detect with ref_area: the touching pair is
    split by k-means and both tracks stay HEALTHY, without and with LEDs."""
    cv2.setRNGSeed(1)
    px_cm = 5.5
    r_px = BALL_D / 2 * px_cm
    corners = np.array([[20, 20], [1380, 20], [1380, 1030], [20, 1030]], float)
    roi = blobs_mod.make_roi(corners, (1050, 1400))
    tmap = np.full(roi['mask'].shape, 30, np.uint8)
    # each covers 0.85 s x 30.5 cm/s = 25.9 cm before stopping: they end 7.0 cm
    # apart (shells overlapping by ~1.5 px, as blurred real blobs do), sit for
    # 1.2 s, then back apart
    a, b = Robot('A', 100, 90, 0), Robot('B', 158.85, 90, 180)
    for r, h in ((a, 0), (b, 180)):
        r.cmd(0.1, 50, h)
    a.cmd(0.95, 0, 0)
    b.cmd(0.95, 0, 180)
    a.cmd(2.6, 50, 180)
    b.cmd(2.6, 50, 0)
    robots = [a, b]
    mgr = TrackManager()
    for r in robots:
        mgr.add_track(r.name, 0.0, (r.x, r.y), r.th, math.pi * r_px ** 2)
        for c in r.cmds[1:]:
            mgr.add_cmd(r.name, *c)
    rng = np.random.default_rng(0)
    min_gap, hist = 1e9, []
    for f in range(1, 300):
        for k in range(10):
            for r in robots:
                r.advance((f - 1) / 60 + (k + 1) / SUB, 1 / SUB)
        img = np.full((1050, 1400), 10, np.uint8)
        for r in robots:
            cv2.circle(img, (int(r.x * px_cm), int(r.y * px_cm)), int(r_px), 150, -1)
        found = blobs_mod.detect(img, roi, tmap, ref_area=mgr.ref_area)
        ms = []
        for bl in found:
            x, y = bl['Centroid'][0] / px_cm, bl['Centroid'][1] / px_cm
            near = min(robots, key=lambda r: math.hypot(r.x - x, r.y - y))
            th = None if bl['Split'] else wrap(near.th + rng.normal(0, 0.03))
            ms.append(Blob(x, y, bl['Area'], bl['Split'], th))
        mgr.step(f / 60, ms)
        min_gap = min(min_gap, math.hypot(a.x - b.x, a.y - b.y) - BALL_D)
        hist.append((f / 60, {n: (tr.status, *tr.pos_cm) for n, tr in mgr.tracks.items()},
                     {r.name: (r.x, r.y) for r in robots}))
    assert min_gap < 0.0                             # they really touched
    assert swaps(hist) == 0
    assert statuses(hist, 'A') == {HEALTHY} and statuses(hist, 'B') == {HEALTHY}
    assert final_err(mgr, robots) < 3.0


def hide_scenario(hide_for):
    a = Robot('A', 60, 90, 0)
    a.cmd(0.1, 50, 0)
    b = Robot('B', 60, 140, 0)                        # static neighbour, 50 cm away
    return [a, b], (lambda t: {'A'} if 1.5 < t <= 1.5 + hide_for else ())


def test_hidden_0p3_s_coasts_and_is_picked_up():
    robots, hide = hide_scenario(0.3)
    mgr, hist = run(robots, 3.0, hide=hide)
    st = statuses(hist, 'A')
    assert COASTING in st and LOST not in st
    assert mgr.tracks['A'].status == HEALTHY and swaps(hist) == 0
    # the 0.3 s blind prediction was good enough to land inside the gate
    assert final_err(mgr, robots) < 3.0


def test_hidden_1_s_goes_lost_then_is_reacquired():
    robots, hide = hide_scenario(1.0)
    mgr, hist = run(robots, 3.5, hide=hide)
    lost_t = [t for t, e, _ in hist if e['A'][0] == LOST]
    assert lost_t and abs(lost_t[0] - 2.0) < 0.05     # 0.5 s after the last fix
    assert mgr.tracks['A'].status == HEALTHY and swaps(hist) == 0
    assert not mgr.tracks['A'].suspect


def three_static(spots):
    return [Robot(n, x, y, 0) for n, (x, y) in zip('ABC', spots)]


def test_three_lost_at_once_reacquire_where_unambiguous():
    spots = [(100, 100), (130, 100), (115, 126)]      # 30 cm apart: inside each other's gates
    robots = three_static(spots)
    mgr, hist = run(robots, 3.0, hide=lambda t: {'A', 'B', 'C'} if 1.0 < t <= 2.0 else ())
    mid = [e for t, e, _ in hist if 1.6 < t <= 2.0]
    assert all(e[n][0] == LOST for e in mid for n in 'ABC')
    assert all(mgr.tracks[n].status == HEALTHY for n in 'ABC')
    assert swaps(hist) == 0


def test_several_lost_do_not_reacquire_an_ambiguous_blob():
    spots = [(100, 100), (130, 100)]
    robots = three_static(spots)
    mgr, _ = run(robots, 2.0, hide=lambda t: {'A', 'B'} if t > 1.0 else ())
    assert mgr.tracks['A'].status == LOST
    # one robot reappears exactly between the two lost tracks
    mgr.step(2.0 + 1 / 60, [Blob(115.0, 100.0, REF)])
    assert mgr.tracks['A'].status == LOST and mgr.tracks['B'].status == LOST
    # ... and nearer A: taken
    mgr.step(2.0 + 2 / 60, [Blob(103.0, 100.0, REF)])
    assert mgr.tracks['A'].status == HEALTHY and mgr.tracks['B'].status == LOST


def test_lost_becomes_unresolved_after_timeout():
    robots = three_static([(100, 100)])
    mgr, hist = run(robots, 4.0, hide=lambda t: {'A'} if t > 1.0 else ())
    assert mgr.tracks['A'].status == UNRESOLVED
    first = next(t for t, e, _ in hist if e['A'][0] == UNRESOLVED)
    assert abs(first - (1.0 + 0.5 + 2.0)) < 0.05


def test_same_behaviour_at_60_and_30_hz():
    scenarios = [(head_on, {}), (right_angle, {}), (close_pass, {}),
                 (lambda: hide_scenario(1.0)[0], {'hide': hide_scenario(1.0)[1]}),
                 (head_on, {'leds': False, 'known_theta': False})]
    for make, kw in scenarios:
        r60, r30 = make(), make()
        m60, h60 = run(r60, 4.0, fps=60, **kw)
        m30, h30 = run(r30, 4.0, fps=30, **kw)

        # Which track wins the one robot-sized blob at full overlap is a coin
        # flip on the noise, so frame-level HEALTHY/COASTING/MERGED may differ;
        # the time-based ladder (LOST / UNRESOLVED), identities, the SUSPECT
        # outcome and the estimates must not.
        def ladder(e):
            return {n: s[0] if s[0] in (LOST, UNRESOLVED) else 'tracked'
                    for n, s in e.items()}
        by_t = {round(t, 6): e for t, e, _ in h60}
        for t, e30, _ in h30:
            assert ladder(e30) == ladder(by_t[round(t, 6)]), t
        assert abs(swaps(h60) / 60 - swaps(h30) / 30) < 0.1      # seconds swapped
        for n in m60.tracks:
            assert m60.tracks[n].suspect == m30.tracks[n].suspect
            assert math.dist(m60.tracks[n].pos_cm, m30.tracks[n].pos_cm) < 1.0


def test_gate_grows_with_covariance_between_bounds():
    mgr = TrackManager()
    tr = mgr.add_track('A', 0.0, (100.0, 100.0), 0.0, REF)
    blind = mgr.add_track('B', 0.0, (100.0, 300.0), None, REF)   # no heading yet
    mgr.step(1 / 60, [])
    assert not mgr._in_gate(tr, Blob(117.0, 100.0))          # > min 15 cm, P still small
    mgr.step(0.4, [])                                         # coasting: P = 1 + 100 t
    assert mgr._in_gate(tr, Blob(118.0, 100.0))               # 3 sigma = 19.2 cm
    assert not mgr._in_gate(tr, Blob(121.0, 100.0))
    assert mgr._in_gate(blind, Blob(140.0, 300.0))            # 1 + 500 t: 42.5 cm
    mgr.step(3.0, [])
    assert mgr._in_gate(tr, Blob(149.0, 100.0))
    assert not mgr._in_gate(tr, Blob(151.0, 100.0))           # never beyond max


def test_ref_area_is_median_of_link_areas():
    mgr = TrackManager()
    assert mgr.ref_area is None
    for n, a in zip('ABC', (380, 420, 900)):
        mgr.add_track(n, 0.0, (0.0, 0.0), area_px=a)
    assert mgr.ref_area == 420


def test_cached_distance_matches_the_ekf():
    mgr = TrackManager()
    tr = mgr.add_track('A', 0.0, (100.0, 100.0), 0.3, REF)
    mgr.add_cmd('A', 0.0, 50, 20)
    mgr.step(0.7, [])
    for b in (Blob(104.0, 97.0, theta=1.0), Blob(90.0, 101.0),
              Blob(95.0, 99.0, split=True, theta=2.0)):
        th = None if b.split else b.theta
        assert math.isclose(mgr._d2(tr, b), tr.ekf.mahalanobis2((b.x, b.y), th), rel_tol=1e-9)


def test_track_linked_without_heading_takes_its_first_led_fix():
    mgr = TrackManager()
    mgr.add_track('A', 0.0, (100.0, 80.0))
    assert not mgr.tracks['A'].ekf.theta_known
    for k in range(1, mgr.first_led_n):
        mgr.step(k / 60.0, [Blob(100.0, 80.0, theta=1.0)])
    assert not mgr.tracks['A'].ekf.theta_known       # needs first_led_n readings
    mgr.step(mgr.first_led_n / 60.0, [Blob(100.0, 80.0, theta=1.0)])
    assert mgr.tracks['A'].ekf.theta_known
    assert abs(wrap(mgr.tracks['A'].theta - 1.0)) < 0.05


def _first_led(thetas, dt=1 / 60.0, n=3):
    """Feed a heading-less track (first_led_n=n) one blob per frame with
    these LED readings (None = no reading); returns the track."""
    mgr = TrackManager(first_led_n=n)
    mgr.add_track('A', 0.0, (100.0, 80.0))
    for k, th in enumerate(thetas, 1):
        mgr.step(k * dt, [Blob(100.0, 80.0, theta=th)])
    return mgr.tracks['A']


def test_single_outlier_led_reading_does_not_set_heading():
    tr = _first_led([2.5, None, None, None])
    assert not tr.ekf.theta_known


def test_three_consistent_led_readings_set_heading_from_their_mean():
    tr = _first_led([1.0, 1.2])
    assert not tr.ekf.theta_known
    tr = _first_led([1.0, 1.2, 1.1])
    assert tr.ekf.theta_known
    assert abs(wrap(tr.theta - 1.1)) < 0.01
    # agreement is circular: readings either side of +-pi agree
    tr = _first_led([math.pi - 0.1, -math.pi + 0.1, math.pi])
    assert tr.ekf.theta_known
    assert abs(wrap(tr.theta - math.pi)) < 0.01


def test_inconsistent_led_readings_never_set_heading():
    # scattered over 160-270 deg, like a robot with dead LEDs
    tr = _first_led([math.radians(a) for a in (160, 270, 200, 250, 165, 230, 190, 270)])
    assert not tr.ekf.theta_known
    # ... and at the default first_led_n, readings like SB-3660's false burst
    # (kf_20261005_171405, deg): every 3 in a row are within 20 deg somewhere
    tr = _first_led([math.radians(a) for a in (
        256, 234, 206, 229, 252, 267, 212, 201, 206, 217, 208, 232, 214, 189, 215,
        183, 220, 181, 205, 162)], n=TrackManager().first_led_n)
    assert not tr.ekf.theta_known
    # agreeing readings spread wider than the window
    tr = _first_led([1.0, None, None, None, None, 1.0, None, None, None, None, 1.0],
                    dt=0.1)
    assert not tr.ekf.theta_known


def test_pending_led_readings_reset_on_relink_and_lost():
    mgr = TrackManager()
    mgr.add_track('A', 0.0, (100.0, 80.0))
    mgr.step(1 / 60.0, [Blob(100.0, 80.0, theta=1.0)])
    mgr.step(2 / 60.0, [Blob(100.0, 80.0, theta=1.0)])
    mgr.add_track('A', 3 / 60.0, (100.0, 80.0))
    mgr.step(4 / 60.0, [Blob(100.0, 80.0, theta=1.0)])
    assert not mgr.tracks['A'].ekf.theta_known
    assert len(mgr.tracks['A'].led_pending) == 1
    mgr.step(0.7, [])                                  # > lost_after_s unseen
    assert mgr.tracks['A'].state == LOST
    assert mgr.tracks['A'].led_pending == []

#!/usr/bin/env python3
"""Node smoke tests.

Follows the workspace pattern: no executor, no spin, no processing thread.
Construct the node, poke private attributes, call the private handlers and
`tick()` (one `_process_frame`) directly. Everything runs against
SimOverheadSource, which renders robots on a realistic floor, so the actual
blob detector and floor map are exercised rather than a stub.
"""

import json
import math
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.parameter import Parameter                            # noqa: E402
from std_srvs.srv import Trigger                                 # noqa: E402

from overhead_tracking.overhead_tracker_node import (            # noqa: E402
    ARENA_READY, BLOBS_READY, COASTING, HEALTHY, LINKING, LOST, SUSPECT,
    TRACKING, UNRESOLVED, OverheadTrackerNode)

# an arena quad well inside a 1280x720 frame
QUAD = [80.0, 650.0, 1200.0, 660.0, 1210.0, 60.0, 70.0, 50.0]
LONG, SHORT = 200.0, 120.0


def make_node(tmp_path, positions=None, **extra):
    params = {
        'source': 'sim', 'frame_width': 1280, 'frame_height': 720,
        'arena_long_edge_cm': LONG, 'arena_short_edge_cm': SHORT,
        'manual_corners_px': QUAD, 'arena_source': 'manual',
        'arena_yaml_path': str(tmp_path / 'arena.yaml'),
        'auto_load_arena': False, 'track_rate_hz': 30.0,
        'publish_rate_hz': 10.0, 'annotate_rate_hz': 1000.0,
        'control_reapply_delay_sec': 1000.0,     # keep the one-shot timer away
        'sim_spheros': 4, 'seed': 5,
    }
    params.update(extra)
    overrides = [Parameter(k, value=v) for k, v in params.items()]
    node = OverheadTrackerNode(parameter_overrides=overrides)
    if positions is not None:
        node._source.set_positions(positions)
        node._source.on_tick()
    return node


def tick(node):
    """One processing step on the source's current frame: what the processing
    thread does per frame (minus the 60/30 decimation), without the thread."""
    node._process_frame(node._source.latest_frame())


class Harness:
    """Node + captured pose publishes."""

    def __init__(self, tmp_path, positions=None, **extra):
        self.node = make_node(tmp_path, positions, **extra)
        self.poses = {}
        orig = self.node._publish_pose

        def spy(name, x, y, stamp, theta=None):
            self.poses.setdefault(name, []).append((x, y))
            return orig(name, x, y, stamp, theta)
        self.node._publish_pose = spy

    def arena(self):
        r = self.node._on_detect_arena(Trigger.Request(), Trigger.Response())
        assert r.success, r.message
        return r

    def blobs(self):
        return self.node._on_detect_spheros(Trigger.Request(), Trigger.Response())

    def add_track(self, name, x_cm, y_cm, area=None):
        self.node._ensure_robot_io(name)
        self.node._lock_tracker(name, x_cm, y_cm, area)
        return self.node._mgr.tracks[name]

    def frame(self, stamp):
        """Render the next sim frame with a SYNTHETIC stamp, then tick: the
        tracker runs on frame time, so tests control time exactly."""
        self.node._source.on_tick()
        self.node._source._stamp = stamp
        tick(self.node)

    def cm_of(self, u, v):
        _c, H, _hi, _p = self.node._arena_snapshot()
        from overhead_tracking import homography as hg
        return hg.apply_homography(u, v, H)

    def close(self):
        self.node.destroy_node()


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


# --------------------------------------------------------------- construction
def test_node_constructs_in_sim(ros, tmp_path):
    h = Harness(tmp_path)
    try:
        assert h.node._source is not None
        gray, stamp = h.node._latest_frame()
        assert gray is not None and gray.shape == (720, 1280) and stamp > 0
    finally:
        h.close()


def test_state_json_is_publishable(ros, tmp_path):
    h = Harness(tmp_path)
    try:
        captured = []
        h.node._state_pub.publish = lambda m: captured.append(m)
        h.node._publish_state()
        payload = json.loads(captured[-1].data)
        assert 'state' in payload and 'diag' in payload and 'camera' in payload
    finally:
        h.close()


# ---------------------------------------------------------------------- arena
def test_detect_arena_manual_corners(ros, tmp_path):
    h = Harness(tmp_path)
    try:
        r = h.arena()
        assert h.node._get_state() == ARENA_READY
        assert np.all(np.isfinite(h.node._arena_H))
        assert 'residual_max' in r.message
        assert (tmp_path / 'arena.yaml').exists()
    finally:
        h.close()


def test_arena_maps_corner_pixels_to_the_measured_rectangle(ros, tmp_path):
    h = Harness(tmp_path)
    try:
        h.arena()
        xs, ys = zip(*[h.cm_of(QUAD[i], QUAD[i + 1]) for i in range(0, 8, 2)])
        assert min(xs) == pytest.approx(0.0, abs=1e-6)
        assert max(xs) == pytest.approx(LONG, abs=1e-6)
        assert max(ys) == pytest.approx(SHORT, abs=1e-6)
    finally:
        h.close()


def test_detect_arena_rejected_while_tracking(ros, tmp_path):
    h = Harness(tmp_path)
    try:
        h.arena()
        h.node._set_state(TRACKING)
        r = h.node._on_detect_arena(Trigger.Request(), Trigger.Response())
        assert not r.success and 'reset' in r.message
        assert h.node._get_state() == TRACKING          # state not demoted
    finally:
        h.close()


def test_missing_dimensions_fail_cleanly(ros, tmp_path):
    h = Harness(tmp_path, arena_long_edge_cm=0.0)
    try:
        r = h.node._on_detect_arena(Trigger.Request(), Trigger.Response())
        assert not r.success and 'edge_cm' in r.message
    finally:
        h.close()


def test_arena_persists_and_reloads(ros, tmp_path):
    h = Harness(tmp_path)
    try:
        h.arena()
        H0 = h.node._arena_H.copy()
    finally:
        h.close()
    h2 = Harness(tmp_path, auto_load_arena=True)
    try:
        assert h2.node._arena is not None
        assert np.allclose(h2.node._arena_H, H0)
        assert h2.node._get_state() == ARENA_READY
    finally:
        h2.close()


# ---------------------------------------------------------------------- blobs
def test_detect_spheros_counts_sim_robots(ros, tmp_path):
    pos = [(300.0, 300.0), (600.0, 350.0), (900.0, 400.0), (500.0, 500.0)]
    h = Harness(tmp_path, positions=pos)
    try:
        h.arena()
        r = h.blobs()
        assert r.success, r.message
        blobs = json.loads(r.message)
        assert len(blobs) == 4
        assert h.node._get_state() == BLOBS_READY
        for b in blobs:
            assert 0.0 <= b['x_cm'] <= LONG and 0.0 <= b['y_cm'] <= SHORT
    finally:
        h.close()


def test_detect_spheros_reports_brightness_when_empty(ros, tmp_path):
    h = Harness(tmp_path, positions=[])
    try:
        h.arena()
        r = h.blobs()
        assert not r.success
        assert 'p50' in r.message and 'ratio' in r.message
    finally:
        h.close()


def test_detect_spheros_requires_an_arena(ros, tmp_path):
    h = Harness(tmp_path)
    try:
        r = h.blobs()
        assert not r.success and 'detect_arena' in r.message
    finally:
        h.close()


def test_blobs_outside_the_arena_are_dropped(ros, tmp_path):
    h = Harness(tmp_path, positions=[(300.0, 300.0), (1265.0, 700.0)])
    try:
        h.arena()
        r = h.blobs()
        assert r.success
        assert len(json.loads(r.message)) == 1
    finally:
        h.close()


def test_max_robots_caps_the_snapshot(ros, tmp_path):
    pos = [(150.0 + 70 * i, 200.0 + 60 * (i % 5)) for i in range(8)]
    h = Harness(tmp_path, positions=pos, max_robots=3)
    try:
        h.arena()
        assert len(json.loads(h.blobs().message)) == 3
    finally:
        h.close()


def test_detect_arena_builds_the_floor_map(ros, tmp_path):
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        assert h.node._floor.tmap is not None
        captured = []
        h.node._state_pub.publish = lambda m: captured.append(m)
        h.node._publish_state()
        assert json.loads(captured[-1].data)['floor']['rebuild_count'] == 1
    finally:
        h.close()


def test_rebuild_floor_service(ros, tmp_path):
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        r = h.node._on_rebuild_floor(Trigger.Request(), Trigger.Response())
        assert not r.success and 'detect_arena' in r.message
        h.arena()
        n0 = h.node._floor.rebuild_count
        r = h.node._on_rebuild_floor(Trigger.Request(), Trigger.Response())
        assert r.success, r.message
        assert h.node._floor.rebuild_count == n0 + 1
        assert json.loads(r.message)['ref_level'] == pytest.approx(16, abs=3)
    finally:
        h.close()


def test_floor_drift_triggers_background_rebuild(ros, tmp_path):
    h = Harness(tmp_path, positions=[(600.0, 350.0)], floor_drift_hold_sec=0.05)
    try:
        h.arena()
        x, y = h.cm_of(600.0, 350.0)
        t = h.add_track('SB-AAAA', x, y)
        h.node._set_state(TRACKING)
        h.node._source.set_gain(0.6)                  # the room dims
        deadline = time.time() + 2.0
        while h.node._floor.rebuild_count < 2 and time.time() < deadline:
            h.node._source.on_tick()
            tick(h.node)
            time.sleep(0.02)
        assert h.node._floor.rebuild_count == 2
        for _ in range(3):
            h.node._source.on_tick()
            tick(h.node)
        assert t.status == HEALTHY
    finally:
        h.close()


def test_auto_loaded_arena_builds_map_lazily(ros, tmp_path):
    h = Harness(tmp_path)
    try:
        h.arena()
    finally:
        h.close()
    h2 = Harness(tmp_path, positions=[(600.0, 350.0)], auto_load_arena=True)
    try:
        assert h2.node._floor is not None and h2.node._floor.tmap is None
        deadline = time.time() + 2.0
        while (h2.node._floor.tmap is None or h2.node._floor.busy) and \
                time.time() < deadline:
            h2.node._source.on_tick()
            tick(h2.node)
            time.sleep(0.02)
        assert h2.node._floor.tmap is not None
    finally:
        h2.close()


def test_touching_pair_splits_with_link_time_area(ros, tmp_path):
    import cv2
    cv2.setRNGSeed(0)                                  # k-means++ seeding
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        area = json.loads(h.blobs().message)[0]['area']
        a, b = (583.0, 350.0), (617.0, 350.0)
        h.node._source.set_positions([a, b])
        h.node._source.on_tick()
        h.node._ensure_robot_io('SB-AAAA')
        h.node._ensure_robot_io('SB-BBBB')
        h.node._lock_tracker('SB-AAAA', *h.cm_of(*a), area)
        h.node._lock_tracker('SB-BBBB', *h.cm_of(*b), area)
        ta, tb = h.node._mgr.tracks['SB-AAAA'], h.node._mgr.tracks['SB-BBBB']
        h.node._set_state(TRACKING)
        tick(h.node)
        for t, p in ((ta, a), (tb, b)):
            assert t.status == HEALTHY
            u, v, _area, split = h.node._last_blob[t.name]
            assert split
            assert np.hypot(u - p[0], v - p[1]) < 4.0
    finally:
        h.close()


def test_detections_keys_stable(ros, tmp_path):
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        h.add_track('SB-AAAA', *h.cm_of(600.0, 350.0))
        h.node._set_state(TRACKING)
        got = []
        h.node._det_pub.publish = lambda m: got.append(m)
        h.node._last_pub = 0.0
        h.node._source.on_tick()
        tick(h.node)
        d = json.loads(got[-1].data)
        r = d['robots'][0]
        assert set(r) == {'name', 'u', 'v', 'x_cm', 'y_cm', 'angle', 'heading_deg',
                          'score', 'area', 'split', 'status', 'miss'}
        assert r['score'] is None and r['angle'] is None
        assert r['area'] > 0 and isinstance(r['split'], bool)
        assert r['status'] == HEALTHY and r['heading_deg'] is None   # no LED yet
        assert d['frame_stamp'] > 0
    finally:
        h.close()


# -------------------------------------------------------------------- linking
def test_link_binds_bijectively(ros, tmp_path):
    pos = [(300.0, 300.0), (600.0, 350.0), (900.0, 400.0)]
    h = Harness(tmp_path, positions=pos)
    try:
        h.arena()
        h.blobs()
        names = ['SB-AAAA', 'SB-BBBB', 'SB-CCCC']
        order = {'SB-AAAA': 1, 'SB-BBBB': 2, 'SB-CCCC': 0}

        def fake_probe(name, centres, used):
            # _probe_one returns (ok, blob_index, pre_frame): the caller keeps
            # the robot lit across probe AND aim, so it needs `pre` back for the
            # dark check after it turns the LEDs off. None skips that check.
            idx = order[name]
            return (idx not in used), idx, None
        h.node._probe_one = fake_probe
        for n in names:
            h.node._fleet_last_seen[n] = time.time()

        req = type('R', (), {'callsigns': names, 'skip_compass': True})()
        resp = type('S', (), {'success': False, 'registered': [], 'failed': [],
                              'message': ''})()
        h.node._on_link_spheros(req, resp)
        assert sorted(resp.registered) == sorted(names)
        assert h.node._get_state() == TRACKING
        assert len(set(h.node._links.values())) == 3     # a bijection
        assert len(h.node._mgr.tracks) == 3
    finally:
        h.close()


def test_link_requires_blobs(ros, tmp_path):
    h = Harness(tmp_path)
    try:
        h.arena()
        req = type('R', (), {'callsigns': ['SB-AAAA'], 'skip_compass': True})()
        resp = type('S', (), {'success': False, 'registered': [], 'failed': [],
                              'message': ''})()
        h.node._on_link_spheros(req, resp)
        assert not resp.success and 'detect_spheros' in resp.message
    finally:
        h.close()


# ----------------------------------------------------------------- the tick
def test_tick_publishes_pose_for_locked_tracker(ros, tmp_path):
    """The guard on the hard output contract: PoseStamped, field frame, CM."""
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        x_cm, y_cm = h.cm_of(600.0, 350.0)
        h.add_track('SB-AAAA', x_cm, y_cm)
        h.node._set_state(TRACKING)

        raw = []
        h.node._pose_pubs['SB-AAAA'].publish = lambda m: raw.append(m)
        h.node._last_pub = 0.0
        h.node._source.on_tick()
        tick(h.node)

        assert raw, 'no pose published'
        m = raw[-1]
        assert m.header.frame_id == 'field'
        assert np.isfinite(m.pose.position.x) and np.isfinite(m.pose.position.y)
        assert 0.0 <= m.pose.position.x <= LONG      # cm, not metres
        assert m.pose.position.z == 0.0 and m.pose.orientation.w == 1.0
    finally:
        h.close()


def test_tick_locks_onto_the_true_position(ros, tmp_path):
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        tx, ty = h.cm_of(600.0, 350.0)
        h.add_track('SB-AAAA', tx + 8.0, ty + 5.0)     # seeded off-target
        h.node._set_state(TRACKING)
        for _ in range(25):
            h.node._source.on_tick()
            tick(h.node)
        gx, gy = h.node._mgr.tracks['SB-AAAA'].pos_cm
        assert abs(gx - tx) < 2.0 and abs(gy - ty) < 2.0
        assert h.node._mgr.tracks['SB-AAAA'].status == HEALTHY
    finally:
        h.close()


def test_publish_is_decimated_to_10hz(ros, tmp_path):
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        x, y = h.cm_of(600.0, 350.0)
        h.add_track('SB-AAAA', x, y)
        h.node._set_state(TRACKING)
        t0 = time.time()
        for _ in range(40):
            h.node._source.on_tick()
            tick(h.node)
        elapsed = max(time.time() - t0, 1e-6)
        n = len(h.poses.get('SB-AAAA', []))
        assert n <= int(elapsed * 10) + 2      # never faster than the contract
    finally:
        h.close()


def test_stale_frame_still_publishes_without_a_step(ros, tmp_path):
    """A missing/stale frame must not create a gap in the pose stream, and must
    not count as a miss either: the tracker only steps on NEW frames."""
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        x, y = h.cm_of(600.0, 350.0)
        t = h.add_track('SB-AAAA', x, y)
        h.node._set_state(TRACKING)
        h.node._source.on_tick()
        tick(h.node)
        t_before = t.t
        h.node._last_pub = 0.0
        tick(h.node)                                # same stamp -> not fresh
        assert len(h.poses['SB-AAAA']) >= 2
        assert t.t == t_before and t.status == HEALTHY
        assert h.node._miss['SB-AAAA'] == 0
    finally:
        h.close()


def test_miss_ladder_in_frame_time_then_reacquire(ros, tmp_path):
    h = Harness(tmp_path, positions=[(600.0, 350.0)], unresolved_after_s=1000.0)
    try:
        h.arena()
        t0 = time.time()
        h.node._source._stamp = t0
        x, y = h.cm_of(600.0, 350.0)
        t = h.add_track('SB-AAAA', x, y)
        h.node._set_state(TRACKING)
        k = 0
        for _ in range(3):
            k += 1
            h.frame(t0 + k / 30.0)
        assert t.status == HEALTHY

        h.node._source.set_positions([])              # robot disappears
        for _ in range(10):                           # 0.33 s < lost_after_s 0.5
            k += 1
            h.frame(t0 + k / 30.0)
        assert t.status == COASTING
        for _ in range(10):                           # 0.67 s unseen
            k += 1
            h.frame(t0 + k / 30.0)
        assert t.status == LOST

        h.node._source.set_positions([(600.0, 350.0)])   # and comes back
        k += 1
        h.frame(t0 + k / 30.0)
        assert t.status == HEALTHY
    finally:
        h.close()


def test_robot_leaving_the_arena_coasts_and_publishing_continues(
        ros, tmp_path):
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        x, y = h.cm_of(600.0, 350.0)
        t = h.add_track('SB-AAAA', x, y)
        h.node._set_state(TRACKING)
        h.node._source.on_tick()
        tick(h.node)

        # outside the arena polygon + detector margin: no blob at all
        h.node._source.set_positions([(1262.0, 705.0)])
        for _ in range(4):
            h.node._source.on_tick()
            tick(h.node)
        assert t.status in (COASTING, LOST, UNRESOLVED)
        h.node._last_pub = 0.0
        tick(h.node)
        assert h.poses['SB-AAAA']                     # stream never gapped
    finally:
        h.close()


def test_a_distant_jump_never_even_reaches_the_gate(ros, tmp_path):
    """A blob far beyond gate_max_cm is never claimed: the track coasts."""
    h = Harness(tmp_path, positions=[(300.0, 300.0)])
    try:
        h.arena()
        x, y = h.cm_of(300.0, 300.0)
        t = h.add_track('SB-AAAA', x, y)
        h.node._set_state(TRACKING)
        h.node._source.on_tick()
        tick(h.node)
        before = t.pos_cm

        h.node._source.set_positions([(900.0, 400.0)])
        h.node._source.on_tick()
        tick(h.node)
        assert abs(t.pos_cm[0] - before[0]) < 20.0
        assert t.status in (COASTING, LOST)
    finally:
        h.close()


def test_gate_refuses_a_blob_outside_the_covariance_gate(ros, tmp_path):
    """A blob further from the prediction than the gate allows (here clamped to
    gate_max_cm 5) is not fused: the track coasts where it was."""
    h = Harness(tmp_path, positions=[(600.0, 350.0)], gate_min_cm=1.0,
                gate_max_cm=5.0)
    try:
        h.arena()
        x, y = h.cm_of(600.0, 350.0)
        t = h.add_track('SB-AAAA', x, y)
        h.node._set_state(TRACKING)
        h.node._source.on_tick()
        tick(h.node)
        before = t.pos_cm

        h.node._source.set_positions([(640.0, 350.0)])     # ~7 cm away
        h.node._source.on_tick()
        tick(h.node)
        assert t.status == COASTING
        assert abs(t.pos_cm[0] - before[0]) < 1.0     # measurement refused
    finally:
        h.close()


def test_camera_updates_suppressed_while_linking(ros, tmp_path):
    """LED probes flash robots all over the arena; a camera update then is how an
    ID swap gets baked in."""
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        x, y = h.cm_of(600.0, 350.0)
        t = h.add_track('SB-AAAA', x + 15.0, y)
        h.node._set_state(LINKING)
        start = t.pos_cm
        for _ in range(6):
            h.node._source.on_tick()
            tick(h.node)
        # not stepped at all while linking
        assert t.pos_cm == start
    finally:
        h.close()


def test_two_robots_do_not_swap_identity(ros, tmp_path):
    """Converge two robots, then separate them, and check each KF kept its own."""
    h = Harness(tmp_path, positions=[(500.0, 350.0), (700.0, 350.0)])
    try:
        h.arena()
        ax, ay = h.cm_of(500.0, 350.0)
        bx, by = h.cm_of(700.0, 350.0)
        ta = h.add_track('SB-AAAA', ax, ay)
        tb = h.add_track('SB-BBBB', bx, by)
        h.node._set_state(TRACKING)

        for a_u, b_u in [(500, 700), (540, 660), (570, 630), (590, 610),
                         (570, 630), (540, 660), (500, 700)]:
            h.node._source.set_positions([(float(a_u), 350.0), (float(b_u), 350.0)])
            for _ in range(2):
                h.node._source.on_tick()
                tick(h.node)

        final_a = ta.pos_cm[0]
        final_b = tb.pos_cm[0]
        assert final_a < final_b, 'tracks swapped sides'
    finally:
        h.close()


def test_no_publisher_is_created_from_the_tick(ros, tmp_path):
    """Creating rclpy entities concurrently on one node is not thread-safe, so
    the tick must never do it -- all per-robot IO is made at link time."""
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        x, y = h.cm_of(600.0, 350.0)
        h.add_track('SB-AAAA', x, y)
        h.node._set_state(TRACKING)
        before = (len(h.node._pose_pubs), len(h.node._led_pubs),
                  len(h.node._motion_subs), len(h.node._offset_pubs))
        for _ in range(10):
            h.node._source.on_tick()
            tick(h.node)
        after = (len(h.node._pose_pubs), len(h.node._led_pubs),
                 len(h.node._motion_subs), len(h.node._offset_pubs))
        assert before == after
    finally:
        h.close()


# ------------------------------------------------------------------- annotate
def push_frames(node, n):
    """Drive n annotate handoffs, bypassing the rate limiter."""
    for _ in range(n):
        node._source.on_tick()
        node._last_annot_push = 0.0
        node._annotate_push_timer()


def test_annotated_frame_is_published(ros, tmp_path):
    import cv2
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        got = []
        h.node._annot_pub.publish = lambda m: got.append(m)
        x, y = h.cm_of(600.0, 350.0)
        h.add_track('SB-AAAA', x, y)
        h.node._set_state(TRACKING)
        push_frames(h.node, 1)
        for _ in range(50):
            if got:
                break
            time.sleep(0.02)
        assert got, 'annotate worker published nothing'
        m = got[-1]
        assert m.format == 'jpeg' and m.header.frame_id == 'overhead_camera'
        img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), 1)
        assert img.shape == (720, 1280, 3)
    finally:
        h.close()


def test_annotated_feed_runs_before_any_robot_is_linked(ros, tmp_path):
    """The operator needs the live view to aim the camera and place the arena
    corners, which happens long before tracking starts."""
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        got = []
        h.node._annot_pub.publish = lambda m: got.append(m)
        assert h.node._get_state() != TRACKING
        push_frames(h.node, 1)
        for _ in range(50):
            if got:
                break
            time.sleep(0.02)
        assert got, 'no annotated frame outside TRACKING'
    finally:
        h.close()


def test_annotate_queue_is_lossless_under_burst(ros, tmp_path):
    """Frames whose labels were already computed must not be thrown away just
    because the encoder was briefly busy."""
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        h.node._annot_stop.set()                  # park the worker
        time.sleep(0.3)
        push_frames(h.node, 5)
        assert h.node._annot_q.qsize() == 5
        assert h.node._diag['annotate_drops'] == 0
    finally:
        h.close()


def test_annotate_drops_oldest_only_when_full(ros, tmp_path):
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        h.node._annot_stop.set()
        time.sleep(0.3)
        push_frames(h.node, 9)
        assert h.node._annot_q.qsize() == 5        # bound holds -> memory bound
        assert h.node._diag['annotate_drops'] >= 1
        newest = None
        while not h.node._annot_q.empty():
            newest = h.node._annot_q.get_nowait()
        assert newest[3] == 9                      # newest survived
    finally:
        h.close()


def test_handoff_never_blocks_on_a_slow_encoder(ros, tmp_path):
    """Encoding runs on its own thread, so neither the tick nor the handoff may
    inherit its latency."""
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        from overhead_tracking import annotate as ann

        def slow(*a, **k):
            time.sleep(0.2)
            return b'\xff\xd8'
        orig = ann.encode_jpeg
        ann.encode_jpeg = slow
        try:
            x, y = h.cm_of(600.0, 350.0)
            h.add_track('SB-AAAA', x, y)
            h.node._set_state(TRACKING)
            worst_push = worst_tick = 0.0
            for _ in range(8):
                h.node._source.on_tick()
                t0 = time.time()
                tick(h.node)
                worst_tick = max(worst_tick, time.time() - t0)
                h.node._last_annot_push = 0.0
                t0 = time.time()
                h.node._annotate_push_timer()
                worst_push = max(worst_push, time.time() - t0)
            assert worst_tick < 0.15, f'tick blocked {worst_tick*1000:.0f} ms'
            assert worst_push < 0.15, f'handoff blocked {worst_push*1000:.0f} ms'
        finally:
            ann.encode_jpeg = orig
    finally:
        h.close()


# ---------------------------------------------------------------------- reset
def test_reset_clears_tracks_but_keeps_the_arena(ros, tmp_path):
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        h.blobs()
        x, y = h.cm_of(600.0, 350.0)
        h.add_track('SB-AAAA', x, y)
        h.node._set_state(TRACKING)
        r = h.node._on_reset(Trigger.Request(), Trigger.Response())
        assert r.success
        assert h.node._mgr.tracks == {} and h.node._blobs == []
        assert h.node._arena is not None
        assert h.node._get_state() == ARENA_READY
        assert 'SB-AAAA' in h.node._pose_pubs     # publishers deliberately kept
    finally:
        h.close()


# --------------------------------------------------------------- nudge link
class FakeRobots:
    """Stands in for the device controllers: records every /sphero/* write,
    echoes rolls on motion_cmd, and moves the sim robot when it stops (motion
    lags the echo on the real fleet). Nothing reaches ROS."""

    def __init__(self, node, index, shift=(0.0, 60.0)):
        self.node, self.index, self.shift = node, index, shift
        self.log = []

    def install(self, names):
        n = self.node
        for name in names:
            n._ensure_robot_io(name)
            for kind, d in (('roll', n._roll_pubs), ('led', n._led_pubs),
                            ('matrix', n._matrix_pubs), ('heading', n._heading_pubs),
                            ('aim', n._aim_pubs), ('compass', n._compass_pubs)):
                d[name] = self._pub(kind, name)

    def _pub(self, kind, name):
        fake = self

        class P:
            def publish(self, msg):
                d = json.loads(msg.data)
                fake.log.append((kind, name, d))
                if kind != 'roll':
                    return
                if d['speed'] == 0:
                    src, i = fake.node._source, fake.index[name]
                    p = src.positions[i]
                    src.positions[i] = [p[0] + fake.shift[0], p[1] + fake.shift[1]]
                fake.node._on_motion_cmd(name, msg)
        return P()


def test_nudge_link_live_one_robot_at_a_time(ros, tmp_path):
    pos = [(300.0, 300.0), (600.0, 350.0), (900.0, 300.0)]
    h = Harness(tmp_path, positions=pos, link_method='nudge',
                nudge_duration_sec=0.2, nudge_settle_sec=0.3, nudge_timeout_sec=3.0,
                nudge_speed=20)                    # below the floor: raised to 50
    try:
        h.arena()
        blobs = json.loads(h.blobs().message)
        names = ['SB-AAAA', 'SB-BBBB', 'SB-CCCC']
        fake = FakeRobots(h.node, {'SB-AAAA': 2, 'SB-BBBB': 0, 'SB-CCCC': 1})
        fake.install(names)
        req = type('R', (), {'callsigns': names, 'skip_compass': True})()
        resp = type('S', (), {'success': False, 'registered': [], 'failed': [],
                              'message': ''})()
        h.node._on_link_spheros(req, resp)
        assert sorted(resp.registered) == names and resp.failed == [], resp.message
        for name, i in fake.index.items():
            b = blobs[h.node._links[name]]
            assert abs(b['u'] - pos[i][0]) < 5 and abs(b['v'] - pos[i][1]) < 5
            # locked where it is now, not where detect_spheros saw it
            x, y = h.cm_of(pos[i][0], pos[i][1] + 60.0)
            assert np.hypot(*(np.subtract(h.node._mgr.tracks[name].pos_cm, (x, y)))) < 3.0
        rolls = [(n, d['speed']) for k, n, d in fake.log if k == 'roll']
        # one robot at a time: start, stop, next -- at >= 50, then 0
        assert rolls == [('SB-AAAA', 50), ('SB-AAAA', 0), ('SB-BBBB', 50),
                         ('SB-BBBB', 0), ('SB-CCCC', 50), ('SB-CCCC', 0)]
        last_roll = max(i for i, e in enumerate(fake.log) if e[0] == 'roll')
        assert all(k == 'roll' for k, _n, _d in fake.log[:last_roll + 1]), \
            'no LED writes while robots are being nudged'
    finally:
        h.close()


def test_nudge_link_fails_without_echo_and_does_not_probe(ros, tmp_path):
    h = Harness(tmp_path, positions=[(300.0, 300.0)], link_method='nudge',
                nudge_duration_sec=0.1, nudge_settle_sec=0.2, nudge_timeout_sec=1.0)
    try:
        h.arena()
        h.blobs()
        fake = FakeRobots(h.node, {'SB-AAAA': 0})
        fake.install(['SB-AAAA'])
        h.node._on_motion_cmd = lambda *a, **k: None          # device controller down
        h.node._probe_one = lambda *a: pytest.fail('nudge must not fall back to probe')
        req = type('R', (), {'callsigns': ['SB-AAAA'], 'skip_compass': True})()
        resp = type('S', (), {'success': False, 'registered': [], 'failed': [],
                              'message': ''})()
        h.node._on_link_spheros(req, resp)
        assert resp.registered == [] and resp.failed == ['SB-AAAA']
    finally:
        h.close()


def test_unknown_link_method_is_refused(ros, tmp_path):
    h = Harness(tmp_path, positions=[(300.0, 300.0)], link_method='wiggle')
    try:
        h.arena()
        h.blobs()
        req = type('R', (), {'callsigns': ['SB-AAAA'], 'skip_compass': True})()
        resp = type('S', (), {'success': False, 'registered': [], 'failed': [],
                              'message': ''})()
        h.node._on_link_spheros(req, resp)
        assert not resp.success and 'link_method' in resp.message
    finally:
        h.close()


def write_nudge_recording(tmp_path):
    """Sim frames as a raw MJPEG + frames.csv + cmds.csv, kf_record.py style:
    a solo nudge for A, then for B (each moves ~1 s after its echo), then both."""
    import cv2
    from overhead_tracking.camera import SimOverheadSource
    dt, t0 = 0.05, 1000.0
    a0, b0 = (400.0, 300.0), (800.0, 400.0)
    src = SimOverheadSource(width=1280, height=720, positions=[a0, b0], seed=1)
    out = bytearray()
    for k in range(70):
        a = (a0[0] + 20.0 * min(max(k - 20, 0), 4), a0[1])       # moves frames 21-24
        b = (b0[0], b0[1] + 20.0 * min(max(k - 40, 0), 4))       # moves frames 41-44
        if k >= 56:
            a, b = (a[0] + 10.0 * (k - 55), a[1]), (b[0] + 10.0 * (k - 55), b[1])
        src.set_positions([a, b])
        bgr = cv2.cvtColor(src._render(), cv2.COLOR_GRAY2BGR)
        out += cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
    (tmp_path / 'raw.mjpeg').write_bytes(bytes(out))
    (tmp_path / 'frames.csv').write_text(
        'frame,t\n' + ''.join(f'{k},{t0 + k * dt:.4f}\n' for k in range(70)))
    cmds = [(0.75, 'SB-AAAA', 50), (0.95, 'SB-AAAA', 0),
            (1.75, 'SB-BBBB', 50), (1.95, 'SB-BBBB', 0),
            (2.75, 'SB-AAAA', 70), (2.76, 'SB-BBBB', 70)]
    (tmp_path / 'cmds.csv').write_text('t,robot,speed,heading\n' + ''.join(
        f'{t0 + t:.4f},{r},{s},0\n' for t, r, s in cmds))
    return tmp_path / 'raw.mjpeg', a0, b0


def test_nudge_link_on_a_replay_uses_the_recorded_nudges(ros, tmp_path):
    path, a0, b0 = write_nudge_recording(tmp_path)
    h = Harness(tmp_path, source='replay', replay_path=str(path),
                replay_lockstep=True, link_method='nudge',
                nudge_settle_sec=0.5, nudge_timeout_sec=5.0)
    try:
        h.arena()
        blobs = json.loads(h.blobs().message)
        h.node._probe_one = lambda *a: pytest.fail('nudge must not fall back to probe')
        req = type('R', (), {'callsigns': [], 'skip_compass': True})()
        resp = type('S', (), {'success': False, 'registered': [], 'failed': [],
                              'message': ''})()
        h.node._on_link_spheros(req, resp)          # roster from cmds.csv
        assert resp.registered == ['SB-AAAA', 'SB-BBBB'], resp.message
        for name, p0, p1 in (('SB-AAAA', a0, (a0[0] + 80, a0[1])),
                             ('SB-BBBB', b0, (b0[0], b0[1] + 80))):
            b = blobs[h.node._links[name]]
            assert abs(b['u'] - p0[0]) < 5 and abs(b['v'] - p0[1]) < 5
            x, y = h.cm_of(*p1)
            assert np.hypot(*(np.subtract(h.node._mgr.tracks[name].pos_cm, (x, y)))) < 3.0
        # offline: nothing towards /sphero/* was even created
        n = h.node
        assert not (n._led_pubs or n._roll_pubs or n._motion_subs)
        assert set(n._pose_pubs) == {'SB-AAAA', 'SB-BBBB'}
        # the recorded commands after linking reach each track's EKF input
        # through the motion_cmd path
        n._set_state(TRACKING)
        deadline = time.time() + 5.0
        while not n._source.stats()['replay']['eof'] and time.time() < deadline:
            tick(n)
            time.sleep(0.01)                  # let the grab thread decode ahead
        for name in ('SB-AAAA', 'SB-BBBB'):
            assert any(sp == 70 for _t, sp, _h in n._mgr.tracks[name].cmds._cmds)
    finally:
        h.close()


# ------------------------------------------------- EKF / track manager wiring
class Truth:
    """Scripted robots in FIELD cm with a field heading each, rendered by the
    sim through the arena homography. The sim has no colour, so the node's LED
    reader is replaced by one that reports the true heading of the robot under
    a blob -- or nothing when two robots share it (a merged centroid)."""

    def __init__(self, h, robots):
        self.h, self.robots = h, robots           # name -> [x, y, heading_deg]
        _c, H, Hinv, _p = h.node._arena_snapshot()
        self.H, self.Hinv = H, Hinv
        h.node._led_heading = self._led

    def px(self, name):
        from overhead_tracking import homography as hg
        x, y, _h = self.robots[name]
        return hg.field_cm_to_px(x, y, self.Hinv)

    def render(self, hidden=()):
        self.h.node._source.set_positions(
            [self.px(n) for n in self.robots if n not in hidden])

    def _led(self, _color, u, v):
        near = [n for n in self.robots
                if np.hypot(*(np.subtract(self.px(n), (u, v)))) < 8.0]
        return self.robots[near[0]][2] % 360.0 if len(near) == 1 else None


def cmd(h, name, t, speed, heading):
    h.node._on_motion_cmd(name, type('M', (), {'data': json.dumps(
        {'speed': speed, 'heading': heading})})(), t)


def test_crossing_robots_keep_their_identity(ros, tmp_path):
    """Two robots on opposite headings pass each other touching (6 cm apart,
    blobs merge/split): with LED headings and commands the tracks come out on
    the right robots and are not flagged SUSPECT."""
    h = Harness(tmp_path, positions=[], publish_rate_hz=0.0)
    try:
        h.arena()
        t0 = time.time()
        h.node._source._stamp = t0
        tr = Truth(h, {'SB-AAAA': [60.0, 63.0, 0.0], 'SB-BBBB': [140.0, 57.0, 180.0]})
        tr.render()
        h.node._source.on_tick()
        h.node._source._stamp = t0
        area = json.loads(h.blobs().message)[0]['area']
        for n in tr.robots:
            h.add_track(n, *tr.robots[n][:2], area)
        h.node._set_state(TRACKING)
        offset = {'SB-AAAA': 100.0, 'SB-BBBB': 250.0}      # aim offsets
        v = 30.0 / 0.61                                     # speed for 30 cm/s
        dt = 1 / 30.0
        for n in tr.robots:
            cmd(h, n, t0, 0, (offset[n] - tr.robots[n][2]) % 360)
        k = 0
        for _ in range(15):                                 # still: first LED fix
            k += 1
            tr.render()
            h.frame(t0 + k * dt)
        t_go = t0 + k * dt
        for n in tr.robots:
            cmd(h, n, t_go, v, (offset[n] - tr.robots[n][2]) % 360)
        for _ in range(130):                                # ~4.3 s: pass at ~2.2 s
            k += 1
            t = t0 + k * dt
            if t - t_go > 0.45:                             # start latency
                tr.robots['SB-AAAA'][0] += 30.0 * dt
                tr.robots['SB-BBBB'][0] -= 30.0 * dt
            tr.render()
            h.frame(t)
        for n in tr.robots:
            track = h.node._mgr.tracks[n]
            err = np.hypot(*(np.subtract(track.pos_cm, tr.robots[n][:2])))
            assert err < 3.0, f'{n} is {err:.1f} cm off its robot (swapped?)'
            assert track.status == HEALTHY, track.status
            assert track.heading_deg is not None
    finally:
        h.close()


def test_track_follows_a_robot_whose_aim_offset_is_not_zero(ros, tmp_path):
    """Aim offset 137 deg, a turn, and a 0.4 s hide while driving: prediction
    uses delta-commands + the LED heading, so the robot is still inside a tight
    gate where it reappears. (The old node mapped cmd -> field as -heading and
    lost exactly this case.)"""
    h = Harness(tmp_path, positions=[], publish_rate_hz=0.0, gate_max_cm=8.0)
    try:
        h.arena()
        t0 = time.time()
        h.node._source._stamp = t0
        tr = Truth(h, {'SB-AAAA': [50.0, 40.0, 30.0]})
        tr.render()
        h.node._source.on_tick()
        h.node._source._stamp = t0
        area = json.loads(h.blobs().message)[0]['area']
        h.add_track('SB-AAAA', 50.0, 40.0, area)
        h.node._set_state(TRACKING)
        offset, speed_cms, dt = 137.0, 30.0, 1 / 30.0
        robot = tr.robots['SB-AAAA']
        track = h.node._mgr.tracks['SB-AAAA']
        cmd(h, 'SB-AAAA', t0, 0, (offset - 30.0) % 360)
        k = 0
        for _ in range(15):
            k += 1
            tr.render()
            h.frame(t0 + k * dt)
        assert track.heading_deg == pytest.approx(30.0, abs=1.0)
        t_go = t0 + k * dt
        plan = [(t_go, 30.0), (t_go + 1.5, 120.0)]          # field headings
        for t_c, hd in plan:
            cmd(h, 'SB-AAAA', t_c, speed_cms / 0.61, (offset - hd) % 360)
        statuses, hide = [], (t_go + 2.4, t_go + 2.8)
        for _ in range(110):
            k += 1
            t = t0 + k * dt
            if t - t_go > 0.45:
                robot[2] = 30.0 if t < t_go + 1.5 else 120.0
                robot[0] += speed_cms * dt * math.cos(math.radians(robot[2]))
                robot[1] += speed_cms * dt * math.sin(math.radians(robot[2]))
            hidden = ('SB-AAAA',) if hide[0] <= t < hide[1] else ()
            tr.render(hidden)
            h.frame(t)
            statuses.append(track.status)
            if hidden:
                # blind prediction follows the robot in the commanded direction
                err = np.hypot(*(np.subtract(track.pos_cm, robot[:2])))
                assert err < 6.0, f'blind prediction {err:.1f} cm off'
        assert LOST not in statuses and UNRESOLVED not in statuses
        assert track.status == HEALTHY
        assert np.hypot(*(np.subtract(track.pos_cm, robot[:2]))) < 2.0
        assert abs((track.heading_deg - 120.0 + 180) % 360 - 180) < 10.0
    finally:
        h.close()


def test_pose_orientation_is_the_ekf_yaw_once_the_leds_are_seen(ros, tmp_path):
    h = Harness(tmp_path, positions=[], publish_rate_hz=0.0)
    try:
        h.arena()
        t0 = time.time()
        h.node._source._stamp = t0
        tr = Truth(h, {'SB-AAAA': [100.0, 60.0, 90.0]})
        tr.render()
        h.node._source.on_tick()
        h.node._source._stamp = t0
        h.add_track('SB-AAAA', 100.0, 60.0)
        h.node._set_state(TRACKING)
        raw = []
        h.node._pose_pubs['SB-AAAA'].publish = lambda m: raw.append(m)
        h.frame(t0 + 1 / 30.0)
        assert raw[-1].pose.orientation.w == 1.0          # no heading yet: identity
        for k in range(2, 12):
            h.frame(t0 + k / 30.0)
        q = raw[-1].pose.orientation
        assert math.degrees(2 * math.atan2(q.z, q.w)) == pytest.approx(90.0, abs=1.0)
    finally:
        h.close()


def test_heading_offset_is_published_latched_rate_limited_and_reset(ros, tmp_path):
    """offset = LED field heading + commanded Sphero heading, once the command
    has been steady for 1.5 s; published once estimated, then only on > 1 deg
    change at <= 1 Hz; reset_aim and a re-link forget it."""
    h = Harness(tmp_path, positions=[], publish_rate_hz=0.0)
    try:
        h.arena()
        t0 = time.time()
        h.node._source._stamp = t0
        tr = Truth(h, {'SB-AAAA': [100.0, 60.0, 40.0]})
        tr.render()
        h.node._source.on_tick()
        h.node._source._stamp = t0
        h.add_track('SB-AAAA', 100.0, 60.0)
        pub = h.node._offset_pubs['SB-AAAA']
        assert pub.qos_profile.durability.name == 'TRANSIENT_LOCAL'
        assert pub.qos_profile.depth == 1
        got = []
        pub.publish = lambda m: got.append((h.node._source._stamp, m.data))
        h.node._set_state(TRACKING)
        cmd(h, 'SB-AAAA', t0, 0, 100)
        k = 0
        while k < 80:                                     # 2.7 s at 30 Hz
            k += 1
            h.frame(t0 + k / 30.0)
        assert len(got) == 1 and got[0][1] == pytest.approx(140.0, abs=0.01)
        tr.robots['SB-AAAA'][2] = 50.0                    # true offset drifts to 150
        while k < 240:                                    # 6 more s
            k += 1
            h.frame(t0 + k / 30.0)
        stamps = [s for s, _v in got]
        assert 2 < len(got) <= 1 + 6                      # changed, but <= 1 Hz
        assert all(b - a >= 1.0 - 1e-6 for a, b in zip(stamps, stamps[1:]))
        assert got[-1][1] > 141.0                         # moving toward 150
        h.node._reset_aim('SB-AAAA')
        assert h.node._offset['SB-AAAA'].value is None
        n = len(got)
        h.frame(t0 + (k + 1) / 30.0)
        assert len(got) == n                              # silent until re-learnt
        h.add_track('SB-AAAA', *tr.robots['SB-AAAA'][:2])   # re-link: fresh estimator
        assert h.node._offset['SB-AAAA'].value is None
    finally:
        h.close()


def test_reconnect_resets_the_heading_offset(ros, tmp_path):
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        h.add_track('SB-AAAA', *h.cm_of(600.0, 350.0))
        est = h.node._offset['SB-AAAA']
        est._value = 10.0
        st = lambda s: type('M', (), {'data': json.dumps({'connection_state': s})})()  # noqa
        h.node._on_robot_status('SB-AAAA', st('connected'))
        assert est.value is not None                       # first report: no change
        h.node._on_robot_status('SB-AAAA', st('reconnecting'))
        h.node._on_robot_status('SB-AAAA', st('connected'))
        assert est.value is None
    finally:
        h.close()


def test_seed_link_from_a_track_csv_on_replay(ros, tmp_path):
    path, a0, b0 = write_nudge_recording(tmp_path)
    h = Harness(tmp_path, source='replay', replay_path=str(path),
                replay_lockstep=True)
    try:
        h.arena()
        blobs = json.loads(h.blobs().message)
        ax, ay = h.cm_of(*a0)
        bx, by = h.cm_of(*b0)
        seed = tmp_path / 'track.csv'
        seed.write_text('t,robot,x,y,heading_deg\n'
                        f'1,SB-BBBB,{bx + 3:.2f},{by:.2f},\n'
                        f'1,SB-AAAA,{ax:.2f},{ay - 2:.2f},\n'
                        f'2,SB-AAAA,0,0,\n')
        h.node._seed_csv = str(seed)
        h.node._probe_one = lambda *a: pytest.fail('seed must not probe')
        req = type('R', (), {'callsigns': [], 'skip_compass': True})()
        resp = type('S', (), {'success': False, 'registered': [], 'failed': [],
                              'message': ''})()
        h.node._on_link_spheros(req, resp)
        assert resp.registered == ['SB-AAAA', 'SB-BBBB'], resp.message
        assert 'by seed' in resp.message
        for name, p in (('SB-AAAA', a0), ('SB-BBBB', b0)):
            b = blobs[h.node._links[name]]
            assert abs(b['u'] - p[0]) < 5 and abs(b['v'] - p[1]) < 5
    finally:
        h.close()


# ------------------------------------------- frame-driven processing / colour
def sim_frame(h, seq, stamp, jpeg=None):
    """The sim's next gray frame as a Frame with our own seq, stamp and bytes."""
    from overhead_tracking.camera import Frame
    h.node._source.on_tick()
    return Frame(seq, stamp, h.node._source.latest_frame().gray, jpeg)


class FakeColorWorker:
    """Records the seqs submitted and asked for; colour 'ready' only for some."""
    is_hsv = True

    def __init__(self, ready):
        self.ready, self.submitted, self.asked = set(ready), [], []

    def submit(self, seq, buf):
        self.submitted.append(seq)

    def get(self, seq, timeout=0.010):
        self.asked.append(seq)
        return ('colour of', seq) if seq in self.ready else None

    def close(self):
        pass


def test_led_colour_is_paired_with_the_same_frame_by_seq(ros, tmp_path):
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        h.add_track('SB-AAAA', *h.cm_of(600.0, 350.0))
        h.node._set_state(TRACKING)
        h.node._color.close()
        h.node._color = FakeColorWorker(ready={11, 13})
        seen = []
        h.node._led_heading = lambda color, u, v: seen.append(color)
        t0 = time.time()
        for k, seq in enumerate((11, 12, 13)):
            h.node._process_frame(sim_frame(h, seq, t0 + k / 60.0,
                                            np.zeros(4, np.uint8)))
        assert h.node._color.submitted == [11, 12, 13]
        assert h.node._color.asked == [11, 12, 13]       # never another frame's
        # frame 12's colour was not ready: no LED reading for it, not frame 11's
        assert seen == [('colour of', 11), None, ('colour of', 13)]
        assert h.node._diag['color_late'] == 1
        # a frame without bytes (sim) never reaches the worker
        h.node._process_frame(sim_frame(h, 14, t0 + 3 / 60.0))
        assert h.node._color.submitted == [11, 12, 13] and seen[-1] is None
    finally:
        h.close()


@pytest.mark.parametrize('space', ['hsv', 'bgr'])
def test_led_heading_through_the_colour_worker(ros, tmp_path, space):
    """Real ColorWorker on real JPEG bytes: a red back and a green front LED
    either side of the robot give the track its first heading."""
    import cv2
    h = Harness(tmp_path, positions=[(600.0, 350.0)], led_color_space=space)
    try:
        assert h.node._color.mode == space
        h.arena()
        h.add_track('SB-AAAA', *h.cm_of(600.0, 350.0))
        h.node._set_state(TRACKING)
        t0 = time.time()
        for k in range(12):
            f = sim_frame(h, 100 + k, t0 + k / 60.0)
            bgr = cv2.cvtColor(f.gray, cv2.COLOR_GRAY2BGR)
            cv2.circle(bgr, (587, 350), 5, (0, 0, 255), -1)      # back, red
            cv2.circle(bgr, (613, 350), 5, (0, 255, 0), -1)      # front, green
            jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])[1]
            h.node._process_frame(f._replace(jpeg=jpeg.reshape(-1)))
        tr = h.node._mgr.tracks['SB-AAAA']
        assert tr.ekf.theta_known
        assert abs((tr.heading_deg + 180.0) % 360.0 - 180.0) < 5.0   # field +x
        assert h.node._diag['color_late'] == 0
    finally:
        h.close()


def test_touching_robots_do_not_read_each_others_leds(ros, tmp_path):
    """patrol_20261104_163022 at 30 Hz: two unsplit blobs touching, one blob's
    LED search window holds the neighbour's front LED, its heading jumps and
    the identities swap. A blob with another blob's centre within
    led_search_px + point_sep_px / 2 (both axes) gets no LED reading: headings and identities
    hold through approach, contact and separation."""
    import cv2
    h = Harness(tmp_path, positions=[(500.0, 350.0), (700.0, 320.0)],
                track_rate_hz=30.0)
    try:
        h.arena()
        ta = h.add_track('SB-AAAA', *h.cm_of(500.0, 350.0))
        tb = h.add_track('SB-BBBB', *h.cm_of(700.0, 320.0))
        h.node._set_state(TRACKING)
        # A heads field +x, B -x; LEDs 20 px from the centre like the real
        # robots. Touching, B's front LED (595, 322) sits inside A's window and
        # outdraws A's own: an uncrowded read would give A ~39 deg, not 0.
        path = ([(500 + 10 * k, 700 - 11 * k) for k in range(9)] +    # approach
                [(580, 615)] * 30 +                                   # touching
                [(580 - 10 * k, 615 + 11 * k) for k in range(1, 9)])  # part
        seq, t0, during = 0, time.time(), []
        for i, (au, bu) in enumerate([(500, 700)] * 12 + path):
            a, b = (float(au), 350.0), (float(bu), 320.0)
            h.node._source.set_positions([a, b])
            seq += 1
            f = sim_frame(h, seq, t0 + seq / 30.0)
            bgr = cv2.cvtColor(f.gray, cv2.COLOR_GRAY2BGR)
            cv2.circle(bgr, (int(a[0]) - 20, 350), 4, (0, 0, 255), -1)   # A back
            cv2.circle(bgr, (int(a[0]) + 20, 350), 4, (0, 255, 0), -1)   # A front
            cv2.circle(bgr, (int(b[0]) + 20, 322), 7, (0, 0, 255), -1)   # B back
            cv2.circle(bgr, (int(b[0]) - 20, 322), 7, (0, 255, 0), -1)   # B front
            jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])[1]
            h.node._process_frame(f._replace(jpeg=jpeg.reshape(-1)))
            if i >= 12:
                during.append((ta.heading_deg, tb.heading_deg))
                assert np.hypot(*np.subtract(ta.pos_cm, h.cm_of(*a))) < 3.0
                assert np.hypot(*np.subtract(tb.pos_cm, h.cm_of(*b))) < 3.0
        assert ta.ekf.theta_known and tb.ekf.theta_known
        for ha, hb in during:
            assert abs((ha + 180.0) % 360.0 - 180.0) < 10.0
            assert abs(hb % 360.0 - 180.0) < 10.0
        assert ta.status == HEALTHY and tb.status == HEALTHY
        assert h.node._diag['led_crowded'] >= 2 * 30          # both, every contact frame
        assert h.node._diag['color_late'] == 0
    finally:
        h.close()


@pytest.mark.parametrize('rate, want', [(60.0, [1, 2, 3, 4, 5, 6, 7, 8]),
                                        (30.0, [1, 3, 5, 7])])
def test_track_rate_decimates_by_frame_seq(ros, tmp_path, rate, want):
    h = Harness(tmp_path, track_rate_hz=rate)
    try:
        done = []
        h.node._process_frame = lambda f: done.append(f.seq)
        t0 = time.time()
        for seq in range(1, 9):
            h.node._on_frame(sim_frame(h, seq, t0 + seq / 60.0))
        assert done == want
    finally:
        h.close()


def test_30hz_keeps_up_when_the_camera_drops_frames(ros, tmp_path):
    """Decimation is by seq gap, so a frame after a drop is not skipped too."""
    h = Harness(tmp_path, track_rate_hz=30.0)
    try:
        done = []
        h.node._process_frame = lambda f: done.append(f.seq)
        for seq in (1, 2, 4, 5, 6, 9, 11):
            h.node._on_frame(sim_frame(h, seq, seq / 60.0))
        assert done == [1, 4, 6, 9, 11]
    finally:
        h.close()


@pytest.mark.parametrize('rate, step', [(60.0, 1), (30.0, 2), (45.0, 1), (0.0, 1)])
def test_track_rate_accepts_60_or_30_only(ros, tmp_path, rate, step):
    h = Harness(tmp_path, track_rate_hz=rate)
    try:
        assert h.node._track_rate == (30.0 if step == 2 else 60.0)
        assert h.node._frame_step == step
    finally:
        h.close()


def test_unknown_led_color_space_falls_back_to_bgr(ros, tmp_path):
    h = Harness(tmp_path, led_color_space='rgb')
    try:
        assert h.node._color.mode == 'bgr'
    finally:
        h.close()


def test_raw_jpeg_topic_carries_the_camera_bytes(ros, tmp_path):
    h = Harness(tmp_path, publish_raw_jpeg=True, track_rate_hz=30.0)
    try:
        pub = h.node._raw_pub
        assert pub.topic_name == '/overhead_tracker_node/camera/compressed'
        got = []
        pub.publish = got.append
        raw = b'\xff\xd8\xff\xe0 not re-encoded \xff\xd9'
        jpeg = np.frombuffer(raw, np.uint8)
        h.node._on_frame(sim_frame(h, 7, 1234.5, jpeg))
        h.node._on_frame(sim_frame(h, 8, 1234.5 + 1 / 60.0, jpeg))   # decimated
        assert len(got) == 2                                # every camera frame
        m = got[0]
        assert m.format == 'jpeg' and bytes(m.data) == raw
        assert m.header.frame_id == 'overhead_camera'
        assert (m.header.stamp.sec, m.header.stamp.nanosec) == (1234, 500000000)
        h.node._on_frame(sim_frame(h, 9, 1240.0))           # no bytes (sim): nothing
        assert len(got) == 2
    finally:
        h.close()


def test_raw_jpeg_topic_off_by_default(ros, tmp_path):
    h = Harness(tmp_path)
    try:
        assert h.node._raw_pub is None
        assert h.node.get_parameter('publish_raw_jpeg').value is False
    finally:
        h.close()


@pytest.mark.parametrize('rate, step', [(60.0, 1), (30.0, 2)])
def test_processing_thread_pulls_a_lockstep_replay(ros, tmp_path, rate, step):
    path, a0, _b0 = write_nudge_recording(tmp_path)
    h = Harness(tmp_path, source='replay', replay_path=str(path),
                replay_lockstep=True, track_rate_hz=rate)
    try:
        h.arena()
        h.blobs()
        n = h.node
        start = n._current_frame().seq
        time.sleep(0.2)
        assert n._current_frame().seq == start     # idle: the loop pulls nothing
        seqs = []
        orig = n._process_frame
        n._process_frame = lambda f: (seqs.append(f.seq), orig(f))
        n.start_processing()
        time.sleep(0.2)
        assert seqs == [] and n._current_frame().seq == start
        h.add_track('SB-AAAA', *h.cm_of(*a0))
        n._set_state(TRACKING)
        deadline = time.time() + 10.0
        while not n._source.stats()['replay']['eof'] and time.time() < deadline:
            time.sleep(0.02)
        time.sleep(0.1)
        assert seqs and seqs[-1] >= 69 - step + 1
        assert all(b - a == step for a, b in zip(seqs, seqs[1:])), seqs
    finally:
        h.close()


# -------------------------------------------------------------- tracking LEDs
TRACKING_LEDS = [('main', 0, 0, 0), ('front', 0, 255, 0), ('back', 255, 0, 0)]


def leds(log, name):
    return [(d['type'], d['red'], d['green'], d['blue'])
            for k, n, d in log if k == 'led' and n == name]


def test_tracking_leds_after_every_probe(ros, tmp_path):
    pos = [(300.0, 300.0), (600.0, 350.0)]
    h = Harness(tmp_path, positions=pos)
    try:
        h.arena()
        h.blobs()
        names = ['SB-AAAA', 'SB-BBBB', 'SB-CCCC']
        fake = FakeRobots(h.node, {})
        fake.install(names)
        order = {'SB-AAAA': 1, 'SB-BBBB': 0}

        def fake_probe(name, centres, used):
            fake.log.append(('probe', name, {}))
            return name in order, order.get(name, -1), None
        h.node._probe_one = fake_probe
        req = type('R', (), {'callsigns': names, 'skip_compass': True})()
        resp = type('S', (), {'success': False, 'registered': [], 'failed': [],
                              'message': ''})()
        h.node._on_link_spheros(req, resp)
        assert resp.registered == ['SB-AAAA', 'SB-BBBB'] and resp.failed == ['SB-CCCC']
        last_probe = max(i for i, e in enumerate(fake.log) if e[0] == 'probe')
        after = fake.log[last_probe + 1:]
        for name in resp.registered:
            assert leds(after, name)[-3:] == TRACKING_LEDS
            # nothing but dark before the last probe: one lit robot at a time
            assert all(c[1:] == (0, 0, 0) for c in leds(fake.log[:last_probe], name))
        assert leds(after, 'SB-CCCC')[-1] == ('main', 60, 0, 0)    # failed = red
    finally:
        h.close()


def test_tracking_leds_reapplied_when_a_linked_robot_reconnects(ros, tmp_path):
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        fake = FakeRobots(h.node, {})
        fake.install(['SB-AAAA', 'SB-BBBB'])
        h.add_track('SB-AAAA', *h.cm_of(600.0, 350.0))          # BBBB: not linked
        st = lambda s: type('M', (), {'data': json.dumps({'connection_state': s})})()  # noqa
        for name in ('SB-AAAA', 'SB-BBBB'):
            h.node._on_robot_status(name, st('connected'))      # first report
        assert fake.log == []
        for name in ('SB-AAAA', 'SB-BBBB'):
            h.node._on_robot_status(name, st('reconnecting'))
            h.node._on_robot_status(name, st('connected'))
        assert leds(fake.log, 'SB-AAAA') == TRACKING_LEDS
        assert leds(fake.log, 'SB-BBBB') == []
        h.node._on_robot_status('SB-AAAA', st('connected'))     # no change: nothing
        assert len(leds(fake.log, 'SB-AAAA')) == 3
    finally:
        h.close()

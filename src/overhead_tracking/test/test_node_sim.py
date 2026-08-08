#!/usr/bin/env python3
"""Node smoke tests.

Follows the workspace pattern: no executor, no spin. Construct the node, poke
private attributes, call the private handlers and `_tick` directly. Everything
runs against SimOverheadSource, which renders real templates, so the actual
matcher is exercised rather than a stub.
"""

import json
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.parameter import Parameter                            # noqa: E402
from std_srvs.srv import Trigger                                 # noqa: E402

from overhead_tracking.fusion import VX, VY                      # noqa: E402
from overhead_tracking.overhead_tracker_node import (            # noqa: E402
    ARENA_READY, BLOBS_READY, COASTING, HEALTHY, LINKING, LOST, OUT_OF_ARENA,
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


class Harness:
    """Node + captured pose publishes."""

    def __init__(self, tmp_path, positions=None, **extra):
        self.node = make_node(tmp_path, positions, **extra)
        self.poses = {}
        orig = self.node._publish_pose

        def spy(name, x, y, stamp):
            self.poses.setdefault(name, []).append((x, y))
            return orig(name, x, y, stamp)
        self.node._publish_pose = spy

    def arena(self):
        r = self.node._on_detect_arena(Trigger.Request(), Trigger.Response())
        assert r.success, r.message
        return r

    def blobs(self):
        return self.node._on_detect_spheros(Trigger.Request(), Trigger.Response())

    def add_track(self, name, x_cm, y_cm):
        self.node._ensure_robot_io(name)
        self.node._lock_tracker(name, x_cm, y_cm)
        return self.node._tracks[name]

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
        assert 'p50' in r.message and 'min_bright' in r.message
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
            idx = order[name]
            return (idx not in used), idx
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
        assert len(h.node._tracks) == 3
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
        h.node._tick()

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
            h.node._tick()
        gx, gy = h.node._tracks['SB-AAAA'].kf.pos_cm
        assert abs(gx - tx) < 2.0 and abs(gy - ty) < 2.0
        assert h.node._tracks['SB-AAAA'].status == HEALTHY
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
            h.node._tick()
        elapsed = max(time.time() - t0, 1e-6)
        n = len(h.poses.get('SB-AAAA', []))
        assert n <= int(elapsed * 10) + 2      # never faster than the contract
    finally:
        h.close()


def test_stale_frame_still_publishes_predict_only(ros, tmp_path):
    """A missing/stale frame must not create a gap in the 10 Hz stream."""
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        x, y = h.cm_of(600.0, 350.0)
        t = h.add_track('SB-AAAA', x, y)
        h.node._set_state(TRACKING)
        h.node._source.on_tick()
        h.node._tick()
        t.kf.x[VX], t.kf.x[VY] = 30.0, 0.0
        before = t.kf.pos_cm[0]
        h.node._last_pub = 0.0
        h.node._tick()                                # same stamp -> not fresh
        assert len(h.poses['SB-AAAA']) >= 1
        assert t.kf.pos_cm[0] > before                # dead-reckoned forward
    finally:
        h.close()


def test_miss_ladder_then_reacquire(ros, tmp_path):
    h = Harness(tmp_path, positions=[(600.0, 350.0)], max_consecutive_misses=3,
                lost_timeout_sec=1000.0)
    try:
        h.arena()
        x, y = h.cm_of(600.0, 350.0)
        t = h.add_track('SB-AAAA', x, y)
        h.node._set_state(TRACKING)
        for _ in range(3):
            h.node._source.on_tick()
            h.node._tick()
        assert t.status == HEALTHY

        h.node._source.set_positions([])              # robot disappears
        for _ in range(2):
            h.node._source.on_tick()
            h.node._tick()
        assert t.status == COASTING
        for _ in range(5):
            h.node._source.on_tick()
            h.node._tick()
        assert t.status in (LOST, UNRESOLVED)

        h.node._source.set_positions([(600.0, 350.0)])   # and comes back
        for _ in range(5):
            h.node._source.on_tick()
            h.node._tick()
        assert t.status == HEALTHY
    finally:
        h.close()


def test_out_of_arena_measurement_is_rejected_but_publishing_continues(
        ros, tmp_path):
    h = Harness(tmp_path, positions=[(600.0, 350.0)])
    try:
        h.arena()
        x, y = h.cm_of(600.0, 350.0)
        t = h.add_track('SB-AAAA', x, y)
        h.node._set_state(TRACKING)
        h.node._source.on_tick()
        h.node._tick()

        # move it outside the arena polygon; the measurement must be refused
        h.node._source.set_positions([(1262.0, 705.0)])
        h.node._arena_margin_cm = 0.0
        for _ in range(4):
            h.node._source.on_tick()
            h.node._tick()
        assert t.status in (OUT_OF_ARENA, COASTING, LOST, UNRESOLVED)
        h.node._last_pub = 0.0
        h.node._tick()
        assert h.poses['SB-AAAA']                     # stream never gapped
    finally:
        h.close()


def test_a_distant_jump_never_even_reaches_the_gate(ros, tmp_path):
    """The ROI is the FIRST swap defence: a blob that far away is not inside the
    forward-predicted window, so it is never claimed and the track simply
    coasts. The innovation gate below is the second line, for a wrong blob that
    IS inside the window."""
    h = Harness(tmp_path, positions=[(300.0, 300.0)])
    try:
        h.arena()
        x, y = h.cm_of(300.0, 300.0)
        t = h.add_track('SB-AAAA', x, y)
        h.node._set_state(TRACKING)
        h.node._source.on_tick()
        h.node._tick()
        before = t.kf.pos_cm

        h.node._source.set_positions([(900.0, 400.0)])
        h.node._source.on_tick()
        h.node._tick()
        assert abs(t.kf.pos_cm[0] - before[0]) < 20.0
        assert t.status in (COASTING, LOST)
    finally:
        h.close()


def test_innovation_gate_rejects_an_in_roi_measurement_that_disagrees(
        ros, tmp_path):
    """A blob inside the ROI but further from the prediction than the gate must
    be refused and counted, so a persistent disagreement becomes visible."""
    h = Harness(tmp_path, positions=[(600.0, 350.0)], innovation_gate_cm=0.5)
    try:
        h.arena()
        x, y = h.cm_of(600.0, 350.0)
        t = h.add_track('SB-AAAA', x, y)
        h.node._set_state(TRACKING)
        h.node._source.on_tick()
        h.node._tick()
        before = t.kf.pos_cm

        # inside the 51 px ROI, but ~9 cm away -- well past the 0.5 cm gate
        h.node._source.set_positions([(640.0, 350.0)])
        h.node._source.on_tick()
        h.node._tick()
        assert t.disagreements >= 1
        assert abs(t.kf.pos_cm[0] - before[0]) < 1.0     # measurement refused
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
        start = t.kf.pos_cm
        for _ in range(6):
            h.node._source.on_tick()
            h.node._tick()
        # only prediction moved it -- with zero velocity that means barely at all
        assert abs(t.kf.pos_cm[0] - start[0]) < 1.0
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
                h.node._tick()

        final_a = ta.kf.pos_cm[0]
        final_b = tb.kf.pos_cm[0]
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
                  len(h.node._sensor_subs))
        for _ in range(10):
            h.node._source.on_tick()
            h.node._tick()
        after = (len(h.node._pose_pubs), len(h.node._led_pubs),
                 len(h.node._sensor_subs))
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
                h.node._tick()
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
        assert h.node._tracks == {} and h.node._blobs == []
        assert h.node._arena is not None
        assert h.node._get_state() == ARENA_READY
        assert 'SB-AAAA' in h.node._pose_pubs     # publishers deliberately kept
    finally:
        h.close()

#!/usr/bin/env python3
"""ReplaySource: a recorded raw MJPEG must look like the live camera to the node."""

import time

import cv2
import numpy as np

from overhead_tracking import camera as C

N = 6


def level(i):
    return 20 * i + 10                      # frame i is a flat image of this value


def which(gray):
    return int(round((float(gray.mean()) - 10) / 20))


def make_mjpeg(tmp_path, csv_times=None, no_eoi=None):
    """N flat frames plus a truncated tail, as v4l2-ctl --stream-to leaves it.

    no_eoi: index of a frame written without its FF D9, as the camera sometimes
    emits mid-stream."""
    jpgs = [cv2.imencode('.jpg', np.full((48, 64, 3), level(i), np.uint8))[1].tobytes()
            for i in range(N)]
    if no_eoi is not None:
        jpgs[no_eoi] = jpgs[no_eoi][:-2]
    tail = cv2.imencode('.jpg', np.zeros((48, 64, 3), np.uint8))[1].tobytes()
    path = tmp_path / 'raw.mjpeg'
    path.write_bytes(b''.join(jpgs) + tail[:len(tail) // 2])
    if csv_times is not None:
        rows = ''.join(f'{i},{t:.4f}\n' for i, t in enumerate(csv_times))
        (tmp_path / 'frames.csv').write_text('frame,t\n' + rows)
    return path


def pull_all(src, n, timeout=2.0):
    """Pull until n distinct stamps were seen; returns [(frame_idx, stamp)]."""
    out, last, t0 = [], None, time.time()
    while len(out) < n and time.time() - t0 < timeout:
        g, s = src.latest_gray()
        if g is not None and s != last:
            out.append((which(g), s))
            last = s
        time.sleep(0.001)
    return out


def test_index_drops_truncated_tail(tmp_path):
    data = make_mjpeg(tmp_path).read_bytes()
    spans = C.index_mjpeg(data)
    assert [k for k, _s, _e in spans] == list(range(N))
    for _k, s, e in spans:
        assert data[s:s + 3] == b'\xff\xd8\xff' and data[e - 2:e] == b'\xff\xd9'


def test_lockstep_never_drops_and_stamps_follow_fps(tmp_path):
    src = C.build_source('replay', replay_path=str(make_mjpeg(tmp_path)), fps=50.0,
                         replay_lockstep=True)
    t_start = time.time()
    src.start()
    try:
        got = pull_all(src, N)
        assert [i for i, _ in got] == list(range(N))
        stamps = np.array([s for _, s in got])
        assert np.allclose(np.diff(stamps), 0.02)
        assert abs(stamps[0] - t_start) < 0.5          # re-based to the wall clock
        time.sleep(0.1)
        st = src.stats()
        assert st['frames'] == N and st['replay']['eof']
        g, s = src.latest_gray()                       # holds the last frame
        assert which(g) == N - 1 and s == stamps[-1]
        f = src.current_frame()                        # raw bytes kept per frame
        assert f.seq == N - 1 and f.gray is g
        assert cv2.imdecode(f.jpeg, cv2.IMREAD_COLOR).shape == (48, 64, 3)
    finally:
        src.close()


def test_lockstep_waits_for_the_consumer(tmp_path):
    src = C.build_source('replay', replay_path=str(make_mjpeg(tmp_path)),
                         replay_lockstep=True)
    src.start()
    try:
        _i0, s0 = pull_all(src, 1)[0]
        time.sleep(0.2)                                # nobody pulls: nothing moves
        assert src.stats()['frames'] == 1
        g, s = src.latest_gray()
        assert which(g) == 1 and s > s0
    finally:
        src.close()


def test_stamps_from_frames_csv(tmp_path):
    t = 1791234850.0 + np.array([0.0, 0.016, 0.034, 0.050, 0.067, 0.083])
    src = C.build_source('replay', replay_path=str(make_mjpeg(tmp_path, t)),
                         replay_lockstep=True)
    src.start()
    try:
        stamps = np.array([s for _, s in pull_all(src, N)])
        assert np.allclose(np.diff(stamps), np.diff(t), atol=1e-6)
        assert abs(stamps[0] - time.time()) < 1.0      # not the 2026 file epoch
        assert src.stats()['driver_fps'] == round(1.0 / np.median(np.diff(t)), 1)
    finally:
        src.close()


def test_frame_without_end_marker_keeps_csv_rows_aligned(tmp_path):
    t = 1791234850.0 + np.arange(N) * 0.016
    src = C.build_source('replay', replay_path=str(make_mjpeg(tmp_path, t, no_eoi=2)),
                         replay_lockstep=True)
    src.start()
    try:
        got = pull_all(src, N - 1)
        assert [i for i, _ in got] == [0, 1, 3, 4, 5]
        assert np.allclose(np.diff([s for _, s in got]),
                           [0.016, 0.032, 0.016, 0.016], atol=1e-6)
    finally:
        src.close()


def test_loop_wraps_with_monotonic_stamps(tmp_path):
    src = C.build_source('replay', replay_path=str(make_mjpeg(tmp_path)), fps=50.0,
                         replay_loop=True, replay_lockstep=True)
    src.start()
    try:
        got = pull_all(src, 2 * N + 1)
        assert [i for i, _ in got] == list(range(N)) * 2 + [0]
        assert np.allclose(np.diff([s for _, s in got]), 0.02)
        assert src.stats()['replay']['loops'] == 2
        assert not src.stats()['replay']['eof']
    finally:
        src.close()


def test_realtime_paces_to_the_wall_clock_and_stops_at_eof(tmp_path):
    src = C.build_source('replay', replay_path=str(make_mjpeg(tmp_path)), fps=20.0,
                         replay_rate=2.0)                 # 25 ms per frame
    t_start = time.time()
    src.start()
    try:
        time.sleep(0.06)
        g, s = src.latest_gray()
        assert g is not None and abs(s - time.time()) < 0.05   # looks live
        time.sleep(N * 0.025 + 0.2)
        st = src.stats()
        assert st['replay']['eof'] and not src._thread.is_alive()
        g, s = src.latest_gray()
        assert which(g) == N - 1
        assert abs(s - (t_start + (N - 1) * 0.025)) < 0.05
    finally:
        src.close()


def test_realtime_loop_keeps_playing(tmp_path):
    src = C.build_source('replay', replay_path=str(make_mjpeg(tmp_path)), fps=100.0,
                         replay_loop=True)
    src.start()
    try:
        time.sleep(N * 0.01 * 3 + 0.05)
        st = src.stats()
        assert st['replay']['loops'] >= 2 and not st['replay']['eof']
        assert abs(st['stamp'] - time.time()) < 0.05
    finally:
        src.close()


def test_replay_is_not_live():
    """The node skips its stale-camera reopen for a source that is not live."""
    assert C.ReplaySource.live is False
    assert getattr(C.V4L2Source, 'live', True) is True


# ------------------------------------------------------------- cmds.csv feed
def write_cmds(tmp_path, rows):
    (tmp_path / 'cmds.csv').write_text(
        't,robot,speed,heading\n' + ''.join(f'{t:.4f},{r},{s},{h}\n' for t, r, s, h in rows))


def test_cmds_feed_lockstep_aligned_and_in_order(tmp_path):
    t0 = 1000.0
    times = [t0 + 0.1 * i for i in range(N)]
    path = make_mjpeg(tmp_path, csv_times=times)
    # out of order in the file on purpose; one before frame 0, one after the last
    write_cmds(tmp_path, [(t0 + 0.25, 'B', 60, 90), (t0 + 0.05, 'A', 50, 0),
                          (t0 - 1.0, 'A', 0, 0), (t0 + 0.30, 'A', 0, 0),
                          (t0 + 9.0, 'B', 0, 0)])
    src = C.build_source('replay', replay_path=str(path), replay_lockstep=True)
    got = []
    src.cmd_handler = lambda r, s, h, st: got.append((r, s, h, st, src._index))
    src.start()
    try:
        assert src.robots == ['A', 'B']
        frames = pull_all(src, N)
        assert [i for i, _ in frames] == list(range(N))
        stamp_of = dict(frames)
        offset = stamp_of[0] - t0                       # the frames' re-base
        assert [(r, s, h) for r, s, h, _st, _i in got] == \
            [('A', 0, 0), ('A', 50, 0), ('B', 60, 90), ('A', 0, 0)]
        for (_r, _s, _h, st, i), t_file in zip(got, (t0 - 1.0, t0 + 0.05, t0 + 0.25, t0 + 0.30)):
            assert abs(st - (t_file + offset)) < 1e-3   # same offset as frames
            assert st <= stamp_of[i]                    # delivered once passed
            assert i == 0 or st > stamp_of[i - 1]       # ... and not before
    finally:
        src.close()


def test_cmds_feed_realtime_and_loop_replays_commands(tmp_path):
    times = [500.0 + 0.02 * i for i in range(N)]
    path = make_mjpeg(tmp_path, csv_times=times)
    write_cmds(tmp_path, [(500.03, 'A', 50, 0), (500.07, 'A', 0, 0)])
    src = C.build_source('replay', replay_path=str(path), replay_loop=True)
    got = []
    src.cmd_handler = lambda r, s, h, st: got.append((s, st))
    src.start()
    try:
        time.sleep(0.4)                                 # several 0.12 s passes
    finally:
        src.close()
    assert len(got) >= 4 and [s for s, _ in got[:4]] == [50, 0, 50, 0]
    assert all(b[1] > a[1] for a, b in zip(got, got[1:]))   # time order


def test_cmds_ignored_without_frames_csv(tmp_path):
    path = make_mjpeg(tmp_path)
    write_cmds(tmp_path, [(1.0, 'A', 50, 0)])
    src = C.build_source('replay', replay_path=str(path), replay_lockstep=True)
    got = []
    src.cmd_handler = lambda *a: got.append(a)
    src.start()
    try:
        pull_all(src, N)
        assert got == [] and src.robots == []
    finally:
        src.close()


def test_frame_seq_counts_across_loops_and_wait_frame_pulls(tmp_path):
    src = C.build_source('replay', replay_path=str(make_mjpeg(tmp_path)), fps=50.0,
                         replay_loop=True, replay_lockstep=True)
    src.start()
    try:
        seqs, last = [], None
        for _ in range(N + 2):
            f = src.wait_frame(last, 1.0)
            assert f is not None and which(f.gray) == f.seq % N
            seqs.append(f.seq)
            last = f.seq
        assert seqs == list(range(N + 2))            # pass * N + index, no gaps
        assert src.current_frame().seq == N + 1       # current_frame() never pulls
        time.sleep(0.05)
        assert src.current_frame().seq == N + 1
    finally:
        src.close()


def test_wait_frame_times_out_at_eof(tmp_path):
    src = C.build_source('replay', replay_path=str(make_mjpeg(tmp_path)), fps=50.0,
                         replay_lockstep=True)
    src.start()
    try:
        last = None
        for _ in range(N):
            last = src.wait_frame(last, 1.0).seq
        t0 = time.time()
        assert src.wait_frame(last, 0.1) is None
        assert time.time() - t0 < 0.5
    finally:
        src.close()

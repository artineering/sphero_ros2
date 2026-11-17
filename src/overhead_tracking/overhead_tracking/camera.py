#!/usr/bin/env python3
"""Frame sources: the real global-shutter camera, a recorded-file replay, and a
simulator.

Duck-typed pull interface, so the node is identical on hardware and in tests:
    start() / on_tick() / close()
    latest_frame()  -> Frame | None   (lockstep replay: advances, see ReplaySource)
    current_frame() -> Frame | None   (never advances)
    wait_frame(after_seq, timeout) -> the next Frame whose seq != after_seq, or None
    latest_gray()   -> (gray | None, stamp), latest_frame() for older callers

A Frame is (seq, stamp, gray, jpeg): a sequence number that is unique per frame
and paired with that frame's capture stamp, the grayscale decode, and the raw
MJPEG bytes it was decoded from (None for the simulator), copied once. Colour is
decoded from those bytes by whoever needs it (color_decode.ColorWorker for the
LEDs), never by the source: a full-colour decode of a 1920x1200 frame costs
~22 ms, more than the 16.6 ms frame budget.

IMMUTABILITY INVARIANT (load-bearing)
-------------------------------------
Every published frame is a FRESH array that is never written to again. Consumers
may therefore hold a reference, hand a numpy *view* to a worker thread, and queue
it for annotation, all without copying. Violating this -- reusing one buffer --
would silently corrupt in-flight frame users (the floor-map buffer keeps views
without copying). Corollary: anything holding a
frame reference keeps that memory alive, which is why the annotate queue is
bounded.

UVC CONTROL QUIRK
-----------------
The camera discards manual exposure when a stream starts. Controls must be
applied ~1.5 s AFTER streaming begins, and re-applied on every reopen -- not just
the first. `apply_controls` reads every value back, because the driver silently
clamps exposure to the frame period and there is no error to catch: the only way
to learn the request was ignored is to look.
"""

import csv
import mmap
import os
import subprocess
import threading
import time
from collections import namedtuple

import cv2
import numpy as np

#: One camera frame. jpeg: raw MJPEG bytes as a uint8 array, or None (sim).
Frame = namedtuple('Frame', 'seq stamp gray jpeg')


def decode_gray(jpeg):
    """Grayscale decode of raw MJPEG bytes (~6 ms at 1920x1200; libjpeg skips
    the chroma). A fresh array -- see the immutability invariant."""
    return cv2.imdecode(jpeg, cv2.IMREAD_GRAYSCALE)


class V4L2Source:
    """USB3 global-shutter camera via OpenCV/V4L2, MJPG, background grab thread."""

    def __init__(self, device='/dev/video0', width=1920, height=1200, fps=120.0,
                 fourcc='MJPG', controls=None, set_fps=True, logger=None):
        self.device = device
        self.width = int(width)
        self.height = int(height)
        self.expected_fps = float(fps)
        self.fourcc = fourcc
        self.controls = dict(controls or {})
        self.set_fps = bool(set_fps)
        self._log = logger

        self._cap = None
        self._frame = None
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._stop = threading.Event()
        self._thread = None
        self._count = 0
        self._fps_est = 0.0
        self._fps_t0 = 0.0
        self._fps_n0 = 0
        self._decode_ms = 0.0
        self._open_failures = 0
        self.driver_fps = 0.0

    # ------------------------------------------------------------------ setup
    def _info(self, msg):
        if self._log:
            self._log.info(msg)

    def _warn(self, msg):
        if self._log:
            self._log.warning(msg)

    def _open(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f'cannot open {self.device}')
        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        # CAP_PROP_FPS MUST be set. Leaving it alone does not mean "let the camera
        # pick its maximum" -- OpenCV's V4L2 backend simply defaults to 30 fps.
        # Measured: without this the node ran at 30 fps, which quadruples the
        # exposure clamp (33 ms instead of 8.3 ms) and silently invalidates the
        # brightness calibration. Raw `v4l2-ctl` behaves differently, which is
        # where the wrong assumption came from.
        if self.set_fps:
            cap.set(cv2.CAP_PROP_FPS, self.expected_fps)
        # hand back the raw MJPEG bytes: the frame is decoded here to gray only,
        # and to half-res colour on the node's colour worker
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        self.driver_fps = self._read_driver_fps() or float(cap.get(cv2.CAP_PROP_FPS))
        if self.set_fps and self.driver_fps and \
                abs(self.driver_fps - self.expected_fps) > 1.0:
            self._warn(f'requested {self.expected_fps:.0f} fps, driver is at '
                       f'{self.driver_fps:.0f} -- exposure clamp is now '
                       f'{1000.0/max(self.driver_fps,1):.1f} ms')
        return cap

    def _read_driver_fps(self):
        """The camera's own frame interval, from the driver.

        This is the number that sets the exposure clamp. It is NOT the same as
        how fast we manage to consume frames -- MJPEG decode plus the gray
        conversion limits the grab thread well below the sensor rate, and
        conflating the two produces a bogus 'framerate drift' warning.
        """
        try:
            out = subprocess.run(['v4l2-ctl', '-d', self.device, '--get-parm'],
                                 capture_output=True, text=True).stdout
            for line in out.splitlines():
                if 'Frames per second' in line:
                    return float(line.split(':')[1].strip().split()[0])
        except Exception:                                  # noqa: BLE001
            pass
        return 0.0

    def start(self):
        self._cap = self._open()
        self._stop.clear()
        self._fps_t0 = time.time()
        self._fps_n0 = 0
        self._thread = threading.Thread(target=self._grab_loop, daemon=True,
                                        name='v4l2grab')
        self._thread.start()
        self._info(f'V4L2Source started: {self.device} {self.width}x{self.height} '
                   f'{self.fourcc}')

    def _grab_loop(self):
        while not self._stop.is_set():
            cap = self._cap
            if cap is None:
                time.sleep(0.05)
                continue
            ok, buf = cap.read()
            if not ok or buf is None:
                time.sleep(0.005)
                continue
            now = time.time()                # capture stamp, before the decode
            t0 = time.perf_counter()
            if buf.ndim == 3:                # driver ignored CONVERT_RGB=0
                gray, jpeg = cv2.cvtColor(buf, cv2.COLOR_BGR2GRAY), None
            else:
                # copied ONCE: the capture reuses its buffer for the next read.
                # Fresh arrays every frame -- see the immutability invariant.
                jpeg = np.array(buf.reshape(-1), copy=True)
                gray = decode_gray(jpeg)
                if gray is None:             # corrupt frame: skip it
                    continue
            dec = (time.perf_counter() - t0) * 1000.0
            with self._cond:
                self._count += 1
                self._frame = Frame(self._count, now, gray, jpeg)
                self._decode_ms += 0.05 * (dec - self._decode_ms)
                dt = now - self._fps_t0
                if dt >= 1.0:
                    self._fps_est = (self._count - self._fps_n0) / dt
                    self._fps_t0 = now
                    self._fps_n0 = self._count
                self._cond.notify_all()

    # ------------------------------------------------------------- controls
    def apply_controls(self):
        """Set the v4l2 controls, then READ THEM BACK.

        Returns {name: value_as_reported}. The readback is the point: the driver
        clamps exposure to the frame period without reporting an error, so a
        request that was silently ignored looks identical to one that took.
        """
        if not self.controls:
            return {}
        args = [f'--set-ctrl={k}={v}' for k, v in self.controls.items()]
        subprocess.run(['v4l2-ctl', '-d', self.device] + args,
                       capture_output=True)
        names = ','.join(self.controls.keys())
        out = subprocess.run(['v4l2-ctl', '-d', self.device,
                              f'--get-ctrl={names}'],
                             capture_output=True, text=True).stdout
        readback = {}
        for line in out.strip().splitlines():
            if ':' in line:
                k, v = line.split(':', 1)
                readback[k.strip()] = v.strip()
        for k, want in self.controls.items():
            got = readback.get(k)
            if got is not None and str(want) not in str(got):
                self._warn(f'control {k}: requested {want}, driver reports {got}')
        self._info(f'camera controls applied: {readback}')
        return readback

    def reopen(self):
        """Close and reopen the capture. Caller must re-apply controls after."""
        self._warn(f'reopening {self.device}')
        try:
            if self._cap is not None:
                self._cap.release()
        except Exception:                                  # noqa: BLE001
            pass
        try:
            self._cap = self._open()
            self._open_failures = 0
            return True
        except Exception as e:                             # noqa: BLE001
            self._open_failures += 1
            self._warn(f'reopen failed ({self._open_failures}): {e}')
            self._cap = None
            return False

    # --------------------------------------------------------------- pulling
    def on_tick(self):
        """No-op: the grab thread fills the cache asynchronously."""

    def latest_frame(self):
        with self._lock:
            return self._frame

    current_frame = latest_frame

    def wait_frame(self, after_seq, timeout):
        with self._cond:
            self._cond.wait_for(lambda: self._frame is not None and
                                self._frame.seq != after_seq, timeout)
            f = self._frame
        return f if f is not None and f.seq != after_seq else None

    def latest_gray(self):
        f = self.latest_frame()
        return (None, 0.0) if f is None else (f.gray, f.stamp)

    def stats(self):
        with self._lock:
            return {'frames': self._count,
                    'consume_fps': round(self._fps_est, 1),
                    'driver_fps': round(self.driver_fps, 1),
                    'decode_ms': round(self._decode_ms, 2),
                    'stamp': self._frame.stamp if self._frame else 0.0,
                    'device': self.device}

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        try:
            if self._cap is not None:
                self._cap.release()
        except Exception:                                  # noqa: BLE001
            pass
        self._cap = None


def index_mjpeg(buf):
    """(ordinal, start, end) of every complete JPEG in a raw concatenated stream.

    A frame starts at FF D8 FF and ends at the last FF D9 before the next start;
    a span with no FF D9 (the truncated tail of a recording) is dropped.
    Matches scripts/mjpeg2avi.py. `ordinal` counts every start, dropped or not,
    so it stays the row number in the recorder's frames.csv: the camera also
    emits the odd frame with no FF D9 MID-stream (14 of 2471 in
    kf_20261005_171405), and counting only kept frames would shift every later
    stamp.
    """
    spans = []
    k = 0
    s = buf.find(b'\xff\xd8\xff')
    while s >= 0:
        nxt = buf.find(b'\xff\xd8\xff', s + 3)
        end = len(buf) if nxt < 0 else nxt
        j = buf.rfind(b'\xff\xd9', s, end)
        if j > 0:
            spans.append((k, s, j + 2))
        k += 1
        s = nxt
    return spans


class ReplaySource:
    """Recorded raw MJPEG (v4l2-ctl --stream-to / kf_record.py) as a camera.

    Same pull interface, decode and stats as V4L2Source. Per-frame times come
    from a `frames.csv` (frame,t) next to the file, else 1/fps; they are
    re-based to the wall clock at start so stamp ages look live. Frame seq is
    pass * n_frames + index, so it keeps counting across loops and a frame
    skipped in realtime leaves a gap, as a dropped camera frame would.

    realtime (default): frames are released on the recorded schedule divided by
        `rate`; stamps are that schedule. If decode falls behind, it skips to the
        frame that is due, as a live camera drops frames.
    lockstep: never drops. One frame is decoded ahead; it replaces the current
        one only when latest_frame() / wait_frame() is called after the current
        one was already handed out, so every frame is seen exactly once by some
        caller (current_frame() never advances). Stamps
        keep the recorded spacing exactly (rate is ignored), so a consumer slower
        than realtime sees stamps fall behind the wall clock.
    End of file: wrap if `loop`, else hold the last frame and report eof.

    Commands: a `cmds.csv` (t,robot,speed,heading -- the recorded
    /sphero/<ns>/motion_cmd echoes) next to the file is re-based with the SAME
    offset as frames.csv, and each command goes to `cmd_handler(robot, speed,
    heading, stamp)` when the first frame at or after it becomes current, in
    time order, once per pass. Realtime: from the grab thread. Lockstep: from
    the latest_frame() call that promotes the frame, before it returns, so a
    consumer has the commands up to a frame before it sees the frame.
    `robots` lists the callsigns in cmds.csv, in order of first appearance.
    """

    #: Tells the node a stale stamp is not a dead camera to reopen.
    live = False

    def __init__(self, path, fps=60.0, rate=1.0, loop=False, lockstep=False,
                 logger=None):
        self.path = str(path)
        self.expected_fps = float(fps)
        self.rate = float(rate) if rate and rate > 0 else 1.0
        self.loop = bool(loop)
        self.lockstep = bool(lockstep)
        self._log = logger

        self._file = None
        self._mm = None
        self._spans = []
        self._rel = None                     # seconds from frame 0, file time
        self._frame = None
        self._pending = None                 # lockstep: the next Frame
        self._handed = True                  # lockstep: current already pulled
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._stop = threading.Event()
        self._thread = None
        self._count = 0
        self._skipped = 0
        self._loops = 0
        self._index = 0
        self._eof = False
        self._fps_est = 0.0
        self._fps_t0 = 0.0
        self._fps_n0 = 0
        self._decode_ms = 0.0
        self.driver_fps = 0.0
        self.cmd_handler = None              # set before start()
        self.robots = []
        self._t0 = None                      # frames.csv epoch of frame 0
        self._cmds = []                      # (rel s, robot, speed, heading)
        self._cmd_i = 0
        self._last_i = -1
        self._deliver_lock = threading.Lock()

    def _info(self, msg):
        if self._log:
            self._log.info(msg)

    def _warn(self, msg):
        if self._log:
            self._log.warning(msg)

    def _load_times(self, n):
        """Per-frame file times relative to frame 0, and the nominal period."""
        csv_path = os.path.join(os.path.dirname(os.path.abspath(self.path)),
                                'frames.csv')
        if os.path.isfile(csv_path):
            with open(csv_path) as f:
                t = np.array([float(r['t']) for r in csv.DictReader(f)])
            keep = [sp for sp in self._spans if sp[0] < len(t)]
            if len(keep) < n:
                self._warn(f'{csv_path} has {len(t)} rows; replaying only the '
                           f'{len(keep)} stamped frames of {n}')
                self._spans = keep
            self._t0 = float(t[0])
            rel = t[[sp[0] for sp in keep]] - t[0]
            dt = float(np.median(np.diff(t))) if len(t) > 1 else 1.0 / self.expected_fps
            self._info(f'replay stamps from {csv_path}')
            return rel, dt
        dt = 1.0 / self.expected_fps
        return np.array([sp[0] for sp in self._spans]) * dt, dt

    def _load_cmds(self):
        """cmds.csv rows as (seconds from frame 0, robot, speed, heading)."""
        path = os.path.join(os.path.dirname(os.path.abspath(self.path)), 'cmds.csv')
        if not os.path.isfile(path):
            return
        if self._t0 is None:
            self._warn(f'{path} ignored: no frames.csv to align it with')
            return
        with open(path) as f:
            rows = [(float(r['t']) - self._t0, r['robot'], int(float(r['speed'])),
                     int(float(r['heading']))) for r in csv.DictReader(f)]
        self._cmds = sorted(rows, key=lambda r: r[0])
        self.robots = list(dict.fromkeys(r[1] for r in self._cmds))
        self._info(f'replay commands from {path}: {len(self._cmds)} for {self.robots}')

    def start(self):
        self._file = open(self.path, 'rb')
        self._mm = mmap.mmap(self._file.fileno(), 0, access=mmap.ACCESS_READ)
        self._spans = index_mjpeg(self._mm)
        if not self._spans:
            raise RuntimeError(f'no complete JPEG frames in {self.path}')
        self._rel, self._dt = self._load_times(len(self._spans))
        self._load_cmds()
        self.driver_fps = 1.0 / self._dt
        self._stop.clear()
        self._fps_t0 = time.time()
        self._fps_n0 = 0
        self._thread = threading.Thread(target=self._grab_loop, daemon=True,
                                        name='replaygrab')
        self._thread.start()
        mode = 'lockstep' if self.lockstep else f'realtime x{self.rate:g}'
        self._info(f'ReplaySource started: {self.path} {len(self._spans)} frames '
                   f'({self._spans[-1][0] + 1 - len(self._spans)} mid-stream without '
                   f'an end marker dropped) @ {self.driver_fps:.1f} fps, {mode}, '
                   f'loop={self.loop}')

    def _decode(self, i):
        """(gray, jpeg) of frame i. The bytes are copied out of the mmap once,
        so a Frame never pins the file; both are fresh arrays -- see the
        immutability invariant."""
        _k, s, e = self._spans[i]
        t0 = time.perf_counter()
        jpeg = np.array(np.frombuffer(self._mm, np.uint8, e - s, s))
        gray = decode_gray(jpeg)
        self._decode_ms += 0.05 * ((time.perf_counter() - t0) * 1000.0 - self._decode_ms)
        return gray, jpeg

    def _grab_loop(self):
        n = len(self._spans)
        scale = 1.0 if self.lockstep else self.rate
        anchor = time.time()                 # wall time of frame 0, this pass
        i = 0
        while not self._stop.is_set():
            if i >= n:
                if not self.loop:
                    with self._lock:
                        self._eof = True
                    self._info(f'replay finished: {n} frames from {self.path}; '
                               'holding the last frame')
                    return
                anchor += (self._rel[-1] + self._dt) / scale
                i = 0
                with self._lock:
                    self._loops += 1
            if not self.lockstep:
                now = time.time()
                while i + 1 < n and anchor + self._rel[i + 1] / scale <= now:
                    i += 1
                    self._skipped += 1
                due = anchor + self._rel[i] / scale
                if due > now and self._stop.wait(due - now):
                    return
            gray, jpeg = self._decode(i)
            frame = Frame(self._loops * n + i, anchor + self._rel[i] / scale,
                          gray, jpeg)
            if self.lockstep:
                with self._cond:
                    self._pending = frame
                    self._cond.notify_all()
                    while self._pending is not None and not self._stop.is_set():
                        self._cond.wait(0.1)
            else:
                with self._deliver_lock:
                    with self._cond:
                        due = self._publish(frame)
                        self._cond.notify_all()
                    self._deliver(due)
            i += 1

    def _publish(self, frame):
        """Make `frame` current; return the commands it passes as (stamp,
        robot, speed, heading). Caller holds the lock, then _deliver()s them
        outside it, under _deliver_lock so two pullers cannot reorder them."""
        self._frame = frame
        stamp = frame.stamp
        i = frame.seq % len(self._spans)
        self._index = i
        if i < self._last_i:                 # looped: replay the commands too
            self._cmd_i = 0
        self._last_i = i
        due = []
        rel_i = self._rel[i]
        scale = 1.0 if self.lockstep else self.rate
        while self._cmd_i < len(self._cmds) and self._cmds[self._cmd_i][0] <= rel_i:
            rel_c, robot, speed, heading = self._cmds[self._cmd_i]
            due.append((stamp + (rel_c - rel_i) / scale, robot, speed, heading))
            self._cmd_i += 1
        self._count += 1
        now = time.time()
        dt = now - self._fps_t0
        if dt >= 1.0:
            self._fps_est = (self._count - self._fps_n0) / dt
            self._fps_t0 = now
            self._fps_n0 = self._count
        return due

    def _deliver(self, due):
        handler = self.cmd_handler
        if handler is None:
            return
        for stamp, robot, speed, heading in due:
            try:
                handler(robot, speed, heading, stamp)
            except Exception as e:                              # noqa: BLE001
                self._warn(f'replay command handler failed: {e}')

    # ------------------------------------------------------------- controls
    def apply_controls(self):
        return {}

    def reopen(self):
        """Nothing to reopen: a file does not stall."""
        return True

    # --------------------------------------------------------------- pulling
    def on_tick(self):
        """No-op: the grab thread fills the cache asynchronously."""

    def latest_frame(self):
        with self._deliver_lock:
            due = []
            with self._cond:
                if self.lockstep and self._handed and self._pending is not None:
                    due = self._publish(self._pending)
                    self._pending = None
                    self._handed = False
                    self._cond.notify_all()
                if self._frame is not None:
                    self._handed = True
                out = self._frame
            self._deliver(due)
        return out

    def current_frame(self):
        with self._lock:
            return self._frame

    def wait_frame(self, after_seq, timeout):
        """The next frame (seq != after_seq), or None after `timeout` s.
        Lockstep: pulls it, exactly like latest_frame()."""
        deadline = time.time() + timeout
        while True:
            f = self.latest_frame()
            if f is not None and f.seq != after_seq:
                return f
            with self._cond:
                if self.lockstep and self._pending is not None:
                    continue                 # promote it on the next pull
                left = deadline - time.time()
                if left <= 0 or self._stop.is_set():
                    return None
                self._cond.wait(min(left, 0.05))

    def latest_gray(self):
        f = self.latest_frame()
        return (None, 0.0) if f is None else (f.gray, f.stamp)

    def stats(self):
        with self._lock:
            return {'frames': self._count,
                    'consume_fps': round(self._fps_est, 1),
                    'driver_fps': round(self.driver_fps, 1),
                    'decode_ms': round(self._decode_ms, 2),
                    'stamp': self._frame.stamp if self._frame else 0.0,
                    'device': self.path,
                    'replay': {'index': self._index, 'n': len(self._spans),
                               'loops': self._loops, 'skipped': self._skipped,
                               'lockstep': self.lockstep,
                               'eof': self._eof and self._pending is None}}

    def close(self):
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._frame = self._pending = None
        if self._mm is not None:
            try:
                self._mm.close()
            except BufferError:              # a frame view still alive; GC frees it
                pass
            self._mm = None
        if self._file is not None:
            self._file.close()
            self._file = None


def _sim_pattern(ball_d=34, square=28, pt_sep=26, pt_r=3):
    """Robot brightness pattern, 0-1: ball body, LED matrix square and the two
    point LEDs either side of it, slightly blurred."""
    size = ball_d + 6 + (ball_d % 2 == 0)      # odd, with margin
    c = size // 2
    t = np.zeros((size, size), np.float32)
    cv2.circle(t, (c, c), ball_d // 2, 0.25, -1)                # ball body
    h = square // 2
    cv2.rectangle(t, (c - h, c - h), (c + h, c + h), 0.55, -1)   # matrix
    r = pt_sep // 2
    cv2.circle(t, (c - r, c), pt_r, 1.0, -1)                     # point LEDs
    cv2.circle(t, (c + r, c), pt_r, 1.0, -1)
    return cv2.GaussianBlur(t, (3, 3), 0)


def _rotate(t, deg):
    c = t.shape[0] / 2 - 0.5
    m = cv2.getRotationMatrix2D((c, c), deg, 1.0)
    return cv2.warpAffine(t, m, t.shape[::-1], flags=cv2.INTER_LINEAR)


class SimOverheadSource:
    """Synthetic overhead frames: the robot pattern on an uneven ~16-count floor.

    The floor falls from `floor` in the middle to `floor - floor_falloff` in the
    corners, like the real arena (19 mid, 8 corners), and robots scale with the
    local floor like the real ones (3.4-5.6x), so tests exercise the real
    local-floor ratio detector. Robots move at a constant pixel velocity;
    `set_lit` brightens one ball, which is what the serial-probe linking tests
    need; `set_gain` scales the whole scene (room brightness) for drift tests.
    Gray only: frames carry no JPEG, so there is no colour and no LED heading.
    on_tick() renders the next frame; wait_frame() renders one at `fps`.
    """

    def __init__(self, width=1280, height=720, positions=None, n=5, seed=0,
                 noise=3, ball_d=34, floor=18.0, floor_falloff=6.0, fps=30.0,
                 logger=None):
        self.width, self.height = int(width), int(height)
        self.fps = float(fps) if fps and fps > 0 else 30.0
        self.noise = int(noise)
        self.ball_d = int(ball_d)
        self.gain = 1.0
        ys, xs = np.mgrid[0:self.height, 0:self.width].astype(np.float32)
        r2 = ((xs - self.width / 2) / (self.width / 2)) ** 2 + \
            ((ys - self.height / 2) / (self.height / 2)) ** 2
        self._floor = (float(floor) - float(floor_falloff) * r2).astype(np.float32)
        self._log = logger
        rng = np.random.default_rng(seed)
        if positions is None:
            positions = [(float(rng.integers(80, self.width - 80)),
                          float(rng.integers(80, self.height - 80)))
                         for _ in range(n)]
        self.positions = [list(map(float, p)) for p in positions]
        self.velocities = [[0.0, 0.0] for _ in self.positions]
        self.angles = [float((i * 37) % 180) for i in range(len(self.positions))]
        self._lit = None
        self._pattern = _sim_pattern(ball_d=self.ball_d)
        self._gray = None
        self._stamp = 0.0
        self._seq = 0
        self._rng = rng
        self._t0 = time.time()

    # ------------------------------------------------------------- scripting
    def set_positions(self, positions):
        self.positions = [list(map(float, p)) for p in positions]
        while len(self.velocities) < len(self.positions):
            self.velocities.append([0.0, 0.0])
        while len(self.angles) < len(self.positions):
            self.angles.append(0.0)

    def set_velocity(self, i, vx, vy):
        self.velocities[i] = [float(vx), float(vy)]

    def set_lit(self, i):
        self._lit = int(i)

    def clear_lit(self):
        self._lit = None

    def set_gain(self, g):
        self.gain = float(g)

    def step(self, dt):
        for i, (vx, vy) in enumerate(self.velocities):
            self.positions[i][0] += vx * dt
            self.positions[i][1] += vy * dt

    # --------------------------------------------------------------- pulling
    def start(self):
        self.on_tick()

    def _render(self):
        frame = self._floor * self.gain + \
            self._rng.random((self.height, self.width), dtype=np.float32) * self.noise
        for i, (cx, cy) in enumerate(self.positions):
            r = _rotate(self._pattern, self.angles[i % len(self.angles)])
            h, w = r.shape
            x0, y0 = int(round(cx)) - w // 2, int(round(cy)) - h // 2
            x1, y1 = x0 + w, y0 + h
            sx0, sy0 = max(0, x0), max(0, y0)
            sx1, sy1 = min(self.width, x1), min(self.height, y1)
            if sx1 <= sx0 or sy1 <= sy0:
                continue
            r = r[sy0 - y0:sy1 - y0, sx0 - x0:sx1 - x0]
            # robots scale with the local floor: body ~4.5x, matrix ~8.7x
            val = self._floor[sy0:sy1, sx0:sx1] * self.gain * (1.0 + 14.0 * r)
            if i == self._lit:
                val = val + 90.0 * (r > 0.05)          # the ball only, not the square
            frame[sy0:sy1, sx0:sx1] = np.maximum(frame[sy0:sy1, sx0:sx1], val)
        return np.clip(np.round(frame), 0, 255).astype(np.uint8)

    def on_tick(self):
        self._gray = self._render()          # fresh array every tick
        self._stamp = time.time()
        self._seq += 1

    def latest_frame(self):
        # built on demand, so a test that overwrites _stamp gets that stamp
        if self._gray is None:
            return None
        return Frame(self._seq, self._stamp, self._gray, None)

    current_frame = latest_frame

    def wait_frame(self, after_seq, timeout):
        """Render the next frame, paced at `fps`."""
        due = self._stamp + 1.0 / self.fps - time.time()
        if due > 0:
            time.sleep(min(due, timeout))
        self.on_tick()
        return self.latest_frame()

    def latest_gray(self):
        return self._gray, self._stamp

    def apply_controls(self):
        return {}

    def reopen(self):
        return True

    def stats(self):
        return {'frames': 0, 'consume_fps': 0.0, 'driver_fps': 0.0,
                'stamp': self._stamp, 'device': 'sim'}

    def close(self):
        self._gray = None


def build_source(kind, **kw):
    """Factory. 'v4l2' for the real camera, 'sim' for synthetic frames, 'replay'
    for a recorded raw MJPEG file.

    Each branch filters to its own kwargs so one call site can pass the union of
    both sources' options without the unused ones becoming a TypeError.
    """
    if kind == 'v4l2':
        allowed = ('device', 'width', 'height', 'fps', 'fourcc', 'controls',
                   'set_fps', 'logger')
        return V4L2Source(**{k: v for k, v in kw.items() if k in allowed})
    if kind == 'sim':
        allowed = ('width', 'height', 'positions', 'n', 'seed', 'noise',
                   'ball_d', 'floor', 'floor_falloff', 'fps', 'logger')
        return SimOverheadSource(**{k: v for k, v in kw.items() if k in allowed})
    if kind == 'replay':
        return ReplaySource(path=kw.get('replay_path'), fps=kw.get('fps', 60.0),
                            rate=kw.get('replay_rate', 1.0),
                            loop=kw.get('replay_loop', False),
                            lockstep=kw.get('replay_lockstep', False),
                            logger=kw.get('logger'))
    raise SystemExit(f"unknown source '{kind}' (expected 'v4l2', 'sim' or 'replay')")

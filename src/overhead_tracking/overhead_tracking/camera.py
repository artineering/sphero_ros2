#!/usr/bin/env python3
"""Frame sources: the real global-shutter camera and a simulator.

Duck-typed pull interface, so the node is identical on hardware and in tests:
    start() / on_tick() / latest_gray() -> (gray|None, stamp) / close()

IMMUTABILITY INVARIANT (load-bearing)
-------------------------------------
Every published frame is a FRESH array that is never written to again. Consumers
may therefore hold a reference, hand a numpy *view* to a worker thread, and queue
it for annotation, all without copying. Violating this -- reusing one buffer --
would silently corrupt in-flight template matches. Corollary: anything holding a
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

import subprocess
import threading
import time

import cv2
import numpy as np


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
        self._gray = None
        self._bgr = None                     # colour kept for LED hue only
        self._stamp = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._count = 0
        self._fps_est = 0.0
        self._fps_t0 = 0.0
        self._fps_n0 = 0
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
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.005)
                continue
            # fresh array every frame -- see the immutability invariant
            if frame.ndim == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                bgr = np.array(frame, copy=True)
            else:
                gray = np.array(frame, copy=True)
                bgr = None
            now = time.time()
            with self._lock:
                self._gray = gray
                self._bgr = bgr
                self._stamp = now
                self._count += 1
                dt = now - self._fps_t0
                if dt >= 1.0:
                    self._fps_est = (self._count - self._fps_n0) / dt
                    self._fps_t0 = now
                    self._fps_n0 = self._count

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

    def latest_gray(self):
        with self._lock:
            return self._gray, self._stamp

    def latest_bgr(self):
        with self._lock:
            return self._bgr, self._stamp

    def stats(self):
        with self._lock:
            return {'frames': self._count,
                    'consume_fps': round(self._fps_est, 1),
                    'driver_fps': round(self.driver_fps, 1),
                    'stamp': self._stamp, 'device': self.device}

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


class SimOverheadSource:
    """Synthetic overhead frames rendering REAL templates on a dark arena.

    Uses `template.build_template` output rather than a stub, so tests exercise
    the actual matcher. Robots move at a constant pixel velocity; `set_lit` makes
    one much brighter, which is what the serial-probe linking tests need.
    """

    def __init__(self, width=1280, height=720, positions=None, n=5, seed=0,
                 noise=3, ball_d=34, logger=None):
        from . import template as _t
        self._t = _t
        self.width, self.height = int(width), int(height)
        self.noise = int(noise)
        self.ball_d = int(ball_d)
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
        self._tmpl = self._t.build_template(ball_d=self.ball_d)
        self._gray = None
        self._bgr = None                     # colour kept for LED hue only
        self._stamp = 0.0
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

    def step(self, dt):
        for i, (vx, vy) in enumerate(self.velocities):
            self.positions[i][0] += vx * dt
            self.positions[i][1] += vy * dt

    # --------------------------------------------------------------- pulling
    def start(self):
        self.on_tick()

    def _render(self):
        frame = (self._rng.random((self.height, self.width)) * self.noise
                 ).astype(np.uint8)
        for i, (cx, cy) in enumerate(self.positions):
            r = self._t.rotate(self._tmpl, self.angles[i % len(self.angles)])
            scale = 255.0 if i != self._lit else 255.0
            patch = np.clip(r * scale, 0, 255).astype(np.uint8)
            if i == self._lit:
                patch = np.clip(patch.astype(np.int32) + 90, 0, 255).astype(np.uint8)
            h, w = patch.shape
            x0, y0 = int(round(cx)) - w // 2, int(round(cy)) - h // 2
            x1, y1 = x0 + w, y0 + h
            sx0, sy0 = max(0, x0), max(0, y0)
            sx1, sy1 = min(self.width, x1), min(self.height, y1)
            if sx1 <= sx0 or sy1 <= sy0:
                continue
            sub = patch[sy0 - y0:sy1 - y0, sx0 - x0:sx1 - x0]
            frame[sy0:sy1, sx0:sx1] = np.maximum(frame[sy0:sy1, sx0:sx1], sub)
        return frame

    def on_tick(self):
        self._gray = self._render()          # fresh array every tick
        self._stamp = time.time()

    def latest_gray(self):
        return self._gray, self._stamp

    def latest_bgr(self):
        return self._bgr, self._stamp

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
    """Factory. 'v4l2' for the real camera, 'sim' for synthetic frames.

    Each branch filters to its own kwargs so one call site can pass the union of
    both sources' options without the unused ones becoming a TypeError.
    """
    if kind == 'v4l2':
        allowed = ('device', 'width', 'height', 'fps', 'fourcc', 'controls',
                   'set_fps', 'logger')
        return V4L2Source(**{k: v for k, v in kw.items() if k in allowed})
    if kind == 'sim':
        allowed = ('width', 'height', 'positions', 'n', 'seed', 'noise',
                   'ball_d', 'logger')
        return SimOverheadSource(**{k: v for k, v in kw.items() if k in allowed})
    raise SystemExit(f"unknown source '{kind}' (expected 'v4l2' or 'sim')")

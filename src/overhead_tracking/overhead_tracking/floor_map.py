#!/usr/bin/env python3
"""Floor threshold map with brightness-drift rebuild (pure, no ROS).

The detector thresholds at `ratio` x the local floor (blobs.threshold_map), and
the margins are thin: robots 3.4-5.6x, tape <= 2.75x, floor noise <= 2.5x. The
camera runs at fixed gain and manual exposure, so a room-light or daylight
change shifts the whole scene and a stale map breaks: at 0.7x the robot blob
median falls to ~2.4x (on the 12-robot test frame blobs shrink ~25% and robots
start going missing by 0.55x), at ~1.3x floor noise reaches ~3.25x (false blobs).

observe() tracks the in-arena floor level -- the median of a strided sample of
ROI pixels, which a dozen robots (a few % of the arena) do not move -- and
flags a rebuild once it has drifted more than `drift_frac` from the level the
map was built at for `drift_hold_sec` (a passing shadow or someone leaning over
the arena does not count). It also buffers ROI views of recent frames, spaced
`buffer_spacing_sec` apart, for start_rebuild() to build from in a background
thread. The views are not copied (a 1.4 MP copy is ~0.75 ms on the Pi 5), so
the caller must not write into a frame after passing it in -- a fresh array per
frame, as cv2.imdecode returns, is fine. The 61 px opening removes robots, so
rebuilding with them on the floor is safe. The old map stays in use until the
new one is swapped in.
"""

import collections
import threading
import time

import numpy as np

from overhead_tracking.blobs import threshold_map

_WHOLE = {'sl': (slice(None), slice(None))}           # buffered frames are ROI views


class FloorMap:

    def __init__(self, roi, ratio=3.2, drift_frac=0.20, drift_hold_sec=2.0,
                 buffer_size=5, buffer_spacing_sec=0.2, sample_stride=16):
        self.roi = roi
        self.ratio = ratio
        self.drift_frac = drift_frac
        self.drift_hold_sec = drift_hold_sec
        self.buffer_spacing_sec = buffer_spacing_sec
        self.stride = sample_stride
        self._sample_mask = roi['mask'][::sample_stride, ::sample_stride] > 0
        self._buf = collections.deque(maxlen=buffer_size)
        self._buf_t = None
        self._map = (None, None)                           # (tmap, ref level), swapped as one
        self._build_lock = threading.Lock()                # held while a build runs
        self._drift_since = None
        self.level = None
        self.rebuild_count = 0
        self.last_rebuild_sec = None

    @property
    def tmap(self):
        return self._map[0]

    @property
    def ref_level(self):
        return self._map[1]

    @property
    def busy(self):
        return self._build_lock.locked()

    def _level(self, crop):
        return float(np.median(crop[::self.stride, ::self.stride][self._sample_mask]))

    def _build(self, crops):
        t0 = time.monotonic()
        tmap = threshold_map(crops, _WHOLE, self.ratio)
        ref = float(np.median([self._level(c) for c in crops]))
        self._map = (tmap, ref)
        self._drift_since = None
        self.rebuild_count += 1
        self.last_rebuild_sec = time.monotonic() - t0

    def rebuild(self, frames):
        """Synchronous build from full frames (startup, ~/detect_arena)."""
        with self._build_lock:
            self._build([f[self.roi['sl']] for f in frames])

    def start_rebuild(self):
        """Build from the buffered frames in a background thread.

        Returns False if a build is already running or nothing is buffered.
        """
        crops = list(self._buf)
        if not crops or not self._build_lock.acquire(blocking=False):
            return False

        def run():
            try:
                self._build(crops)
            finally:
                self._build_lock.release()
        threading.Thread(target=run, daemon=True).start()
        return True

    def observe(self, gray, t):
        """Track the floor level of a full frame at time `t` (s); True = rebuild due."""
        crop = gray[self.roi['sl']]
        if self._buf_t is None or t - self._buf_t >= self.buffer_spacing_sec:
            self._buf.append(crop)
            self._buf_t = t
        self.level = self._level(crop)
        ref = self.ref_level
        if ref is None or self.busy or abs(self.level / max(ref, 1.0) - 1.0) <= self.drift_frac:
            self._drift_since = None
            return False
        if self._drift_since is None:
            self._drift_since = t
        return t - self._drift_since >= self.drift_hold_sec

    def state(self):
        return {'floor_level': self.level, 'ref_level': self.ref_level,
                'rebuilding': self.busy, 'rebuild_count': self.rebuild_count,
                'last_rebuild_sec': self.last_rebuild_sec}

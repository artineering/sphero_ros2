#!/usr/bin/env python3
"""Colour for the LED heading at 60 fps, decoded beside the gray decode (pure, no ROS).

A full-colour decode of a 1920x1200 frame costs ~22 ms on the Pi 5, more than the
16.6 ms frame budget on its own. Half-resolution colour (IMREAD_REDUCED_COLOR_2,
libjpeg DCT scaling) costs ~7 ms and still finds both LEDs: on
kf_20261005_171405 (SB-418F, 2471 frames) the pair was found in 99.8% of frames
against 100% at full resolution, heading within 2.5 deg mean / 8 deg p95 of the
full-resolution heading, bias -0.3 deg. The camera's MJPEG is 4:2:2, so the half
image keeps the native horizontal chroma and loses only half the vertical.

`ColorWorker` runs that decode on its own thread (imdecode releases the GIL)
while the caller does the gray decode + blob detect of the SAME frame, so the
colour is usually ready by the time the blobs are. In mode 'hsv' (default) it also
converts the frame to HSV there, once, so the per-robot LED lookup only indexes into
it; in mode 'bgr' it returns the BGR image and find_led converts each small patch
(same headings, less worker time, a little more per robot). Results are
keyed by the caller's frame sequence number and only ever returned for that exact frame:
the LED search uses that frame's blob centroids, so a colour image from another
frame is never paired with them. A frame whose colour is not ready in time gets
no LED fix and the EKF carries theta.
"""

import threading
import time

import cv2

from overhead_tracking import heading as hdg

SCALE = 2                                  # IMREAD_REDUCED_COLOR_2 halves each axis


MODES = ('hsv', 'bgr')


class ColorWorker:
    """Half-resolution colour decode (+ HSV conversion in mode 'hsv') on one thread.

    `submit(seq, buf)` never blocks; a job not yet started is replaced by a newer
    one (latest wins). `get(seq, timeout)` returns the image of frame `seq` in
    `mode`'s colour space, or None if it was skipped or is not done within
    `timeout` seconds. Pass `is_hsv=worker.is_hsv` to `led_heading`.
    """

    def __init__(self, mode='hsv'):
        if mode not in MODES:
            raise ValueError(f'ColorWorker mode must be one of {MODES}, got {mode!r}')
        self.mode = mode
        self._cv = threading.Condition()
        self._job = None                   # (seq, buf) not yet started
        self._busy = None                  # seq being decoded
        self._done = (None, None)          # (seq, image) of the last finished decode
        self._stop = False
        self._thread = threading.Thread(target=self._run, name='color_decode', daemon=True)
        self._thread.start()

    @property
    def is_hsv(self):
        return self.mode == 'hsv'

    def submit(self, seq, buf):
        """Queue `buf` (JPEG bytes, not reused by the caller) as frame `seq`."""
        with self._cv:
            self._job = (seq, buf)
            self._cv.notify_all()

    def get(self, seq, timeout=0.010):
        """Half-res image of frame `seq` in `mode`, or None (dropped, failed or late)."""
        deadline = time.monotonic() + timeout
        with self._cv:
            while True:
                if self._done[0] == seq:
                    return self._done[1]
                pending = (self._busy == seq or
                           (self._job is not None and self._job[0] == seq))
                left = deadline - time.monotonic()
                if not pending or left <= 0:
                    return None
                self._cv.wait(left)

    def close(self):
        with self._cv:
            self._stop = True
            self._cv.notify_all()
        self._thread.join()

    def _run(self):
        while True:
            with self._cv:
                while self._job is None and not self._stop:
                    self._cv.wait()
                if self._stop:
                    return
                seq, buf = self._job
                self._job, self._busy = None, seq
            img = cv2.imdecode(buf, cv2.IMREAD_REDUCED_COLOR_2)
            if img is not None and self.is_hsv:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            with self._cv:
                self._done, self._busy = (seq, img), None
                self._cv.notify_all()


def led_heading(color, u, v, *, is_hsv, search_px=30, min_sep_px=13, sat_min=90, val_min=60):
    """Field heading (deg) from the LED pair near full-res blob centroid (u, v), or None.

    `color` is the half-resolution image from ColorWorker.get; `is_hsv` (required,
    pass `worker.is_hsv`) says whether it is HSV or BGR. The search radius and
    the minimum pair separation are given in full-resolution pixels, as elsewhere.
    """
    cu, cv_ = (u + 0.5) / SCALE - 0.5, (v + 0.5) / SCALE - 0.5
    red = hdg.find_led(color, cu, cv_, hdg.RED_HUE, search_px // SCALE, sat_min, val_min,
                       is_hsv=is_hsv)
    if red is None:
        return None
    green = hdg.find_led(color, cu, cv_, hdg.GREEN_HUE, search_px // SCALE, sat_min, val_min,
                         is_hsv=is_hsv)
    if green is None:
        return None
    return hdg.heading_from_pair(red, green, min_sep_px / SCALE)

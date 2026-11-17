#!/usr/bin/env python3
"""Half-resolution colour worker and LED heading from its image."""

import cv2
import numpy as np
import pytest

from overhead_tracking.color_decode import ColorWorker, led_heading, SCALE


def robot_jpeg(cx, cy, heading_deg, sep=28, size=(400, 640), quality=90):
    """JPEG of a gray ball with back red / front green LEDs; heading CCW, v down."""
    img = np.full((*size, 3), 12, np.uint8)
    cv2.circle(img, (cx, cy), 20, (150, 150, 150), -1)
    th = np.deg2rad(heading_deg)
    du, dv = sep / 2 * np.cos(th), -sep / 2 * np.sin(th)
    cv2.circle(img, (int(round(cx - du)), int(round(cy - dv))), 3, (0, 0, 255), -1)
    cv2.circle(img, (int(round(cx + du)), int(round(cy + dv))), 3, (0, 255, 0), -1)
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    assert ok
    return buf


MODES = ['hsv', 'bgr']


def half(buf, mode):
    """What ColorWorker(mode).get returns for `buf`."""
    img = cv2.imdecode(buf, cv2.IMREAD_REDUCED_COLOR_2)
    return cv2.cvtColor(img, cv2.COLOR_BGR2HSV) if mode == 'hsv' else img


def ang_diff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


@pytest.fixture(params=MODES)
def worker(request):
    w = ColorWorker(mode=request.param)
    yield w
    w.close()


def heading_of(worker, img):
    return led_heading(img, 320.0, 200.0, is_hsv=worker.is_hsv)


@pytest.mark.parametrize('mode', MODES)
@pytest.mark.parametrize('heading', [0, 37, 90, 145, 180, 233, 300])
def test_heading_from_half_resolution(heading, mode):
    h = led_heading(half(robot_jpeg(320, 200, heading), mode), 320.0, 200.0,
                    is_hsv=mode == 'hsv')
    assert h is not None and ang_diff(h, heading) < 6.0


@pytest.mark.parametrize('heading', range(0, 360, 7))
def test_modes_give_identical_headings(heading):
    buf = robot_jpeg(317, 203, heading)
    hsv = led_heading(half(buf, 'hsv'), 317.0, 203.0, is_hsv=True)
    bgr = led_heading(half(buf, 'bgr'), 317.0, 203.0, is_hsv=False)
    assert hsv is not None and hsv == bgr


@pytest.mark.parametrize('mode', MODES)
def test_no_heading_without_leds(mode):
    img = np.full((400, 640, 3), 12, np.uint8)
    cv2.circle(img, (320, 200), 20, (150, 150, 150), -1)
    assert led_heading(half(cv2.imencode('.jpg', img)[1], mode), 320.0, 200.0,
                       is_hsv=mode == 'hsv') is None


def test_bad_mode_raises():
    with pytest.raises(ValueError):
        ColorWorker(mode='rgb')


def test_mode_and_is_hsv(worker):
    assert worker.mode in MODES and worker.is_hsv == (worker.mode == 'hsv')


def test_get_returns_half_res_image_of_that_frame_in_its_mode(worker):
    buf = robot_jpeg(320, 200, 90)
    worker.submit(7, buf)
    img = worker.get(7, timeout=1.0)
    assert img is not None and img.shape == (400 // SCALE, 640 // SCALE, 3)
    assert np.array_equal(img, half(buf, worker.mode))
    other = 'bgr' if worker.is_hsv else 'hsv'
    assert not np.array_equal(img, half(buf, other))
    assert ang_diff(heading_of(worker, img), 90) < 6.0


def test_get_unknown_frame_is_none_without_waiting(worker):
    worker.submit(1, robot_jpeg(320, 200, 0))
    assert worker.get(1, timeout=1.0) is not None
    assert worker.get(2, timeout=5.0) is None      # never submitted: returns at once


def test_results_are_never_paired_with_another_frame(worker):
    # Each frame encodes its own heading; whatever is returned must match its seq,
    # and frames replaced before decoding come back as None (latest wins).
    headings = {seq: (seq * 29) % 360 for seq in range(40)}
    bufs = {seq: robot_jpeg(320, 200, h) for seq, h in headings.items()}
    got = 0
    for seq, buf in bufs.items():
        worker.submit(seq, buf)
        if seq % 3 == 0:                            # caller only waits on some frames
            img = worker.get(seq, timeout=1.0)
            assert img is not None
            assert ang_diff(heading_of(worker, img), headings[seq]) < 6.0
            got += 1
    assert got == 14
    last = worker.get(39, timeout=1.0)
    assert last is not None and ang_diff(heading_of(worker, last), headings[39]) < 6.0


def test_close_stops_the_thread():
    w = ColorWorker()
    w.close()
    assert not w._thread.is_alive()

#!/usr/bin/env python3
"""Bright-blob detector: local-floor threshold, fragment join, split, margin."""

import os

import cv2
import numpy as np

from overhead_tracking import blobs as B

H, W = 600, 800
ARENA = np.array([[100.0, 80.0], [700.0, 80.0], [700.0, 520.0], [100.0, 520.0]])
R = 16                                                    # disc ~800 px, like a robot


def floor():
    """Dim, uneven floor: ~20 in the middle falling to ~8 in the corners."""
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    r2 = ((xx - W / 2) / (W / 2)) ** 2 + ((yy - H / 2) / (H / 2)) ** 2
    return 20.0 - 6.0 * r2


def disc(img, base, cx, cy, r=R, gain=5.0):
    m = np.zeros((H, W), np.uint8)
    cv2.circle(m, (int(cx), int(cy)), r, 1, -1)
    img[m > 0] = gain * base[m > 0]
    return m


def run(img, ref_area=None):
    g = np.clip(np.round(img), 0, 255).astype(np.uint8)
    roi = B.make_roi(ARENA, g.shape)
    return B.detect(g, roi, B.threshold_map([g], roi), ref_area=ref_area)


def test_bright_discs_found_on_uneven_floor_tape_ignored():
    base = floor()
    img = base.copy()
    # blue tape: a long bright line at 2.7x the local floor, under the 3.2x ratio
    tape = np.zeros((H, W), np.uint8)
    cv2.line(tape, (110, 300), (690, 300), 1, 6)
    img[tape > 0] = 2.7 * base[tape > 0]
    centres = [(140, 120), (660, 120), (660, 480), (140, 480), (400, 200), (300, 400)]
    for cx, cy in centres:                                # corners and middle
        disc(img, base, cx, cy)
    out = run(img)
    assert len(out) == len(centres)
    got = sorted(b['Centroid'] for b in out)
    for (gx, gy), (cx, cy) in zip(got, sorted(centres)):
        assert abs(gx - cx) < 1 and abs(gy - cy) < 1
    assert not any(b['Split'] for b in out)


def test_fragmented_disc_joins_into_one_blob():
    base = floor()
    img = base.copy()
    disc(img, base, 400, 300, r=14)
    for y in (295, 296, 304, 305):                        # dark side: 3 pieces < 200 px
        img[y, 380:420] = base[y, 380:420]
    g = np.clip(np.round(img), 0, 255).astype(np.uint8)
    roi = B.make_roi(ARENA, g.shape)
    tmap = B.threshold_map([g], roi)
    b = cv2.compare(g[roi['sl']], tmap, cv2.CMP_GT) & roi['mask']
    pieces = cv2.findContours(b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    assert len(pieces) == 3 and all(cv2.contourArea(c) < 200 for c in pieces)
    out = B.detect(g, roi, tmap)
    assert len(out) == 1
    cx, cy = out[0]['Centroid']
    assert abs(cx - 400) < 1.5 and abs(cy - 300) < 1.5


def touching_pair(img, base):
    disc(img, base, 400, 300)
    disc(img, base, 400 + 2 * R, 300)


def test_touching_pair_split_with_three_or_more_blobs():
    base = floor()
    img = base.copy()
    touching_pair(img, base)
    for c in [(200, 150), (600, 150), (200, 450)]:
        disc(img, base, *c)
    out = run(img)
    assert len(out) == 5
    split = sorted(b['Centroid'] for b in out if b['Split'])
    assert len(split) == 2
    assert abs(split[0][0] - 400) < 3 and abs(split[1][0] - 432) < 3


def test_touching_pair_alone_needs_ref_area_to_split():
    base = floor()
    img = base.copy()
    touching_pair(img, base)
    alone = run(img)
    assert len(alone) == 1 and not alone[0]['Split']      # no median from 1 blob
    area = int(np.count_nonzero(disc(np.zeros((H, W), np.float32), base, 400, 300)))
    out = run(img, ref_area=area)
    assert len(out) == 2 and all(b['Split'] for b in out)


def test_centroid_within_margin_kept_beyond_dropped():
    base = floor()
    img = base.copy()
    disc(img, base, 400, ARENA[0, 1] - 10)               # 10 px above the top edge
    disc(img, base, 250, ARENA[0, 1] - 45)               # 45 px above: past the margin
    out = run(img)
    assert len(out) == 1
    assert abs(out[0]['Centroid'][0] - 400) < 1


# One frame (index 30) of recordings/motion_20261005_164754/raw.mjpeg, 12 robots
# in view, cropped to the arena ROI at full-frame offset (275, 35).
FIXTURE = os.path.join(os.path.dirname(__file__), 'data', 'arena_12_robots.png')
FIXTURE_OFFSET = np.array([275, 35])
# ARENA_PX in scripts/blob_detect_timing.py (TL TR BR BL, full-frame px)
ARENA_PX = np.array([[318.5, 60.0], [1660.0, 83.5], [1680.5, 1056.0], [300.0, 1065.0]])


def test_recorded_frame_finds_all_twelve_robots():
    g = cv2.imread(FIXTURE, cv2.IMREAD_GRAYSCALE)
    roi = B.make_roi(ARENA_PX - FIXTURE_OFFSET, g.shape)
    out = B.detect(g, roi, B.threshold_map([g], roi))
    assert len(out) == 12
    assert all(400 <= b['Area'] <= 1000 for b in out)

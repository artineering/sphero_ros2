#!/usr/bin/env python3
"""LED search and the LED-pair heading."""

import cv2
import numpy as np
import pytest

from overhead_tracking import heading as H


def frame_with_led(cx, cy, bearing_deg, radius=13, colour=(0, 0, 255)):
    """BGR frame with one coloured blob at `bearing_deg` from (cx, cy)."""
    img = np.zeros((200, 200, 3), np.uint8)
    th = np.deg2rad(bearing_deg)
    u = int(round(cx + radius * np.cos(th)))
    v = int(round(cy - radius * np.sin(th)))       # v grows downward
    cv2.circle(img, (u, v), 4, colour, -1)
    return img


def test_finds_the_red_blob():
    img = frame_with_led(100, 100, 0.0)
    assert H.find_led(img, 100, 100, H.RED_HUE) is not None


def test_red_search_ignores_green():
    img = frame_with_led(100, 100, 0.0, colour=(0, 255, 0))
    assert H.find_led(img, 100, 100, H.RED_HUE) is None
    assert H.find_led(img, 100, 100, H.GREEN_HUE) is not None


def test_precomputed_hsv_gives_the_same_led():
    img = frame_with_led(100, 100, 37.0)
    img[5:9, 5:9] = (0, 0, 255)                    # a second red blob, near the corner
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    for cx, cy in ((100, 100), (3, 3), (198, 196)):  # incl. windows clipped by the edge
        for hue in (H.RED_HUE, H.GREEN_HUE):
            assert (H.find_led(hsv, cx, cy, hue, 30, 90, 60, is_hsv=True) ==
                    H.find_led(img, cx, cy, hue, 30, 90, 60))


@pytest.mark.parametrize('hue_range', [H.RED_HUE, H.GREEN_HUE, (0.0, 359.0),
                                       (81.0, 81.0), (359.0, 1.0)])
def test_mask_matches_the_degree_thresholds_for_every_hsv_value(hue_range):
    """The inRange bounds equal the plain rule: hue*2 in range, sat/val at least the minimum."""
    h, s, v = np.meshgrid(np.arange(180), np.arange(256), np.arange(256), indexing='ij')
    hsv = np.stack([h, s, v], -1).astype(np.uint8).reshape(180 * 256, 256, 3)
    lo, hi = hue_range
    deg = hsv[:, :, 0].astype(np.int32) * 2
    band = (deg >= lo) | (deg <= hi) if lo > hi else (deg >= lo) & (deg <= hi)
    want = band & (hsv[:, :, 1] >= 90) & (hsv[:, :, 2] >= 60)
    got = np.zeros(want.shape, bool)
    for b_lo, b_hi in H._hue_bounds(hue_range, 90, 60):
        got |= cv2.inRange(hsv, b_lo, b_hi) > 0
    assert np.array_equal(got, want)


def test_none_when_nothing_lit():
    blank = np.zeros((200, 200, 3), np.uint8)
    assert H.find_led(blank, 100, 100, H.RED_HUE) is None


def test_pair_beats_the_bank_quantisation():
    """A pair gives a continuous angle the 15 deg spine bank cannot express."""
    for want in (7.0, 52.0, 143.0, 218.0, 301.0):
        r = 13.0
        th = np.deg2rad(want)
        red = (100 - r * np.cos(th), 100 + r * np.sin(th))
        green = (100 + r * np.cos(th), 100 - r * np.sin(th))
        got = H.heading_from_pair(red, green, 13.0)
        assert got is not None and H.ang_diff(got, want) < 0.5, (want, got)


def test_pair_rejected_when_green_merges_with_the_matrix():
    """A contaminated green centroid collapses the separation -> fall back."""
    assert H.heading_from_pair((110.0, 100.0), (104.0, 100.0), 13.0) is None

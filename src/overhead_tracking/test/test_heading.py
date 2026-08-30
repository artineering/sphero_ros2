#!/usr/bin/env python3
"""Back-LED search and spine disambiguation."""

import cv2
import numpy as np

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


def test_none_when_nothing_lit():
    blank = np.zeros((200, 200, 3), np.uint8)
    assert H.find_led(blank, 100, 100, H.RED_HUE) is None


def test_spine_picks_the_end_away_from_the_back_led():
    # back LED at 292 deg, spine 120 -> front is 120, not 300
    assert H.heading_from_spine(120.0, 651, 101, 659.2, 121.4) == 120.0
    # same spine, LED on the other side -> front flips
    assert H.heading_from_spine(120.0, 651, 101, 642.8, 80.6) == 300.0


def test_spine_all_four_quadrants():
    for want in (0.0, 90.0, 180.0, 270.0):
        img = frame_with_led(100, 100, (want + 180.0) % 360.0)
        led = H.find_led(img, 100, 100, H.RED_HUE)
        assert led is not None
        got = H.heading_from_spine(want % 180.0, 100, 100, *led)
        assert H.ang_diff(got, want) < 1.0, (want, got)


def test_v_axis_sign_is_not_mirrored():
    """A back LED BELOW the ball in the image means the front points UP (+y)."""
    assert H.heading_from_spine(90.0, 100, 100, 100, 120) == 90.0


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

#!/usr/bin/env python3
"""Absolute field heading from the front (green) and back (red) LEDs.

Find BOTH LEDs and take the bearing red -> green, which is continuous. With
either LED missing there is no reading; the tracker's EKF carries the heading.

Heading 0 is field +x, which is LEFT-TO-RIGHT in the camera image. Image v grows
downward while field y grows upward, so every bearing negates the v difference.
"""

import math

import cv2
import numpy as np

RED_HUE = (335.0, 25.0)          # wraps through 0
GREEN_HUE = (80.0, 175.0)


def _hue_bounds(hue_range, sat_min, val_min):
    """cv2.inRange (lo, hi) HSV bounds for a hue range in degrees (OpenCV hue = deg / 2)."""
    lo, hi = math.ceil(hue_range[0] / 2), math.floor(hue_range[1] / 2)
    if hue_range[0] > hue_range[1]:                  # wraps through 0
        return [((lo, sat_min, val_min), (179, 255, 255)),
                ((0, sat_min, val_min), (hi, 255, 255))]
    return [((lo, sat_min, val_min), (hi, 255, 255))]


def find_led(img, cx, cy, hue_range, search_px=30, sat_min=90, val_min=60, is_hsv=False):
    """Centroid of the largest blob of that hue near (cx, cy), or None.

    Searched rather than sampled at a fixed offset: the LED was measured 22 px
    from the blob centre, not the nominal 13.

    `img` is BGR, or with `is_hsv` the whole frame already converted with
    COLOR_BGR2HSV once, so a caller looking up many robots does not convert per call.
    """
    h, w = img.shape[:2]
    x0, y0 = max(0, int(cx) - search_px), max(0, int(cy) - search_px)
    x1, y1 = min(w, int(cx) + search_px + 1), min(h, int(cy) + search_px + 1)
    patch = img[y0:y1, x0:x1]
    if patch.size == 0:
        return None
    hsv = patch if is_hsv else cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    mask = None
    for lo, hi in _hue_bounds(hue_range, sat_min, val_min):
        m = cv2.inRange(hsv, lo, hi)
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    if not cv2.countNonZero(mask):                   # most blobs are unlit: skip the labelling
        return None
    n, _labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    if n < 2:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return float(x0 + centroids[i][0]), float(y0 + centroids[i][1])


def bearing(u0, v0, u1, v1):
    """Field bearing from (u0,v0) to (u1,v1) in pixels, degrees, 0-360."""
    return float(np.degrees(np.arctan2(-(v1 - v0), u1 - u0)) % 360.0)


def ang_diff(a, b):
    """Smallest absolute angle between two bearings, degrees."""
    return abs((a - b + 180.0) % 360.0 - 180.0)


def heading_from_pair(red, green, min_sep_px):
    """Continuous heading from back -> front LED, or None if the pair is unusable.

    A green MATRIX merges with the green front LED into one blob whose centroid
    slides toward the ball centre (measured: 418 px where an LED is ~28), which
    collapses the separation. Rejecting a short pair rejects exactly that case
    (no reading for that frame).
    """
    sep = float(np.hypot(green[0] - red[0], green[1] - red[1]))
    if sep < min_sep_px:
        return None
    return bearing(red[0], red[1], green[0], green[1])

#!/usr/bin/env python3
"""Absolute field heading from the front (green) and back (red) LEDs.

Preferred: find BOTH LEDs and take the bearing red -> green. That is continuous.

Fallback: find only the red back LED and use it to pick which end of the template
spine is the front. The template bank only spans 0-180 deg -- the two point LEDs
are identical in grayscale -- so this resolves the direction but inherits the
bank's 15 deg quantisation. Only the LED's BEARING is used, so it is a one-bit
decision: measured 2026-08-29, the real LED sat 8 deg from one spine end and 172
from the other.

Heading 0 is field +x, which is LEFT-TO-RIGHT in the camera image. Image v grows
downward while field y grows upward, so every bearing negates the v difference.
"""

import cv2
import numpy as np

RED_HUE = (335.0, 25.0)          # wraps through 0
GREEN_HUE = (80.0, 175.0)


def find_led(bgr, cx, cy, hue_range, search_px=30, sat_min=90, val_min=60):
    """Centroid of the largest blob of that hue near (cx, cy), or None.

    Searched rather than sampled at a fixed offset: the lit matrix pulls the
    matched centre off the ball centre, and the LED was measured 22 px out where
    the template puts point LEDs at 13.
    """
    h, w = bgr.shape[:2]
    x0, y0 = max(0, int(cx) - search_px), max(0, int(cy) - search_px)
    x1, y1 = min(w, int(cx) + search_px + 1), min(h, int(cy) + search_px + 1)
    patch = bgr[y0:y1, x0:x1]
    if patch.size == 0:
        return None
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0].astype(np.int32) * 2          # OpenCV packs hue in 0..179
    lo, hi = hue_range
    band = (hue >= lo) | (hue <= hi) if lo > hi else (hue >= lo) & (hue <= hi)
    mask = (band & (hsv[:, :, 1] >= sat_min) &
            (hsv[:, :, 2] >= val_min)).astype(np.uint8)
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
    and falls back to the spine.
    """
    sep = float(np.hypot(green[0] - red[0], green[1] - red[1]))
    if sep < min_sep_px:
        return None
    return bearing(red[0], red[1], green[0], green[1])


def heading_from_spine(spine_deg, cx, cy, red_u, red_v):
    """Resolve a 0-180 spine into 0-360 using the back LED. Quantised to the bank."""
    opposite = (bearing(cx, cy, red_u, red_v) + 180.0) % 360.0
    a = spine_deg % 360.0
    b = (spine_deg + 180.0) % 360.0
    return float(a if ang_diff(a, opposite) <= ang_diff(b, opposite) else b)

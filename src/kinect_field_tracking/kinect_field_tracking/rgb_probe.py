#!/usr/bin/env python3
"""RGB lit-confirmation for callsign registration (brightness-change + overlap).

The DEPTH blob provides detection + position; lighting the Sphero only CONFIRMS/
DISAMBIGUATES which depth blob we just lit. Probing is one-at-a-time serial, so
the lit Sphero is simply "the region that got much brighter than when it was
off". This is robust even when the bright LED BLOWS OUT the Kinect RGB to near
white (green-hue detection then fails) -- we detect a BRIGHTNESS INCREASE vs the
dark pre-frame, not a colour. Diffing post-pre also cancels anything already lit
in both frames (e.g. previously-registered Spheros showing solid green/red).

Because the Kinect v1 RGB and depth are different sensors (~2.5cm baseline +
different intrinsics), the lit Sphero appears tens of pixels from the depth
pixel, so matching is by OVERLAP (proximity of a lit RGB blob to the depth blob's
projected RGB centre), NOT pixel-exact patches.

NOTE: brightness alone disambiguates ONLY because probing is serial/one-at-a-time.
Simultaneous multi-COLOUR probing (future) would need hue + registered depth.

Pure helpers (no ROS): `detect_lit_blobs` finds newly-bright blobs from a
pre/post RGB pair; `match_lit_depth_blob` matches them to projected depth centres.
"""

import cv2
import numpy as np


def _luma(rgb):
    """Per-pixel brightness (max channel -- robust to a blown-out single LED)."""
    return rgb.max(axis=2).astype(np.int32)


def detect_lit_blobs(post_rgb, pre_rgb, brightness_delta=50, min_area=15):
    """Detect newly-lit blobs as regions much brighter in post than in pre.

    diff = luma(post) - luma(pre); threshold diff > brightness_delta; connected
    components -> blobs with centroid/bbox/area, filtered by min_area. Cancels
    anything already bright in BOTH frames (the diff there is ~0).

    Returns (blobs, mask) where blobs is a list of
    {'centroid': (u, v), 'bbox': (x, y, w, h), 'area': int}.
    """
    post = _luma(post_rgb)
    if pre_rgb is None:
        # no dark reference: fall back to absolute brightness (degraded)
        mask = post > (255 - brightness_delta)
    else:
        mask = (post - _luma(pre_rgb)) > brightness_delta
    mask = mask.astype(np.uint8) * 255
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    n, _labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    blobs = []
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        x = int(stats[i, cv2.CC_STAT_LEFT])
        y = int(stats[i, cv2.CC_STAT_TOP])
        w = int(stats[i, cv2.CC_STAT_WIDTH])
        hh = int(stats[i, cv2.CC_STAT_HEIGHT])
        cu, cv_ = centroids[i]
        blobs.append({'centroid': (float(cu), float(cv_)),
                      'bbox': (x, y, w, hh), 'area': area})
    return blobs, mask


def match_lit_depth_blob(projected_centres, green_blobs, overlap_tol_px):
    """Match the lit (green) RGB blob to a depth blob by projected-centre overlap.

    Parameters
    ----------
    projected_centres : list of (u, v) -- each depth blob's centre projected into
        RGB pixels (index-aligned to the depth-blob list).
    green_blobs : list of probe-colour blob dicts (from detect_probe_blobs).
    overlap_tol_px : max centre-to-centre distance for a match (absorbs the
        RGB/depth sensor offset).

    Returns dict:
      {'matched': bool, 'depth_index': int|None, 'green_index': int|None,
       'distance': float, 'reason': str}.
    Picks the globally closest (depth, green) pair; matches if within tolerance.
    This disambiguates multiple depth blobs: the lit Sphero is the depth blob
    whose projected centre is closest to a detected green blob.
    """
    if not green_blobs:
        return {'matched': False, 'depth_index': None, 'green_index': None,
                'distance': float('inf'), 'reason': 'no_green'}
    if not projected_centres:
        return {'matched': False, 'depth_index': None, 'green_index': None,
                'distance': float('inf'), 'reason': 'no_depth'}

    best = None  # (distance, depth_index, green_index)
    for di, (pu, pv) in enumerate(projected_centres):
        for gj, gb in enumerate(green_blobs):
            gu, gv = gb['centroid']
            dist = float(np.hypot(pu - gu, pv - gv))
            if best is None or dist < best[0]:
                best = (dist, di, gj)

    dist, di, gj = best
    if dist <= overlap_tol_px:
        return {'matched': True, 'depth_index': di, 'green_index': gj,
                'distance': dist, 'reason': ''}
    return {'matched': False, 'depth_index': di, 'green_index': gj,
            'distance': dist, 'reason': 'too_far'}

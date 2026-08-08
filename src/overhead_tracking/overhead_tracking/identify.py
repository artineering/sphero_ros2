#!/usr/bin/env python3
"""Blob <-> callsign linking helpers (pure, no ROS).

Identity is established by a SERIAL LED probe: light exactly one robot, find the
blob that just got brighter, bind it, turn it off, repeat. Brightness alone is
enough to disambiguate precisely BECAUSE the probe is one-at-a-time. Diffing
against a dark pre-frame also cancels anything already lit in both frames, so
previously-linked robots showing their status colour do not confuse the match.

Simultaneous multi-colour probing would need hue and is deliberately not done
here -- we measured that a bright LED saturates all three channels and reads as
white, which destroys hue exactly when the signal is strongest.
"""

import cv2
import numpy as np


# --------------------------------------------------------------------------- #
# Roster selection / status bookkeeping
# --------------------------------------------------------------------------- #
def fresh_deployed(last_seen, now, heartbeat_fresh_sec):
    """Names whose heartbeat is fresh: (now - last_seen) <= heartbeat_fresh_sec."""
    out = []
    for name, ls in last_seen.items():
        if ls is None:
            continue
        if (now - float(ls)) <= heartbeat_fresh_sec:
            out.append(name)
    return sorted(out)


def select_targets(requested, last_seen, now, heartbeat_fresh_sec):
    """Targets to link: explicit `requested` if non-empty, else fresh-deployed."""
    if requested:
        return list(requested)
    return fresh_deployed(last_seen, now, heartbeat_fresh_sec)


def merge_link_status(prev_registered, prev_failed, targets,
                      new_registered, new_failed):
    """Merge a subset-scoped pass into the full status.

    Callsigns NOT in `targets` keep their previous outcome; callsigns in
    `targets` take the new one. This is what makes re-probing a single robot
    safe -- it cannot silently mark everyone else failed.
    """
    targets = set(targets)
    new_reg = set(new_registered)
    new_fail = set(new_failed)

    registered = {n for n in prev_registered if n not in targets} | new_reg
    failed = {n for n in prev_failed if n not in targets} | new_fail
    failed -= registered          # a freshly-linked name must not linger in failed
    return sorted(registered), sorted(failed)


def link_status_payload(registered, failed, links=None):
    """The latched ~/link_status JSON dict."""
    return {
        'complete': True,
        'registered': sorted(registered),
        'failed': sorted(failed),
        'links': dict(links or {}),
    }


# --------------------------------------------------------------------------- #
# Lit-blob detection and matching
# --------------------------------------------------------------------------- #
def lit_blobs_gray(post_gray, pre_gray, brightness_delta=40, min_area=20):
    """Blobs that got much brighter between the dark pre-frame and the lit post.

    Returns (blobs, mask) where each blob is
    {'centroid': (u, v), 'bbox': (x, y, w, h), 'area': int}.
    """
    post = np.asarray(post_gray).astype(np.int32)
    if pre_gray is None:
        mask = post > (255 - brightness_delta)
    else:
        mask = (post - np.asarray(pre_gray).astype(np.int32)) > brightness_delta
    mask = mask.astype(np.uint8) * 255
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    n, _labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    blobs = []
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        blobs.append({
            'centroid': (float(centroids[i][0]), float(centroids[i][1])),
            'bbox': (int(stats[i, cv2.CC_STAT_LEFT]),
                     int(stats[i, cv2.CC_STAT_TOP]),
                     int(stats[i, cv2.CC_STAT_WIDTH]),
                     int(stats[i, cv2.CC_STAT_HEIGHT])),
            'area': area,
        })
    return blobs, mask


def match_lit_blob(blob_centres, lit_blobs, tol_px, exclude_indices=None):
    """Match the newly-lit region to one of the known blob centres.

    `exclude_indices` are blob indices already bound to another callsign; passing
    them keeps the overall assignment a bijection, so two robots can never be
    linked to the same blob.

    Returns {'matched', 'blob_index', 'lit_index', 'distance', 'reason'} with
    reason in {'', 'no_lit', 'no_blobs', 'too_far'}.
    """
    exclude = set(exclude_indices or ())
    if not lit_blobs:
        return {'matched': False, 'blob_index': None, 'lit_index': None,
                'distance': float('inf'), 'reason': 'no_lit'}
    candidates = [(i, c) for i, c in enumerate(blob_centres) if i not in exclude]
    if not candidates:
        return {'matched': False, 'blob_index': None, 'lit_index': None,
                'distance': float('inf'), 'reason': 'no_blobs'}

    best = None
    for bi, (pu, pv) in candidates:
        for lj, lb in enumerate(lit_blobs):
            lu, lv = lb['centroid']
            dist = float(np.hypot(pu - lu, pv - lv))
            if best is None or dist < best[0]:
                best = (dist, bi, lj)

    dist, bi, lj = best
    if dist <= tol_px:
        return {'matched': True, 'blob_index': bi, 'lit_index': lj,
                'distance': dist, 'reason': ''}
    return {'matched': False, 'blob_index': bi, 'lit_index': lj,
            'distance': dist, 'reason': 'too_far'}

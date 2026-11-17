#!/usr/bin/env python3
"""Bright-blob robot detector (pure, no ROS), ported from scripts/blob_detect_timing.py.

Threshold: the arena is lit unevenly -- the floor reads ~19 in the middle and
~8 in the corners, and robots dim with it (gray p90 ~160 mid, ~55 corner), so
no single gray level works. Measured against the LOCAL floor, though, every
robot is 3.4-5.6x (blob median), the blue tape <= 2.75x and floor noise
<= 2.5x. The floor map is a morphological opening (61 px > a ~40 px robot, so
robots and tape vanish) of still frames, built once; per frame it costs one
compare against the precomputed map.

Edges: thresholding runs inside the arena polygon grown by `margin` px, and the
centroid -- not the pixel mask -- decides membership, so a robot straddling the
boundary keeps its whole blob. The centroid may sit up to `margin` px outside:
the calibrated edge runs ~a robot radius inside the wall.

A robot rolling with its dark side up fragments into pieces under `min_area`;
each piece under it joins its nearest piece within `link_px` (less than a robot).

Touching robots merge into one blob of ~2x robot area. A blob of at least 1.6x
the reference area is split into round(area / ref) parts by k-means on its pixel
coordinates. The reference is `ref_area` when given, else the frame's median
blob area (only when >= 3 blobs are found).
"""

import cv2
import numpy as np


def make_roi(corners_px, frame_shape, margin=25):
    """Crop around the arena grown by `margin` px; polygon kept crop-relative."""
    corners = np.asarray(corners_px, dtype=np.float64)
    fh, fw = frame_shape[:2]
    x0, y0, w, h = cv2.boundingRect(corners.astype(np.float32))
    x0, y0 = max(0, x0 - margin), max(0, y0 - margin)
    w, h = min(fw - x0, w + 2 * margin), min(fh - y0, h + 2 * margin)
    poly = (corners - (x0, y0)).astype(np.float32)
    mask = np.zeros((h, w), np.uint8)
    cv2.fillPoly(mask, [np.round(poly).astype(np.int32)], 255)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * margin + 1, 2 * margin + 1))
    return {'sl': (slice(y0, y0 + h), slice(x0, x0 + w)), 'mask': cv2.dilate(mask, k),
            'poly': poly, 'offset': np.array([x0, y0]), 'margin': margin}


def threshold_map(still_grays, roi, ratio=3.2):
    """Per-pixel threshold = ratio x local floor brightness, from still frames."""
    g = np.median(np.stack([f[roi['sl']] for f in still_grays]), axis=0).astype(np.uint8)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (61, 61))
    floor = cv2.morphologyEx(cv2.medianBlur(g, 5), cv2.MORPH_OPEN, k)
    floor = cv2.GaussianBlur(floor.astype(np.float32), (0, 0), 15)
    return np.clip(np.round(ratio * np.maximum(floor, 1)), 0, 255).astype(np.uint8)


def _blob(area, cx, cy, x, y, w, h, off, split=False):
    return {'Area': int(area), 'Centroid': (float(cx + off[0]), float(cy + off[1])),
            'BoundingBox': (int(x + off[0]), int(y + off[1]), int(w), int(h)),
            'Split': split}


def detect(gray, roi, tmap, min_area=200, link_px=25, ref_area=None):
    """Blobs in full-frame px: dicts of Area, Centroid, BoundingBox, Split."""
    crop = gray[roi['sl']]
    b = cv2.compare(crop, tmap, cv2.CMP_GT) & roi['mask']
    # Outer contours (8-connected, like bwconncomp(b,8)) instead of labelling the
    # whole 1.5 MP crop: 0.9 ms vs 8.5 ms. Area/centroid are then exact pixel
    # moments of each blob's own small patch, so they match regionprops.
    contours, _ = cv2.findContours(b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    parts = []
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        if w * h < 10:                                    # single-pixel noise
            continue
        mo = cv2.moments(c)
        if mo['m00'] > 0:
            parts.append((c, mo['m10'] / mo['m00'], mo['m01'] / mo['m00'], x, y, w, h))
        else:
            parts.append((c, x + w / 2, y + h / 2, x, y, w, h))
    # A robot rolling with its dark side up breaks into fragments (e.g. 100+82+41
    # px) that each fall under min_area. Each part smaller than min_area joins
    # its single nearest part within link_px (< one ~35 px robot); full-size
    # parts never initiate a join, so two robots cannot be chained together.
    group = list(range(len(parts)))

    def root(i):
        while group[i] != i:
            group[i] = group[group[i]]
            i = group[i]
        return i
    if parts:
        xy = np.array([(p[1], p[2]) for p in parts])
        d2 = ((xy[:, None] - xy[None]) ** 2).sum(axis=2)
        np.fill_diagonal(d2, np.inf)
        for i, p in enumerate(parts):
            j = int(d2[i].argmin())
            if cv2.contourArea(p[0]) < min_area and d2[i, j] < link_px ** 2:
                group[root(i)] = root(j)
    groups = {}
    for i in range(len(parts)):
        groups.setdefault(root(i), []).append(parts[i])

    found = []
    for members in groups.values():
        x = min(p[3] for p in members)
        y = min(p[4] for p in members)
        w = max(p[3] + p[5] for p in members) - x
        h = max(p[4] + p[6] for p in members) - y
        if w * h < min_area:
            continue
        m = np.zeros((h, w), np.uint8)
        cv2.drawContours(m, [p[0] - (x, y) for p in members], -1, 255, -1)
        m &= b[y:y + h, x:x + w]
        mo = cv2.moments(m, binaryImage=True)
        if mo['m00'] < min_area:
            continue
        cx, cy = x + mo['m10'] / mo['m00'], y + mo['m01'] / mo['m00']
        # centroid may sit up to `margin` outside: the calibrated edge runs about
        # a robot radius inside the wall, and a robot cannot be past the wall
        if cv2.pointPolygonTest(roi['poly'], (cx, cy), True) < -roi['margin']:
            continue
        found.append((int(mo['m00']), cx, cy, x, y, w, h, m))
    if not found:
        return []
    # touching robots merge into one ~2x blob; split those >= 1.6x the reference
    # area. Without ref_area the frame median stands in, which needs >= 3 blobs.
    can_split = ref_area is not None or len(found) >= 3
    ref = float(ref_area) if ref_area is not None else float(np.median([f[0] for f in found]))
    off, out = roi['offset'], []
    for area, cx, cy, x, y, w, h, m in found:
        k = int(round(area / ref)) if can_split and area >= 1.6 * ref else 1
        if k < 2:
            out.append(_blob(area, cx, cy, x, y, w, h, off))
            continue
        ys, xs = np.nonzero(m)
        pts = np.column_stack([xs + x, ys + y]).astype(np.float32)
        crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 20, 0.5)
        _, lab, centers = cv2.kmeans(pts, k, None, crit, 3, cv2.KMEANS_PP_CENTERS)
        for j in range(k):
            p = pts[lab.ravel() == j]
            px0, py0 = p.min(axis=0)
            px1, py1 = p.max(axis=0)
            out.append(_blob(len(p), *centers[j], px0, py0, px1 - px0 + 1, py1 - py0 + 1,
                             off, split=True))
    return out

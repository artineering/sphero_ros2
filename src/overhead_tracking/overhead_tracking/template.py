#!/usr/bin/env python3
"""Colour-invariant shape template for overhead Sphero detection (pure, no ROS).

The pattern is a circular ball with a square LED matrix in the middle and two
bright point LEDs on opposite sides. The points rotate with the robot's heading,
so matching runs over a bank of rotated templates.

Geometry measured on the overhead global-shutter camera at 1280x720:
    ball diameter        33-35 px  (BALL_D)
    matrix square        26-32 px  (SQUARE)
    point separation     25.8-25.9 px, diametrically opposed (PT_SEP)

Matching uses TM_CCOEFF_NORMED, which is contrast-invariant. That is what makes
detection work when the matrix is off -- but it also means the correlation will
happily lock onto sensor noise on a near-black arena. Measured: without a
brightness gate a 1280x720 frame yields 74 false positives whose scores reach
0.634 against a true-robot 0.668. `min_bright` is therefore load-bearing, not an
optimisation.

Two-stage structure
-------------------
`bright_components` (mask + connected components) and `match_bank_at` (rotation
bank over one small window) are the two halves of `detect_roi`, exposed
separately so the node can run the mask ONCE per frame and dispatch only the
per-candidate half to a thread pool. `detect_roi` is defined as their
composition, so the validated algorithm cannot drift from the split version --
`test_template.py` asserts the two agree exactly.
"""

import cv2
import numpy as np

BALL_D = 34
SQUARE = 28
PT_SEP = 26
PT_R = 3


def build_template(ball_d=BALL_D, square=SQUARE, pt_sep=PT_SEP, pt_r=PT_R):
    """Grayscale template. Values are relative brightness, not absolute levels --
    normalised correlation means only the pattern matters."""
    size = ball_d + 6 + (ball_d % 2 == 0)      # odd, with margin
    c = size // 2
    t = np.zeros((size, size), np.float32)
    cv2.circle(t, (c, c), ball_d // 2, 0.25, -1)                # ball body
    h = square // 2
    cv2.rectangle(t, (c - h, c - h), (c + h, c + h), 0.55, -1)   # matrix
    r = pt_sep // 2
    cv2.circle(t, (c - r, c), pt_r, 1.0, -1)                     # point LEDs
    cv2.circle(t, (c + r, c), pt_r, 1.0, -1)
    return cv2.GaussianBlur(t, (3, 3), 0)


def rotate(t, deg):
    c = t.shape[0] / 2 - 0.5
    m = cv2.getRotationMatrix2D((c, c), deg, 1.0)
    return cv2.warpAffine(t, m, t.shape[::-1], flags=cv2.INTER_LINEAR)


def bank(t, step=15):
    """Rotated copies. The two point LEDs are indistinguishable in grayscale, so
    the pattern has 180 deg symmetry and the bank only needs to cover 0-180."""
    return [(d, rotate(t, d)) for d in range(0, 180, step)]


def bright_components(gray, min_bright=60, min_area=30, dilate_half=20):
    """Candidate robots from a brightness mask.

    Returns [(cx, cy, area, (x, y, w, h))] -- centroid, pixel area and bbox of
    each connected bright region. The dilation merges a robot's matrix and its
    point LEDs into one component; without it a single robot yields three.
    """
    mask = cv2.dilate((gray >= min_bright).astype(np.uint8),
                      np.ones((dilate_half, dilate_half), np.uint8))
    n, _lbl, stats, cent = cv2.connectedComponentsWithStats(mask, 8)
    out = []
    for i in range(1, n):
        area = int(stats[i, 4])
        if area < min_area:
            continue
        out.append((int(cent[i][0]), int(cent[i][1]), area,
                    (int(stats[i, 0]), int(stats[i, 1]),
                     int(stats[i, 2]), int(stats[i, 3]))))
    return out


def bright_components_scaled(gray, min_bright=60, min_area=30, dilate_half=20,
                             scale=1):
    """`bright_components` run on a downsampled copy, centroids scaled back up.

    The candidate mask is the pipeline's bottleneck: a 20x20 dilation over a
    2.3 MP frame costs 31.6 ms, versus 0.93 ms to actually match one candidate.
    Halving the resolution first drops that to 5.2 ms and, measured on a real
    1920x1200 frame, returns byte-identical detections -- because the centroid
    only has to be good enough to centre the match window, and `pad_extra`
    absorbs the residual error.

    Do NOT compensate by widening `pad_extra` too far: at pad 12 the larger
    response map started admitting false positives (7 hits where 5 are real).
    pad 8 with scale 2 was exact.
    """
    if scale is None or scale <= 1:
        return bright_components(gray, min_bright, min_area, dilate_half)
    s = int(scale)
    small = cv2.resize(gray, None, fx=1.0 / s, fy=1.0 / s,
                       interpolation=cv2.INTER_AREA)
    comps = bright_components(small, min_bright, max(4, min_area // (s * s)),
                              max(3, dilate_half // s))
    return [(cx * s, cy * s, area * s * s,
             (bx * s, by * s, bw * s, bh * s))
            for cx, cy, area, (bx, by, bw, bh) in comps]


def match_bank_at(gray, tbank, cx, cy, pad_extra=4):
    """Best rotation-bank match in a small window around (cx, cy).

    Returns (x, y, score, angle_deg) in ABSOLUTE image coords, or None if the
    window is too small (robot at the frame border). The score is returned
    unthresholded -- callers decide, so a near-miss can still be reported as a
    diagnostic.

    Pure and side-effect free: it takes a numpy view of `gray` rather than a
    copy, and spends nearly all its time inside OpenCV with the GIL released.
    That is what makes it safe and worthwhile to run in a thread pool.
    """
    ts = tbank[0][1].shape[0]
    half = ts // 2
    pad = half + pad_extra
    h_img, w_img = gray.shape[:2]
    x0, y0 = max(0, int(cx) - pad), max(0, int(cy) - pad)
    x1, y1 = min(w_img, int(cx) + pad), min(h_img, int(cy) + pad)
    win = gray[y0:y1, x0:x1].astype(np.float32)
    if win.shape[0] < ts or win.shape[1] < ts:
        return None
    best_score, best_loc, best_ang = -1.0, None, 0.0
    for deg, t in tbank:
        r = cv2.matchTemplate(win, t, cv2.TM_CCOEFF_NORMED)
        _, mx, _, loc = cv2.minMaxLoc(r)
        if mx > best_score:
            best_score, best_loc, best_ang = mx, loc, deg
    if best_loc is None:
        return None
    return (x0 + best_loc[0] + half, y0 + best_loc[1] + half,
            float(best_score), float(best_ang))


def detect_roi(gray, tbank, thresh=0.45, min_bright=60, min_area=30,
               pad_extra=4, scale=1):
    """Candidate-limited detection: mask -> components -> bank per component.

    Measured on real frames: 16.9 ms at 1280x720 full-res, and 17.1 ms at
    1920x1200 with scale=2 + pad_extra=8 -- i.e. 2.5x the field of view for the
    same cost. `scale=1` is the exact composition of `bright_components` and
    `match_bank_at`, which `test_template.py` asserts.
    """
    ts = tbank[0][1].shape[0]
    hits = []
    for cx, cy, _area, _bbox in bright_components_scaled(
            gray, min_bright, min_area, ts // 2, scale):
        m = match_bank_at(gray, tbank, cx, cy, pad_extra)
        if m is not None and m[2] >= thresh:
            hits.append(m)
    return hits


def detect(gray, tbank, thresh=0.45, min_sep=20, min_bright=60):
    """Full-frame reference implementation. Correct but slow (~786 ms at 720p).

    Kept because it is the ground truth `detect_roi` was validated against, and
    because it is the right tool for a one-shot offline sweep where latency does
    not matter. Do not use it in the tracking loop.
    """
    best = None
    ang = None
    for deg, t in tbank:
        r = cv2.matchTemplate(gray.astype(np.float32), t, cv2.TM_CCOEFF_NORMED)
        if best is None:
            best = r.copy()
            ang = np.full(r.shape, deg, np.float32)
        else:
            m = r > best
            best[m] = r[m]
            ang[m] = deg
    off = tbank[0][1].shape[0] // 2
    bright = cv2.dilate((gray >= min_bright).astype(np.uint8),
                        np.ones((off * 2 + 1,) * 2, np.uint8))
    best[bright[off:off + best.shape[0], off:off + best.shape[1]] == 0] = -1.0
    hits = []
    work = best.copy()
    while True:
        _, mx, _, loc = cv2.minMaxLoc(work)
        if mx < thresh:
            break
        hits.append((loc[0] + off, loc[1] + off, float(mx),
                     float(ang[loc[1], loc[0]])))
        cv2.circle(work, loc, min_sep, 0.0, -1)
    return hits

#!/usr/bin/env python3
"""Arena boundary detection and polygon tests (pure, no ROS).

Ported from the Kinect field-rectangle detector and generalised to accept a
grayscale frame (the overhead camera is used as mono).

A caution specific to this rig: with the exposure tuned so the matte-black arena
reads p50 2-4, there is essentially no edge signal to find. Auto-detection is a
convenience for a brightly-lit calibration capture, NOT the primary path --
`arena_source: manual` with operator-supplied corner pixels is the default and
the one that reliably works. Callers that want auto-detection should raise the
exposure for the capture and restore it afterwards.
"""

import cv2
import numpy as np


def _as_bgr(img):
    """Accept gray or 3-channel; return (gray, bgr_for_debug)."""
    a = np.asarray(img)
    if a.ndim == 2:
        return a, cv2.cvtColor(a, cv2.COLOR_GRAY2BGR)
    if a.ndim == 3 and a.shape[2] == 3:
        return cv2.cvtColor(a, cv2.COLOR_BGR2GRAY), a.copy()
    raise ValueError(f'unsupported image shape {a.shape}')


def detect_arena_quad(img, approx_eps_frac=0.02, canny_sigma=0.33,
                      close_iters=3, border_margin=6, min_area_frac=0.05,
                      max_area_frac=0.95):
    """Largest convex 4-vertex quad in the image.

    The boundary is often COLOURED tape on a near-neutral floor: a strong colour
    edge but a weak brightness edge. When a 3-channel image is supplied we OR
    auto-thresholded Canny over luminance AND the two LAB chroma channels, so a
    tape edge of any hue is caught without thresholding a specific colour. For a
    grayscale frame only the luminance channel exists.

    Thin tape gaps are bridged by a morphological CLOSE; the image border is
    suppressed so walls and baseboards touching the frame edge do not connect
    into the contour; then the largest contour's CONVEX HULL is reduced to 4
    vertices (the hull ignores interior floor texture and tolerates small gaps).

    `max_area_frac` rejects a quad that is essentially the whole frame. This is
    not hypothetical: on a dark noisy image Canny fires everywhere, the CLOSE
    merges it into one blob, and its external contour is the rectangle just
    inside the suppressed border -- yielding a confident, completely bogus
    full-frame "arena". Failing is far better than handing the operator a
    homography built from the image edges.

    Returns (corners_px | None, diag) where diag carries the debug images and the
    counts a failed calibration message should report.
    """
    gray, vis = _as_bgr(img)
    h, w = gray.shape[:2]
    img_area = float(w * h)

    channels = [gray]
    a = np.asarray(img)
    if a.ndim == 3 and a.shape[2] == 3:
        lab = cv2.cvtColor(a, cv2.COLOR_BGR2LAB)
        channels += [lab[:, :, 1], lab[:, :, 2]]

    edges = np.zeros((h, w), np.uint8)
    for ch in channels:
        chb = cv2.GaussianBlur(ch, (5, 5), 0)
        v = float(np.median(chb))
        lo = int(max(0, (1.0 - canny_sigma) * v))
        hi = int(min(255, (1.0 + canny_sigma) * v))
        edges = cv2.bitwise_or(edges, cv2.Canny(chb, lo, hi))

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, k, iterations=int(close_iters))
    if border_margin > 0:
        b = int(border_margin)
        edges[:b, :] = 0
        edges[-b:, :] = 0
        edges[:, :b] = 0
        edges[:, -b:] = 0

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(vis, contours, -1, (0, 255, 255), 1)      # all contours

    best, best_area, hull_verts, rejected = None, 0.0, 0, ''
    if contours:
        big = max(contours, key=cv2.contourArea)
        hull = cv2.convexHull(big)
        cv2.polylines(vis, [hull], True, (255, 0, 0), 2)       # largest hull
        peri = cv2.arcLength(hull, True)
        for epsf in (approx_eps_frac, 0.03, 0.04, 0.05, 0.06, 0.08):
            approx = cv2.approxPolyDP(hull, epsf * peri, True)
            if epsf == approx_eps_frac:
                hull_verts = len(approx)
            if not (len(approx) == 4 and cv2.isContourConvex(approx)):
                continue
            frac = cv2.contourArea(approx) / img_area
            if frac < min_area_frac:
                rejected = f'quad too small ({frac:.2f} < {min_area_frac})'
                continue
            if frac > max_area_frac:
                # the whole-frame artefact -- see the docstring
                rejected = (f'quad fills the frame ({frac:.2f} > {max_area_frac}); '
                            'likely image-border edges, not an arena')
                continue
            best = approx
            best_area = cv2.contourArea(approx)
            rejected = ''
            break
    if best is not None:
        cv2.polylines(vis, [best], True, (0, 0, 255), 3)       # chosen quad

    diag = {
        'edges': edges,
        'num_contours': len(contours),
        'num_quads': 1 if best is not None else 0,
        'largest_quad_area': float(best_area),
        'hull_verts': int(hull_verts),
        'rejected': rejected,
        'contours_img': vis,
    }
    corners = best.reshape(4, 2).astype(float) if best is not None else None
    return corners, diag


def arena_mask_px(corners_px, shape):
    """uint8 mask (255 inside the arena quad) for an image of the given shape."""
    h, w = shape[:2]
    mask = np.zeros((h, w), np.uint8)
    pts = np.asarray(corners_px, dtype=np.int32).reshape(-1, 1, 2)
    cv2.fillConvexPoly(mask, pts, 255)
    return mask


def contains_px(corners_px, u, v, margin_px=0.0):
    """Is pixel (u, v) inside the arena quad, expanded by margin_px?"""
    pts = np.asarray(corners_px, dtype=np.float32).reshape(-1, 2)
    d = cv2.pointPolygonTest(pts, (float(u), float(v)), True)
    return d >= -float(margin_px)


def contains_cm(poly_cm, x_cm, y_cm, margin_cm=0.0):
    """Is field point (x_cm, y_cm) inside the arena polygon, expanded by margin?

    `cv2.pointPolygonTest` returns a signed distance, so the margin is a true
    metric dilation of the polygon rather than a bounding-box slop.
    """
    pts = np.asarray(poly_cm, dtype=np.float32).reshape(-1, 2)
    d = cv2.pointPolygonTest(pts, (float(x_cm), float(y_cm)), True)
    return d >= -float(margin_cm)

#!/usr/bin/env python3
"""Arena boundary detection and polygon tests (pure, no ROS).

Ported from the Kinect field-rectangle detector. Two paths, tried in order:

1. HUE THRESHOLD on the boundary tape (needs a colour frame). The tape is a
   different hue from everything else in the room, so it thresholds into a
   filled closed ring whose outer contour and hole are its two edges; the
   corners returned are the tape CENTRELINE.
2. CANNY EDGES, the original path, used for a mono frame or if the hue mask
   fails.

Why the order: with the exposure tuned so the matte-black arena reads p50 2-4
there is almost no brightness edge to find, and worse, the mat's edges merge
with room structure running off-frame -- border suppression then cuts that
junction and leaves the mat as an OPEN chain, whose contourArea is its stroke
rather than its area, so it loses `max(contours, key=contourArea)` to any small
closed object in the room. Thresholding sidesteps both problems: hue ignores
brightness, and a filled region has a real area.
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



def _quad_from_contour(c, approx_eps_frac):
    """Convex hull reduced to exactly 4 convex vertices, or None."""
    hull = cv2.convexHull(c)
    peri = cv2.arcLength(hull, True)
    for epsf in (approx_eps_frac, 0.03, 0.04, 0.05, 0.06, 0.08):
        approx = cv2.approxPolyDP(hull, epsf * peri, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            return approx.reshape(4, 2).astype(float)
    return None


def _tape_centreline_quad(bgr, hue_lo, hue_hi, approx_eps_frac,
                          min_area_frac, max_area_frac):
    """Corners on the CENTRELINE of the boundary tape, from a hue mask.

    Measured on this rig: tape H 102-112, mat 11-14, floor/wall 20-22 (OpenCV's
    0-179 scale), so a hue band isolates the tape by ~80 units where no
    brightness or chroma EDGE separates it at all -- the mat is matte black and
    the tape is a weak luminance edge on it.

    Thresholding also fixes a topology problem the edge path cannot: the tape
    becomes a FILLED closed ring, so `contourArea` is its real area instead of
    the area of an open stroke. The ring's outer contour and its largest hole
    are the two edges of the tape, and matched corners averaged land on the
    middle of the stroke.

    Returns (corners|None, diag, reason).
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv[:, :, 0], int(hue_lo), int(hue_hi))
    cs, hier = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    diag = {'mask': mask, 'num_contours': len(cs)}
    if not cs:
        return None, diag, f'hue mask [{hue_lo},{hue_hi}] is empty'
    hier = hier[0]
    tops = [i for i in range(len(cs)) if hier[i][3] == -1]
    oi = max(tops, key=lambda i: cv2.contourArea(cv2.convexHull(cs[i])))
    outer = _quad_from_contour(cs[oi], approx_eps_frac)
    if outer is None:
        return None, diag, 'largest hue blob is not a 4-vertex convex quad'

    img_area = float(mask.shape[0] * mask.shape[1])
    frac = cv2.contourArea(outer.astype(np.float32)) / img_area
    if frac < min_area_frac:
        return None, diag, f'hue quad too small ({frac:.2f} < {min_area_frac})'
    if frac > max_area_frac:
        return None, diag, f'hue quad fills the frame ({frac:.2f} > {max_area_frac})'

    kids = [i for i in range(len(cs)) if hier[i][3] == oi]
    inner = (_quad_from_contour(cs[max(kids, key=lambda i: cv2.contourArea(cs[i]))],
                                approx_eps_frac) if kids else None)
    if inner is None:
        # No usable hole: the corners are the tape's OUTER edge, half a stroke
        # width out from the centreline. Usable, but say so.
        return outer, diag, 'no inner tape edge; corners are the OUTER edge'

    # Pair each outer corner with its nearest unused inner corner. Winding is
    # normally already aligned; matching makes it so regardless.
    used, mid = set(), []
    for p in outer:
        j = min((k for k in range(4) if k not in used),
                key=lambda k: float(np.hypot(*(inner[k] - p))))
        used.add(j)
        mid.append((p + inner[j]) / 2.0)
    return np.asarray(mid, dtype=float), diag, ''


def detect_arena_quad(img, approx_eps_frac=0.02, canny_sigma=0.33,
                      close_iters=3, border_margin=6, min_area_frac=0.05,
                      max_area_frac=0.95, hue_lo=90, hue_hi=130):
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

    # ---- primary: hue threshold on the boundary tape (colour frames only).
    # Falls through to the Canny path below on a mono frame or any failure.
    a0 = np.asarray(img)
    if a0.ndim == 3 and a0.shape[2] == 3:
        quad, hdiag, why = _tape_centreline_quad(
            a0, hue_lo, hue_hi, approx_eps_frac, min_area_frac, max_area_frac)
        if quad is not None:
            cv2.polylines(vis, [quad.astype(np.int32)], True, (0, 0, 255), 3)
            return quad, {
                'edges': hdiag['mask'],
                'num_contours': hdiag['num_contours'],
                'num_quads': 1,
                'largest_quad_area': float(cv2.contourArea(
                    quad.astype(np.float32))),
                'hull_verts': 4,
                'rejected': why,          # '' unless the outer edge was used
                'contours_img': vis,
                'source': 'hue',
            }
        hue_rejected = f'hue: {why}'
    else:
        hue_rejected = 'hue: frame is mono'

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

    if rejected:
        rejected = f'{hue_rejected}; canny: {rejected}'
    else:
        rejected = hue_rejected if best is None else ''
    diag = {
        'source': 'canny',
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

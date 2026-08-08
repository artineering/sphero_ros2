#!/usr/bin/env python3
"""Planar homography: image pixels <-> field centimetres (pure, no ROS).

The Spheros roll on a flat floor, so a single 3x3 homography from the four arena
corner pixels to the measured arena rectangle is EXACT for an ideal pinhole
camera and additionally absorbs modest lens distortion into the fit. No depth, no
plane RANSAC, no camera intrinsics.

What it cannot do: a homography is a global perspective map and cannot model true
radial (barrel) distortion, so a wide-FOV lens can leave a residual that grows
toward the frame corners. `residual_cm` reports the corner fit error; if the
bench pass shows it exceeding tolerance, the fix is `cv2.undistort` before the
homography, not a different mapping.

Field-frame convention (must match what the control stack already expects)
--------------------------------------------------------------------------
    origin = the arena corner that is BOTTOM-LEFT in image space
    +x     = along the LONG edge      (long_edge_cm)
    +y     = along the SHORT edge     (short_edge_cm)
    units  = centimetres

Corner ADJACENCY comes from ring topology (angle about the centroid), not pixel
distance. Under a tilted camera perspective compresses the far edge, so the
diagonal corner can be *closer* in pixels than a true edge-adjacent one --
picking neighbours by pixel distance is simply wrong. Ring order is projective
and therefore survives the perspective.

Which edge is "long" IS decided by pixel length, because in image space that is
all we have. Perspective can invert it; `swap_axes` is the escape hatch and the
caller should report the choice so an operator can see it.
"""

import cv2
import numpy as np


def select_bottom_left(corners_px):
    """Index of the bottom-left corner in IMAGE space.

    Bottom = largest pixel v (image y grows downward); among the two bottom-most
    corners, left = smallest pixel u.
    """
    c = np.asarray(corners_px, dtype=float).reshape(-1, 2)
    order_v = np.argsort(c[:, 1])
    i0, i1 = order_v[-2:]
    return int(i0) if c[i0, 0] <= c[i1, 0] else int(i1)


def cyclic_order(corners_px):
    """Indices of the 4 corner pixels in ring order around their centroid."""
    c = np.asarray(corners_px, dtype=float).reshape(-1, 2)
    rel = c - c.mean(axis=0)
    angles = np.arctan2(rel[:, 1], rel[:, 0])
    return [int(i) for i in np.argsort(angles)]


def field_corners_from_px(corners_px, long_cm, short_cm, swap_axes=False):
    """Field-cm coordinates for each corner pixel, index-aligned to `corners_px`.

    Returns (4, 2) float array and a dict describing the choices made, so the
    caller can surface them:
        {'origin_idx', 'long_idx', 'short_idx', 'diag_idx',
         'long_edge_px', 'short_edge_px', 'swapped'}
    """
    c = np.asarray(corners_px, dtype=float).reshape(-1, 2)
    if c.shape[0] != 4:
        raise ValueError(f'need exactly 4 corners, got {c.shape[0]}')
    if long_cm <= 0 or short_cm <= 0:
        raise ValueError('long_cm and short_cm must be positive')

    if len({(round(x, 6), round(y, 6)) for x, y in c}) != 4:
        raise ValueError('degenerate corner set (duplicate points)')

    o = select_bottom_left(c)
    ring = cyclic_order(c)
    pos = ring.index(o)
    a = ring[(pos - 1) % 4]
    b = ring[(pos + 1) % 4]
    diag = ring[(pos + 2) % 4]

    len_a = float(np.linalg.norm(c[a] - c[o]))
    len_b = float(np.linalg.norm(c[b] - c[o]))
    if len_a < 1e-9 or len_b < 1e-9:
        raise ValueError('degenerate quad (zero-length edge)')
    long_idx, short_idx = (a, b) if len_a >= len_b else (b, a)
    if swap_axes:
        long_idx, short_idx = short_idx, long_idx

    field = np.zeros((4, 2), dtype=float)
    field[o] = (0.0, 0.0)
    field[long_idx] = (long_cm, 0.0)
    field[short_idx] = (0.0, short_cm)
    field[diag] = (long_cm, short_cm)

    info = {
        'origin_idx': int(o),
        'long_idx': int(long_idx),
        'short_idx': int(short_idx),
        'diag_idx': int(diag),
        'long_edge_px': max(len_a, len_b),
        'short_edge_px': min(len_a, len_b),
        'swapped': bool(swap_axes),
    }
    return field, info


def build_homography(corners_px, corners_cm):
    """3x3 H mapping image pixels -> field cm."""
    src = np.asarray(corners_px, dtype=np.float32).reshape(-1, 2)
    dst = np.asarray(corners_cm, dtype=np.float32).reshape(-1, 2)
    if src.shape[0] != dst.shape[0]:
        raise ValueError('corner count mismatch')
    if src.shape[0] == 4:
        H = cv2.getPerspectiveTransform(src, dst)
    else:
        H, _ = cv2.findHomography(src, dst, cv2.RANSAC)
        if H is None:
            raise ValueError('findHomography failed')
    return np.asarray(H, dtype=float)


def invert_homography(H):
    return np.linalg.inv(np.asarray(H, dtype=float))


def apply_homography(u, v, H):
    """Pixel -> field cm."""
    H = np.asarray(H, dtype=float)
    p = H @ np.array([float(u), float(v), 1.0])
    w = p[2]
    if abs(w) < 1e-12:
        raise ValueError('homography sent the point to infinity')
    return float(p[0] / w), float(p[1] / w)


def apply_homography_many(uv, H):
    """(N,2) pixels -> (N,2) field cm."""
    pts = np.asarray(uv, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return np.empty((0, 2))
    ones = np.ones((len(pts), 1))
    hom = np.hstack([pts, ones]) @ np.asarray(H, dtype=float).T
    w = hom[:, 2:3]
    w[np.abs(w) < 1e-12] = 1e-12
    return hom[:, :2] / w


def field_cm_to_px(x_cm, y_cm, H_inv):
    """Field cm -> pixel. Pass the INVERSE homography."""
    return apply_homography(x_cm, y_cm, H_inv)


def px_per_cm_at(u, v, H):
    """Local scale (pixels per cm) at a pixel location.

    A homography is not a uniform scaling -- px/cm varies across the frame under
    perspective. This is the local Jacobian, used to convert a cm-space velocity
    into the pixel-space ROI growth term. Area-based so it is isotropic.
    """
    p0 = np.array(apply_homography(u, v, H))
    p1 = np.array(apply_homography(u + 1.0, v, H))
    p2 = np.array(apply_homography(u, v + 1.0, H))
    d1, d2 = p1 - p0, p2 - p0
    area_cm2_per_px2 = abs(d1[0] * d2[1] - d1[1] * d2[0])
    if area_cm2_per_px2 < 1e-12:
        raise ValueError('degenerate homography scale')
    return float(1.0 / np.sqrt(area_cm2_per_px2))


def residual_cm(corners_px, corners_cm, H):
    """Corner reprojection error in cm: (max, mean). The homography-quality check."""
    mapped = apply_homography_many(corners_px, H)
    target = np.asarray(corners_cm, dtype=float).reshape(-1, 2)
    err = np.linalg.norm(mapped - target, axis=1)
    return float(err.max()), float(err.mean())

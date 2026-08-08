#!/usr/bin/env python3
"""Pixel <-> field-cm mapping and the corner convention."""

import numpy as np
import pytest

from overhead_tracking import homography as H

# a perspective-distorted quad: bottom edge wider than top (camera looking down
# and forward), so pixel distance and metric distance genuinely disagree
QUAD = [[100.0, 600.0], [900.0, 640.0], [950.0, 120.0], [80.0, 100.0]]
LONG, SHORT = 200.0, 120.0


def build():
    cm, info = H.field_corners_from_px(QUAD, LONG, SHORT)
    return H.build_homography(QUAD, cm), cm, info


def test_corner_roundtrip_is_exact():
    h, cm, _ = build()
    mx, mean = H.residual_cm(QUAD, cm, h)
    assert mx < 1e-6 and mean < 1e-6


def test_origin_is_bottom_left_in_image_space():
    _h, cm, info = build()
    o = info['origin_idx']
    assert tuple(cm[o]) == (0.0, 0.0)
    # bottom-left = largest v, then smallest u
    pts = np.asarray(QUAD)
    bottom_two = np.argsort(pts[:, 1])[-2:]
    assert o in bottom_two
    assert pts[o, 0] == min(pts[i, 0] for i in bottom_two)


def test_axes_follow_long_and_short_edges():
    _h, cm, info = build()
    assert tuple(cm[info['long_idx']]) == (LONG, 0.0)
    assert tuple(cm[info['short_idx']]) == (0.0, SHORT)
    assert tuple(cm[info['diag_idx']]) == (LONG, SHORT)


def test_swap_axes_exchanges_the_assignment():
    _cm_a, info_a = H.field_corners_from_px(QUAD, LONG, SHORT)
    _cm_b, info_b = H.field_corners_from_px(QUAD, LONG, SHORT, swap_axes=True)
    assert info_a['long_idx'] == info_b['short_idx']
    assert info_a['short_idx'] == info_b['long_idx']


def test_inverse_maps_back_to_the_same_pixel():
    h, _cm, _ = build()
    hi = H.invert_homography(h)
    for u, v in [(640, 360), (200, 550), (880, 180)]:
        x, y = H.apply_homography(u, v, h)
        bu, bv = H.field_cm_to_px(x, y, hi)
        assert abs(bu - u) < 1e-6 and abs(bv - v) < 1e-6


def test_interior_pixel_maps_inside_the_rectangle():
    h, _cm, _ = build()
    x, y = H.apply_homography(500, 380, h)
    assert 0.0 < x < LONG and 0.0 < y < SHORT


def test_apply_many_matches_scalar_version():
    h, _cm, _ = build()
    pts = [(300, 300), (700, 500), (450, 200)]
    many = H.apply_homography_many(pts, h)
    for (u, v), got in zip(pts, many):
        assert np.allclose(got, H.apply_homography(u, v, h))


def test_px_per_cm_matches_finite_difference():
    """Local scale is used to size the ROI growth term from a cm velocity."""
    h, _cm, _ = build()
    u, v = 640.0, 360.0
    got = H.px_per_cm_at(u, v, h)
    p0 = np.array(H.apply_homography(u, v, h))
    p1 = np.array(H.apply_homography(u + 10.0, v, h))
    fd_px_per_cm = 10.0 / np.linalg.norm(p1 - p0)
    assert abs(got - fd_px_per_cm) / fd_px_per_cm < 0.05


def test_px_per_cm_varies_across_a_perspective_frame():
    """If this were constant the local Jacobian would be pointless."""
    h, _cm, _ = build()
    near = H.px_per_cm_at(500, 600, h)
    far = H.px_per_cm_at(500, 130, h)
    assert abs(near - far) / max(near, far) > 0.01


def test_degenerate_inputs_raise():
    with pytest.raises(ValueError):
        H.field_corners_from_px([[0, 0], [1, 1], [2, 2]], LONG, SHORT)
    with pytest.raises(ValueError):
        H.field_corners_from_px(QUAD, 0.0, SHORT)
    with pytest.raises(ValueError):
        H.field_corners_from_px([[0, 0], [0, 0], [1, 1], [2, 2]], LONG, SHORT)

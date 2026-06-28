"""Unit tests for pure field-calibration geometry (no hardware)."""

import os

import numpy as np
import pytest

from kinect_field_tracking.detection import Intrinsics, project
from kinect_field_tracking import geometry as g


INTR = Intrinsics()


def _plane_points(z=2000.0, n=400, noise=2.0, outliers=20, seed=0):
    rng = np.random.default_rng(seed)
    xs = rng.uniform(-600, 600, n)
    ys = rng.uniform(-400, 400, n)
    zs = np.full(n, z) + rng.normal(0, noise, n)
    pts = np.column_stack([xs, ys, zs])
    # add gross outliers well off the plane
    out = np.column_stack([
        rng.uniform(-600, 600, outliers),
        rng.uniform(-400, 400, outliers),
        rng.uniform(500, 1500, outliers),
    ])
    return np.vstack([pts, out])


def test_plane_fit_recovers_flat_plane():
    pts = _plane_points(z=2000.0)
    n, d, inliers = g.fit_plane_ransac(pts, thresh_mm=15.0, seed=1)
    n, d = g.orient_normal_toward_camera(n, d)
    # normal is ~ +/- z
    assert abs(abs(n[2]) - 1.0) < 1e-2
    # on-plane points evaluate ~0
    on_plane = pts[inliers]
    resid = np.abs(on_plane @ n + d)
    assert resid.mean() < 10.0
    # oriented toward camera => n_z negative (ground at +z forward)
    assert n[2] < 0


def _rect_corners_cam():
    # long edge 1200mm (camera x), short 800mm (camera y), on plane z=2000
    return np.array([
        [-600.0, -400.0, 2000.0],   # C0
        [600.0, -400.0, 2000.0],    # C1
        [600.0, 400.0, 2000.0],     # C2
        [-600.0, 400.0, 2000.0],    # C3
    ])


def test_corner_backprojection_roundtrip():
    corners = _rect_corners_cam()
    n = np.array([0.0, 0.0, -1.0])
    d = 2000.0
    px = np.array([project(c[0], c[1], c[2], INTR) for c in corners])
    for i, (u, v) in enumerate(px):
        rec = g.backproject_corner(u, v, n, d, INTR)
        assert np.allclose(rec, corners[i], atol=1e-3)


def test_field_frame_construction():
    corners = _rect_corners_cam()
    n = np.array([0.0, 0.0, -1.0])
    px = np.array([project(c[0], c[1], c[2], INTR) for c in corners])
    res = g.build_field_frame(corners, px, n)

    # bottom-left in image space is C3 (largest v, smallest u of the bottom two)
    assert res['origin_idx'] == 3
    # origin maps to (0,0)
    fc = np.asarray(res['corners_field_cm'])
    assert np.allclose(fc[res['origin_idx']], [0.0, 0.0], atol=1e-6)
    # long edge along +x = 120cm, short edge along +y = 80cm
    assert abs(res['long_edge_len_cm'] - 120.0) < 1e-3
    assert abs(res['short_edge_len_cm'] - 80.0) < 1e-3
    # right-handed rotation
    assert abs(np.linalg.det(res['R_fc']) - 1.0) < 1e-6
    # all corners have non-negative field coords (origin is the corner)
    assert (fc[:, 0] >= -1e-6).all() and (fc[:, 1] >= -1e-6).all()


def test_adjacent_corners_perspective_invariant_diagonal_closer_in_pixels():
    # Regression: under a TILTED camera, perspective compresses the far edge so
    # the DIAGONAL corner can be closer in pixels than a true edge-adjacent
    # corner. Adjacency must come from rectangle topology, not pixel distance.
    L, Wd = 1500.0, 1300.0  # long, short edges (mm)
    up = np.array([0.0, -0.65, -0.76])
    up /= np.linalg.norm(up)
    ref = np.array([1.0, 0.0, 0.0])
    x_hat = ref - (ref @ up) * up
    x_hat /= np.linalg.norm(x_hat)
    y_hat = np.cross(up, x_hat)
    y_hat /= np.linalg.norm(y_hat)
    O = np.array([-700.0, 700.0, 1300.0])
    corners = np.array([O, O + L * x_hat, O + L * x_hat + Wd * y_hat, O + Wd * y_hat])
    px = np.array([project(c[0], c[1], c[2], INTR) for c in corners])

    o_idx = g.select_bottom_left(px)
    others = [i for i in range(4) if i != o_idx]
    pdist = {i: float(np.linalg.norm(px[i] - px[o_idx])) for i in others}
    mdist = {i: float(np.linalg.norm(corners[i] - corners[o_idx])) for i in others}
    diag_idx = max(mdist, key=mdist.get)
    pixel_nearest_two = sorted(others, key=lambda i: pdist[i])[:2]
    # the bug precondition: old pixel-distance logic would treat the diagonal as
    # edge-adjacent
    assert diag_idx in pixel_nearest_two

    # topology-based adjacency must EXCLUDE the diagonal
    adj = set(g._adjacent_corners(corners, up, o_idx))
    assert diag_idx not in adj

    # build_field_frame recovers correct long/short, right angles, right-handed
    res = g.build_field_frame(corners, px, up)
    assert abs(res['long_edge_len_cm'] - 150.0) < 0.5
    assert abs(res['short_edge_len_cm'] - 130.0) < 0.5
    assert abs(np.linalg.det(res['R_fc']) - 1.0) < 1e-6

    fc = np.asarray(res['corners_field_cm'])
    oi = res['origin_idx']
    assert np.allclose(fc[oi], [0.0, 0.0], atol=1e-6)
    # the two edges from the origin are orthogonal (axis-aligned rectangle)
    edges = sorted((fc[i] - fc[oi] for i in range(4) if i != oi),
                   key=np.linalg.norm)
    cos = np.dot(edges[0], edges[1]) / (np.linalg.norm(edges[0]) * np.linalg.norm(edges[1]))
    assert abs(np.degrees(np.arccos(cos)) - 90.0) < 0.5


RGB_INTR = Intrinsics(fx=525.0, fy=525.0, cx=319.5, cy=239.5)


def test_rgb_intrinsics_give_larger_field_than_depth_intrinsics():
    # Corners are detected in the RGB image; back-projecting them with the RGB
    # intrinsics (fx~525) instead of the depth intrinsics (fx~594) scales the
    # field up by ~594/525. Synthetic, hardware-independent.
    n = np.array([0.0, 0.0, -1.0])
    d = 2000.0
    corners = _rect_corners_cam()  # placed at z=2000 on a flat plane
    px = np.array([project(c[0], c[1], c[2], INTR) for c in corners])
    res_depth = g.build_field_frame(
        np.array([g.backproject_corner(u, v, n, d, INTR) for (u, v) in px]), px, n)
    res_rgb = g.build_field_frame(
        np.array([g.backproject_corner(u, v, n, d, RGB_INTR) for (u, v) in px]), px, n)
    ratio = res_rgb['long_edge_len_cm'] / res_depth['long_edge_len_cm']
    assert abs(ratio - (594.21 / 525.0)) < 0.02
    assert res_rgb['long_edge_len_cm'] > res_depth['long_edge_len_cm']
    assert abs(np.linalg.det(res_rgb['R_fc']) - 1.0) < 1e-6


def test_real_calibration_corners_recover_field_with_rgb_intrinsics_if_present():
    # Host-dependent: if the device's live calibration YAML exists, re-back-project
    # its RGB corner pixels with the RGB intrinsics onto the saved depth-fitted
    # plane and assert the interim-corrected field (~166 x ~147 cm). Skips on hosts
    # without the file (it is not committed to the repo). NOTE: ~166/147 vs the
    # phone-measured 169x138 is the deferred registration work, not this fix.
    import yaml
    path = os.path.expanduser('~/kinect_field/kinect_field.yaml')
    if not os.path.exists(path):
        pytest.skip('no live calibration YAML on this host')
    fc = yaml.safe_load(open(path))['field_calibration']
    corners_px = np.array(fc['corners_px'])
    n = np.array(fc['plane']['n'])
    d = float(fc['plane']['d'])
    corners_cam = np.array(
        [g.backproject_corner(u, v, n, d, RGB_INTR) for (u, v) in corners_px])
    res = g.build_field_frame(corners_cam, corners_px, n)
    assert abs(res['long_edge_len_cm'] - 165.9) < 2.0
    assert abs(res['short_edge_len_cm'] - 147.4) < 2.0
    assert res['long_edge_len_cm'] < 185.0  # definitively not the diagonal


def test_contact_point_lands_on_ground():
    n = np.array([0.0, 0.0, -1.0])
    d = 2000.0
    r = 36.5
    # ball on the ground: centre z = 2000 - r, top seen by camera at z = 2000 - 2r
    z_top = 2000.0 - 2 * r
    contact = g.contact_point_cam_mm(INTR.cx, INTR.cy, z_top, INTR, n, d, r)
    # contact lies on the plane
    assert abs(float(n @ contact + d)) < 1e-6
    # straight-down ray => contact at the optical axis on the ground
    assert np.allclose(contact, [0.0, 0.0, 2000.0], atol=1e-6)

"""Hardware-free node smoke test: construct in sim, inject calibration + a
locked tracker, run one tracking tick, assert a field pose is produced.

Registration's camera-confirmed LED probe cannot run in pure sim (no device
controller bridges the LED topic to the SimSource), so this test injects a
tracker directly to exercise the 10 Hz fusion + publish path. The registration
LOGIC itself is covered by test_registration_logic.py.
"""

import cv2
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from kinect_field_tracking.field_tracker_node import (
    FieldTrackerNode,
    TRACKING,
    detect_field_rectangle,
    build_track_markers,
)
from kinect_field_tracking.calibration import CalibrationResult
from kinect_field_tracking.fusion import FusionKF
from visualization_msgs.msg import Marker


def test_build_track_markers_metres_stable_ids_lifetime():
    from builtin_interfaces.msg import Time
    stamp = Time()
    positions_cm = {'SB-AAAD': (150.0, 80.0), 'SB-1FA8': (-50.0, 200.0)}
    id_map = {}
    arr = build_track_markers(positions_cm, sphere_radius_mm=36.5, stamp=stamp,
                              id_map=id_map, lifetime_s=0.5)
    # one SPHERE + one TEXT per tracker
    assert len(arr.markers) == 4
    spheres = [m for m in arr.markers if m.type == Marker.SPHERE]
    texts = [m for m in arr.markers if m.type == Marker.TEXT_VIEW_FACING]
    assert len(spheres) == 2 and len(texts) == 2

    by_text = {m.text: m for m in texts}
    sph0 = spheres[0]
    assert sph0.header.frame_id == 'field'
    # positions are METRES (cm/100), NOT cm
    name0 = 'SB-AAAD'
    s0 = next(m for m in spheres if abs(m.pose.position.x - 1.50) < 1e-6)
    assert abs(s0.pose.position.x - 1.50) < 1e-6   # 150cm -> 1.5m
    assert abs(s0.pose.position.y - 0.80) < 1e-6
    assert abs(s0.pose.position.z - 0.0365) < 1e-6  # rests on ground at radius
    # scale = ball diameter
    assert abs(s0.scale.x - 0.073) < 1e-6
    # text label present + above the sphere
    assert name0 in by_text
    assert by_text[name0].pose.position.z > 0.073
    # lifetime set (~0.5s)
    assert sph0.lifetime.sec == 0 and sph0.lifetime.nanosec == 500000000

    # stable ids across calls (same id_map) -> same ids, no accumulation
    ids_first = sorted(m.id for m in arr.markers)
    arr2 = build_track_markers(positions_cm, 36.5, stamp, id_map, lifetime_s=0.5)
    assert sorted(m.id for m in arr2.markers) == ids_first


def test_detect_field_rectangle_finds_quad_and_diag():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(img, (100, 80), (540, 400), (255, 255, 255), -1)
    corners, diag = detect_field_rectangle(img, approx_eps_frac=0.02)
    assert corners is not None and corners.shape == (4, 2)
    assert diag['num_quads'] >= 1
    assert diag['largest_quad_area'] > 0
    assert diag['edges'].shape == (480, 640)
    assert diag['contours_img'].shape == (480, 640, 3)


def test_detect_field_rectangle_none_on_blank():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    corners, diag = detect_field_rectangle(img, approx_eps_frac=0.02)
    assert corners is None
    assert diag['num_contours'] == 0


def test_detect_field_rectangle_colored_tape_on_textured_floor():
    # Regression for the real failure: a COLOURED (blue) boundary on a brown,
    # textured floor -- weak in luminance, strong in chroma. Grayscale-only Canny
    # missed it; the multi-channel detector must find it.
    rng = np.random.default_rng(0)
    # brown wood-ish floor with grain noise
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:] = (140, 90, 50)  # RGB brown
    img += rng.integers(-15, 15, img.shape, dtype=np.int16).clip(-40, 40).astype(np.int16).astype(np.uint8)
    # blue tape rectangle outline (thin), similar luminance to wood
    cv2.rectangle(img, (90, 70), (520, 410), (30, 60, 200), 8)  # RGB blue
    corners, diag = detect_field_rectangle(img, approx_eps_frac=0.02)
    assert corners is not None, f"missed coloured boundary (diag={diag['num_contours']} contours)"
    frac = cv2.contourArea(corners.astype(np.int32)) / (640.0 * 480.0)
    assert frac > 0.3


def test_detect_field_rectangle_real_frame_if_present():
    # If the operator's actual captured frame is on disk (rpi5), assert the fix
    # finds the field. Skips cleanly elsewhere (frame not committed to the repo).
    import os
    path = os.path.expanduser('~/kinect_field/calib_rgb.png')
    if not os.path.exists(path):
        pytest.skip('no captured calib_rgb.png on this host')
    bgr = cv2.imread(path)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    corners, diag = detect_field_rectangle(rgb, approx_eps_frac=0.02)
    assert corners is not None, f"real frame: no quad (diag={diag})"
    frac = cv2.contourArea(corners.astype(np.int32)) / float(rgb.shape[0] * rgb.shape[1])
    assert frac > 0.3


def _identity_calibration():
    # field == camera (flat plane at z=2000, axis-aligned) for a simple round-trip
    T = np.eye(4).tolist()
    return CalibrationResult(
        intrinsics={'fx': 594.21, 'fy': 591.04, 'cx': 339.31, 'cy': 242.74},
        corners_px=[[0, 0], [100, 0], [100, 100], [0, 100]],
        corners_cam_mm=[[0, 0, 2000], [1200, 0, 2000], [1200, 800, 2000], [0, 800, 2000]],
        corners_field_cm=[[0, 0], [120, 0], [120, 80], [0, 80]],
        plane_n=[0.0, 0.0, -1.0], plane_d=2000.0,
        T_field_from_cam=T, origin_corner_index=0,
        long_edge_len_cm=120.0, short_edge_len_cm=80.0)


def test_tick_publishes_pose_for_locked_tracker():
    rclpy.init()
    node = None
    try:
        node = FieldTrackerNode()  # source defaults to sim
        node._apply_calibration(_identity_calibration())
        node._heightmap = node._source.baseline()

        # inject a locked tracker + capture its published poses
        name = 'SB-TEST'
        node._trackers[name] = FusionKF(10.0, 20.0, node._fparams)
        published = {}
        orig = node._publish_pose

        def _capture(n, kf, stamp):
            published[n] = kf.pos_cm
            return orig(n, kf, stamp)

        node._publish_pose = _capture
        node._state = TRACKING

        node._tick()  # one cycle: predict (+ maybe camera) + publish
        assert name in published
        x, y = published[name]
        assert np.isfinite(x) and np.isfinite(y)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_marker_republish_publishes_only_when_calibrated():
    rclpy.init()
    node = None
    try:
        node = FieldTrackerNode()
        calls = []
        node._publish_corners_marker = lambda: calls.append(1)

        node._calib = None
        node._republish_marker()
        assert calls == []  # no-op while uncalibrated

        node._apply_calibration(_identity_calibration())  # sets _calib
        calls.clear()
        node._republish_marker()
        assert calls == [1]  # republishes once calibrated
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

"""Unit tests for brightness-change lit-confirmation + overlap match (no hardware)."""

import cv2
import numpy as np

from kinect_field_tracking.rgb_probe import detect_lit_blobs, match_lit_depth_blob


def _frame(discs):
    """discs: list of (center, radius, brightness) -> RGB frame (grey discs)."""
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    for center, radius, b in discs:
        cv2.circle(img, center, radius, (b, b, b), -1)
    return img


def test_detect_lit_blob_from_brightness_increase():
    pre = _frame([((300, 200), 14, 60)])      # dim disc
    post = _frame([((300, 200), 14, 255)])     # same disc, now blown-out white
    blobs, _ = detect_lit_blobs(post, pre, brightness_delta=50, min_area=15)
    assert len(blobs) == 1
    cu, cv = blobs[0]['centroid']
    assert abs(cu - 300) < 3 and abs(cv - 200) < 3


def test_no_lit_blob_when_no_brightening():
    pre = _frame([((300, 200), 14, 60)])
    post = _frame([((300, 200), 14, 60)])      # identical -> diff ~0
    blobs, _ = detect_lit_blobs(post, pre, brightness_delta=50, min_area=15)
    assert blobs == []


def test_already_bright_region_in_both_frames_is_cancelled():
    # a region bright in BOTH pre and post (e.g. an already-registered Sphero)
    # must NOT be detected; only the NEWLY-bright disc is.
    pre = _frame([((100, 100), 14, 255), ((400, 300), 14, 60)])
    post = _frame([((100, 100), 14, 255), ((400, 300), 14, 255)])
    blobs, _ = detect_lit_blobs(post, pre, brightness_delta=50, min_area=15)
    assert len(blobs) == 1
    cu, cv = blobs[0]['centroid']
    assert abs(cu - 400) < 3 and abs(cv - 300) < 3  # only the newly-lit one


def test_lit_blob_blowout_white_is_detected():
    # explicit blow-out-to-white case (the real Kinect failure): hue is gone but
    # the brightness jump is large.
    pre = _frame([((250, 250), 12, 40)])
    post = _frame([((250, 250), 12, 255)])
    blobs, _ = detect_lit_blobs(post, pre, brightness_delta=50, min_area=15)
    assert len(blobs) == 1


def test_match_within_and_beyond_tolerance():
    proj = [(100.0, 100.0)]
    near = [{'centroid': (115.0, 108.0), 'bbox': (108, 101, 14, 14), 'area': 150}]
    far = [{'centroid': (160.0, 100.0), 'bbox': (153, 93, 14, 14), 'area': 150}]
    assert match_lit_depth_blob(proj, near, 25.0)['matched']
    res = match_lit_depth_blob(proj, far, 25.0)
    assert not res['matched'] and res['reason'] == 'too_far'


def test_multi_blob_disambiguation():
    proj = [(100.0, 100.0), (400.0, 300.0)]
    lit = [{'centroid': (410.0, 305.0), 'bbox': (403, 298, 14, 14), 'area': 150}]
    res = match_lit_depth_blob(proj, lit, 25.0)
    assert res['matched'] and res['depth_index'] == 1


def test_end_to_end_brighten_then_match():
    proj_centre = (300.0, 200.0)
    pre = _frame([((312, 210), 14, 60)])       # dim, offset ~15.6px from projection
    post = _frame([((312, 210), 14, 255)])      # lit
    lit, _ = detect_lit_blobs(post, pre, brightness_delta=50, min_area=15)
    res = match_lit_depth_blob([proj_centre], lit, overlap_tol_px=25.0)
    assert res['matched'] and res['distance'] < 25.0


def test_no_lit_means_no_match():
    pre = _frame([((300, 200), 14, 60)])
    post = _frame([((300, 200), 14, 60)])
    lit, _ = detect_lit_blobs(post, pre, brightness_delta=50, min_area=15)
    res = match_lit_depth_blob([(300.0, 200.0)], lit, 25.0)
    assert not res['matched'] and res['reason'] == 'no_green'

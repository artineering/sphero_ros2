#!/usr/bin/env python3
"""Arena quad detection and polygon containment."""

import cv2
import numpy as np

from overhead_tracking import arena as A

POLY_CM = [[0.0, 0.0], [200.0, 0.0], [200.0, 120.0], [0.0, 120.0]]


def test_white_quad_on_dark_is_found():
    img = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(img, (100, 80), (540, 400), (255, 255, 255), 3)
    corners, diag = A.detect_arena_quad(img)
    assert corners is not None and corners.shape == (4, 2)
    assert diag['num_quads'] == 1
    xs, ys = corners[:, 0], corners[:, 1]
    assert abs(xs.min() - 100) < 12 and abs(xs.max() - 540) < 12
    assert abs(ys.min() - 80) < 12 and abs(ys.max() - 400) < 12


def test_grayscale_input_is_accepted():
    """The overhead camera is used as mono, so gray must work as well as BGR."""
    gray = np.zeros((480, 640), np.uint8)
    cv2.rectangle(gray, (100, 80), (540, 400), 255, 3)
    corners, _ = A.detect_arena_quad(gray)
    assert corners is not None and corners.shape == (4, 2)


def test_blank_frame_finds_nothing():
    corners, diag = A.detect_arena_quad(np.zeros((480, 640), np.uint8))
    assert corners is None and diag['num_quads'] == 0


def test_matte_black_arena_is_rejected_not_hallucinated():
    """Documents WHY manual corners are the default, and guards a real defect.

    At the calibrated exposure the arena reads p50 2-4 with no edge to find. On
    such a frame Canny fires on noise everywhere, the CLOSE merges it, and the
    external contour is the rectangle just inside the suppressed border -- so the
    detector confidently returns the whole frame as the "arena". max_area_frac
    catches that; without it the operator gets a homography built from the image
    edges and every published position is wrong.
    """
    rng = np.random.default_rng(0)
    dark = (rng.random((720, 1280)) * 4).astype(np.uint8)
    corners, diag = A.detect_arena_quad(dark)
    assert corners is None
    assert 'fills the frame' in diag['rejected']


def test_quad_hugging_the_frame_edge_is_rejected():
    """Two independent guards catch this: border suppression zeroes edges within
    border_margin, and max_area_frac catches anything that survives. Here it is
    the former, so only the outcome is asserted."""
    img = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(img, (2, 2), (637, 477), (255, 255, 255), 3)
    corners, _ = A.detect_arena_quad(img)
    assert corners is None


def test_coloured_tape_on_textured_floor_is_found():
    """Tape can be a strong colour edge but a weak brightness edge."""
    rng = np.random.default_rng(2)
    img = np.zeros((480, 640, 3), np.uint8)
    img[:, :] = (90, 110, 130)
    img += (rng.random((480, 640, 3)) * 18).astype(np.uint8)
    cv2.rectangle(img, (90, 70), (550, 410), (150, 90, 40), 4)   # blue-ish tape
    corners, _ = A.detect_arena_quad(img)
    assert corners is not None


def test_contains_cm_inside_outside_and_margin():
    assert A.contains_cm(POLY_CM, 100.0, 60.0)
    assert not A.contains_cm(POLY_CM, 250.0, 60.0)
    assert not A.contains_cm(POLY_CM, -3.0, 60.0)
    assert A.contains_cm(POLY_CM, -3.0, 60.0, margin_cm=5.0)
    assert not A.contains_cm(POLY_CM, -3.0, 60.0, margin_cm=1.0)


def test_contains_px_and_mask_agree():
    quad = [[100, 600], [900, 640], [950, 120], [80, 100]]
    mask = A.arena_mask_px(quad, (720, 1280))
    for u, v in [(500, 400), (200, 200), (1200, 700), (10, 10)]:
        assert A.contains_px(quad, u, v) == bool(mask[v, u])

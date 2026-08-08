#!/usr/bin/env python3
"""Arena calibration persistence."""

import os

import numpy as np
import pytest

from overhead_tracking import homography as H
from overhead_tracking.arena_store import ArenaCalibration, load_arena, save_arena

QUAD = [[100.0, 600.0], [900.0, 640.0], [950.0, 120.0], [80.0, 100.0]]


def make():
    cm, _ = H.field_corners_from_px(QUAD, 200.0, 120.0)
    h = H.build_homography(QUAD, cm)
    mx, mean = H.residual_cm(QUAD, cm, h)
    return ArenaCalibration(
        corners_px=[list(p) for p in QUAD],
        corners_field_cm=[list(p) for p in cm],
        H=[list(r) for r in h],
        long_edge_len_cm=200.0, short_edge_len_cm=120.0,
        frame_width=1280, frame_height=720,
        residual_max_cm=mx, residual_mean_cm=mean,
        camera_settings={'gain': 100, 'auto_exposure': 1},
        created='2026-08-08T12:00:00', source='manual')


def test_roundtrip_preserves_the_homography(tmp_path):
    cal = make()
    p = save_arena(str(tmp_path / 'sub' / 'arena.yaml'), cal)
    assert os.path.exists(p)
    back = load_arena(p)
    assert np.allclose(back.H_np, cal.H_np, rtol=0, atol=1e-12)
    assert back.corners_px == cal.corners_px
    assert back.corners_field_cm == cal.corners_field_cm
    assert back.long_edge_len_cm == 200.0 and back.short_edge_len_cm == 120.0


def test_camera_settings_survive(tmp_path):
    """Recorded so it is obvious when a stored arena predates a camera change."""
    p = save_arena(str(tmp_path / 'a.yaml'), make())
    assert load_arena(p).camera_settings == {'gain': 100, 'auto_exposure': 1}


def test_loaded_homography_still_maps_corners(tmp_path):
    p = save_arena(str(tmp_path / 'a.yaml'), make())
    back = load_arena(p)
    mx, _ = H.residual_cm(back.corners_px_np, back.corners_cm_np, back.H_np)
    assert mx < 1e-6


def test_inverse_property_is_consistent(tmp_path):
    back = load_arena(save_arena(str(tmp_path / 'a.yaml'), make()))
    assert np.allclose(back.H_np @ back.H_inv_np, np.eye(3), atol=1e-9)


def test_missing_file_raises(tmp_path):
    with pytest.raises(OSError):
        load_arena(str(tmp_path / 'nope.yaml'))

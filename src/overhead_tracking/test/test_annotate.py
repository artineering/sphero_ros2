#!/usr/bin/env python3
"""Overlay drawing and JPEG encoding."""

import cv2
import numpy as np

from overhead_tracking import annotate as A

GRAY = np.zeros((240, 320), np.uint8)
RENDER = {
    'state': 'TRACKING',
    'arena_px': [[20, 220], [300, 225], [305, 20], [15, 18]],
    'tracks': [{'name': 'SB-33E3', 'u': 160, 'v': 120, 'status': 'HEALTHY',
                'score': 0.76, 'angle': 30.0, 'roi': (110, 70, 210, 170)}],
}


def test_overlay_returns_bgr_and_draws_something():
    out = A.draw_overlay(GRAY, RENDER)
    assert out.shape == (240, 320, 3) and out.dtype == np.uint8
    assert out.any()


def test_status_changes_the_colour():
    a = A.draw_overlay(GRAY, RENDER)
    lost = {**RENDER, 'tracks': [{**RENDER['tracks'][0], 'status': 'LOST'}]}
    assert not np.array_equal(a, A.draw_overlay(GRAY, lost))


def test_empty_and_none_render_are_safe():
    assert A.draw_overlay(GRAY, {}) is not None
    assert A.draw_overlay(GRAY, None) is not None
    assert A.draw_overlay(None, RENDER) is None


def test_staged_blobs_render_before_linking():
    out = A.draw_overlay(GRAY, {'state': 'BLOBS_READY',
                                'blobs': [{'id': 0, 'u': 100, 'v': 100}]})
    assert out.any()


def test_jpeg_roundtrip():
    blob = A.encode_jpeg(A.draw_overlay(GRAY, RENDER), 70)
    assert blob[:2] == b'\xff\xd8'                       # JPEG SOI
    assert cv2.imdecode(np.frombuffer(blob, np.uint8), 1).shape == (240, 320, 3)


def test_lower_quality_is_smaller():
    img = A.draw_overlay(np.full((240, 320), 40, np.uint8), RENDER)
    assert len(A.encode_jpeg(img, 40)) < len(A.encode_jpeg(img, 90))


def test_scale_shrinks_the_output():
    """The degrade path halves the edge, quartering the pixels."""
    img = A.draw_overlay(GRAY, RENDER)
    dec = cv2.imdecode(np.frombuffer(A.encode_jpeg(img, 70, 0.5), np.uint8), 1)
    assert dec.shape == (120, 160, 3)


def test_encode_none_returns_empty():
    assert A.encode_jpeg(None) == b''

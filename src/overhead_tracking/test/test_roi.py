#!/usr/bin/env python3
"""ROI sizing, clamping and forward prediction."""

from overhead_tracking import homography as H
from overhead_tracking import roi as R

BALL_D, SCALE = 34, 3.0


def test_base_half_is_three_times_the_ball_radius():
    assert R.roi_half(BALL_D, SCALE, 0.0, 0.0, 0) == 51      # 3 * 34 / 2


def test_velocity_widens_the_window():
    still = R.roi_half(BALL_D, SCALE, 0.0, 1 / 30, 0)
    moving = R.roi_half(BALL_D, SCALE, 600.0, 1 / 30, 0)
    assert moving > still


def test_misses_grow_the_window_geometrically_then_saturate():
    halves = [R.roi_half(BALL_D, SCALE, 0.0, 0.0, m, growth=1.5, lo=30, hi=160)
              for m in range(8)]
    assert halves[1] > halves[0] and halves[2] > halves[1]
    assert halves[-1] == 160                                  # clamped at hi
    assert all(b >= a for a, b in zip(halves, halves[1:]))    # monotonic


def test_lower_bound_holds():
    assert R.roi_half(4, 0.5, 0.0, 0.0, 0, lo=30, hi=160) == 30


def test_rect_clamps_at_every_border():
    for cx, cy in [(5, 5), (1275, 5), (5, 715), (1275, 715)]:
        x0, y0, x1, y1, clamped = R.roi_from_center(cx, cy, 51, 1280, 720)
        assert clamped
        assert 0 <= x0 < x1 <= 1280 and 0 <= y0 < y1 <= 720


def test_centre_rect_is_not_clamped():
    rect = R.roi_from_center(640, 360, 51, 1280, 720)
    assert rect == (589, 309, 691, 411, False)
    assert (rect[2] - rect[0]) == 102                          # "3x blob size"


def test_degenerate_flags_windows_too_small_for_the_template():
    """A window must hold the 41 px template plus its 4 px search pad (49 px).

    Note a corner ROI is NOT automatically degenerate: at half=51 the clamped
    window is still 53x53, which fits. Degeneracy is about the resulting SIZE.
    """
    assert not R.degenerate(R.roi_from_center(640, 360, 51, 1280, 720), 41, 4)
    assert not R.degenerate(R.roi_from_center(2, 2, 51, 1280, 720), 41, 4)
    assert R.degenerate(R.roi_from_center(2, 2, 25, 1280, 720), 41, 4)
    assert R.degenerate((0, 0, 48, 100), 41, 4)
    assert not R.degenerate((0, 0, 49, 49), 41, 4)


def test_contains():
    rect = R.roi_from_center(640, 360, 51, 1280, 720)
    assert R.contains(rect, 640, 360)
    assert not R.contains(rect, 800, 360)


def test_forward_prediction_extrapolates_in_cm_then_maps():
    """Extrapolating in cm and then mapping is the only correct order under
    perspective, where px/cm varies across the frame."""
    quad = [[100.0, 600.0], [900.0, 640.0], [950.0, 120.0], [80.0, 100.0]]
    cm, _ = H.field_corners_from_px(quad, 200.0, 120.0)
    h = H.build_homography(quad, cm)
    hi = H.invert_homography(h)

    x_cm, y_cm = H.apply_homography(500, 380, h)
    u, v = R.predict_center_px(x_cm, y_cm, 0.0, 0.0, 0.0, hi, H.field_cm_to_px)
    assert abs(u - 500) < 1e-6 and abs(v - 380) < 1e-6

    # moving +x in field cm must move in the +x field direction, not raw pixels
    u2, v2 = R.predict_center_px(x_cm, y_cm, 60.0, 0.0, 0.5, hi, H.field_cm_to_px)
    x2, y2 = H.apply_homography(u2, v2, h)
    assert abs(x2 - (x_cm + 30.0)) < 1e-6
    assert abs(y2 - y_cm) < 1e-6

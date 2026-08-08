#!/usr/bin/env python3
"""Shape template + matching. The composition test is the regression guard."""

import numpy as np
import pytest

from overhead_tracking import template as T


def render(width=1280, height=720, robots=(), noise=3, seed=0, ball_d=34):
    """Frame with real templates drawn at the given (cx, cy, angle) triples."""
    rng = np.random.default_rng(seed)
    frame = (rng.random((height, width)) * noise).astype(np.uint8)
    t = T.build_template(ball_d=ball_d)
    for cx, cy, ang in robots:
        patch = np.clip(T.rotate(t, ang) * 255, 0, 255).astype(np.uint8)
        h, w = patch.shape
        y0, x0 = cy - h // 2, cx - w // 2
        frame[y0:y0 + h, x0:x0 + w] = np.maximum(frame[y0:y0 + h, x0:x0 + w],
                                                 patch)
    return frame


def test_template_shape_and_bank():
    t = T.build_template()
    assert t.shape == (41, 41)
    assert len(T.bank(t, 15)) == 12          # 180 deg / 15, point LEDs symmetric
    assert [d for d, _ in T.bank(t, 15)][:3] == [0, 15, 30]


def test_detect_roi_finds_all_robots_at_truth():
    truth = [(300, 200, 0), (700, 400, 45), (1000, 150, 120), (500, 600, 90)]
    frame = render(robots=truth)
    hits = T.detect_roi(frame, T.bank(T.build_template()))
    assert len(hits) == len(truth)
    found = sorted((x, y) for x, y, _s, _a in hits)
    for cx, cy, _ in sorted(truth):
        assert any(abs(fx - cx) <= 2 and abs(fy - cy) <= 2 for fx, fy in found)


def test_detect_roi_equals_component_plus_match_composition():
    """detect_roi must remain exactly the two exposed halves, composed.

    The node runs bright_components once per frame and dispatches match_bank_at
    to a thread pool. If that split ever drifts from the validated detect_roi,
    the fast path silently stops being the thing that was measured.
    """
    frame = render(robots=[(300, 200, 0), (800, 500, 60), (1100, 300, 135)])
    tb = T.bank(T.build_template())
    hits = T.detect_roi(frame, tb)

    manual = []
    for cx, cy, _a, _b in T.bright_components(frame, 60, 30, tb[0][1].shape[0] // 2):
        m = T.match_bank_at(frame, tb, cx, cy, 4)
        if m is not None and m[2] >= 0.45:
            manual.append(m)
    assert manual == hits


def test_scaled_candidates_agree_with_full_resolution():
    """The candidate mask is the bottleneck (31.6 ms at 2.3 MP vs 0.93 ms to
    match one candidate), so it runs at half resolution. That must not change
    which robots are found or where."""
    truth = [(400, 300, 0), (900, 500, 45), (1500, 800, 120), (700, 950, 90)]
    frame = render(width=1920, height=1200, robots=truth)
    tb = T.bank(T.build_template())
    full = sorted(T.detect_roi(frame, tb, pad_extra=4, scale=1))
    half = sorted(T.detect_roi(frame, tb, pad_extra=8, scale=2))
    assert len(full) == len(half) == len(truth)
    for (xf, yf, _sf, _af), (xh, yh, _sh, _ah) in zip(full, half):
        assert abs(xf - xh) <= 1 and abs(yf - yh) <= 1


def test_scale_one_is_the_unscaled_path():
    frame = render(robots=[(300, 200, 0), (800, 500, 60)])
    tb = T.bank(T.build_template())
    assert T.detect_roi(frame, tb, scale=1) == T.detect_roi(frame, tb)
    assert (T.bright_components_scaled(frame, 60, 30, 20, 1)
            == T.bright_components(frame, 60, 30, 20))


def test_scaled_components_map_back_to_full_resolution_coords():
    frame = render(width=1920, height=1200, robots=[(960, 600, 0)])
    comps = T.bright_components_scaled(frame, 60, 30, 20, 2)
    assert comps
    cx, cy = comps[0][0], comps[0][1]
    assert abs(cx - 960) <= 4 and abs(cy - 600) <= 4    # within pad_extra=8


def test_blank_frame_yields_nothing():
    """The brightness gate is load-bearing: ungated, noise scored up to 0.634."""
    tb = T.bank(T.build_template())
    assert T.detect_roi(np.zeros((720, 1280), np.uint8), tb) == []
    rng = np.random.default_rng(1)
    noise = (rng.random((720, 1280)) * 6).astype(np.uint8)
    assert T.detect_roi(noise, tb) == []


def test_robot_below_min_bright_is_suppressed():
    frame = render(robots=[(400, 300, 0)])
    frame = (frame.astype(np.int32) // 8).astype(np.uint8)      # dim it right down
    assert T.detect_roi(frame, T.bank(T.build_template())) == []


def test_match_bank_at_returns_none_at_frame_border():
    """A clamped window smaller than the template must miss, not half-match."""
    frame = render(robots=[(300, 200, 0)])
    tb = T.bank(T.build_template())
    assert T.match_bank_at(frame, tb, 2, 2) is None


def test_match_bank_at_is_identical_on_a_view_and_a_copy():
    """Pool workers receive numpy views; they must behave like copies."""
    frame = render(robots=[(640, 360, 30)])
    tb = T.bank(T.build_template())
    view = frame[100:700, 200:1100]
    assert T.match_bank_at(view, tb, 440, 260) == \
        T.match_bank_at(np.array(view, copy=True), tb, 440, 260)


def test_match_bank_at_reports_score_below_threshold():
    """Unthresholded return keeps near-misses available as diagnostics."""
    frame = render(robots=[(640, 360, 0)])
    m = T.match_bank_at(frame, T.bank(T.build_template()), 640, 360)
    assert m is not None and 0.0 <= m[2] <= 1.0


@pytest.mark.parametrize('angle', [0, 30, 75, 135, 165])
def test_bank_recovers_rendered_heading(angle):
    """Heading comes out of the winning bank entry, within one bank step."""
    frame = render(robots=[(640, 360, angle)])
    hits = T.detect_roi(frame, T.bank(T.build_template(), 15))
    assert len(hits) == 1
    got = hits[0][3] % 180
    diff = min(abs(got - angle), 180 - abs(got - angle))
    assert diff <= 15

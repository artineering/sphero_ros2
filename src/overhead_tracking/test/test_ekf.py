#!/usr/bin/env python3
"""HeadingEKF, CommandHistory and HeadingOffsetEstimator on synthetic truth."""

import math

import numpy as np

from overhead_tracking.ekf import (CommandHistory, HeadingEKF,
                                   HeadingOffsetEstimator, wrap)

FPS = 60.0
DT = 1.0 / FPS
K = 0.61                                   # speed -> cm/s


def simulate(cmds, t_end, offset_deg=0.0, lag=0.45, rate_dps=180.0, x0=(100.0, 100.0)):
    """Truth robot obeying the same model as the filter: speed applied `lag` s
    late, field heading = offset - cmd turned at <= rate_dps. Starts facing
    offset - (first cmd heading). Returns per-frame (t, x, y, theta)."""
    cmds = sorted(cmds)
    th = math.radians(offset_deg - cmds[0][2])
    x, y, t, out = x0[0], x0[1], 0.0, []
    rate = math.radians(rate_dps) * DT

    def at(tq):
        last = None
        for c in cmds:
            if c[0] <= tq:
                last = c
        return last

    while t < t_end:
        t += DT
        c = at(t - lag)
        v = c[1] * K if c else 0.0
        c = at(t)
        target = math.radians(offset_deg - c[2]) if c else th
        th = wrap(th + max(-rate, min(rate, wrap(target - th))))
        x += v * math.cos(th) * DT
        y += v * math.sin(th) * DT
        out.append((t, x, y, th))
    return out


def run_filter(cmds, truth, noise=(0.2, 0.03), hide=lambda t: False, seed=0):
    rng = np.random.default_rng(seed)
    hist = CommandHistory()
    for c in cmds:
        hist.add(*c)
    t0, x0, y0, th0 = truth[0]
    ekf = HeadingEKF(x0, y0, th0)
    hist.inputs(t0)
    errs = []
    for t, x, y, th in truth[1:]:
        ekf.predict(DT, *hist.inputs(t))
        if not hide(t):
            ekf.update((x + rng.normal(0, noise[0]), y + rng.normal(0, noise[0])),
                       th + rng.normal(0, noise[1]))
        errs.append((t, math.hypot(ekf.x[0] - x, ekf.x[1] - y),
                     abs(wrap(ekf.x[2] - th))))
    return ekf, np.array(errs)


# ------------------------------------------------------------------ EKF
def test_converges_on_straight_run_with_stop_start():
    cmds = [(0.0, 0, 0), (1.0, 60, 0), (3.0, 0, 0), (4.0, 60, 0), (6.0, 0, 0)]
    truth = simulate(cmds, 7.0)
    _ekf, e = run_filter(cmds, truth)
    late = e[e[:, 0] > 1.0]
    assert late[:, 1].mean() < 0.3
    assert late[:, 2].max() < math.radians(5)


def test_blind_prediction_follows_lagged_speed_and_turns():
    # 0.5 s gaps every 2 s while driving and turning
    cmds = [(0.0, 0, 0)] + [(1.0 + 0.6 * k, 60, (30 * k) % 360) for k in range(12)]
    truth = simulate(cmds, 9.0)

    def hide(t):
        return (t % 2.0) > 1.5

    _ekf, e = run_filter(cmds, truth, hide=hide)
    blind = e[np.array([hide(t) for t in e[:, 0]])]
    assert blind[:, 1].max() < 3.0           # same model -> only noise-driven error
    # a filter with no inputs drifts much further over the same gaps
    ekf = HeadingEKF(*truth[0][1:3], truth[0][3])
    worst = 0.0
    for t, x, y, th in truth[1:]:
        ekf.predict(DT, 0.0, 0.0)
        if not hide(t):
            ekf.update((x, y), th)
        else:
            worst = max(worst, math.hypot(ekf.x[0] - x, ekf.x[1] - y))
    assert worst > 5 * blind[:, 1].max()


def test_angle_wraps_across_180():
    # robot facing ~179 deg field, commanded through the +-180 seam
    cmds = [(0.0, 0, 181), (1.0, 40, 181), (2.0, 40, 175), (3.0, 40, 185), (4.0, 40, 181)]
    truth = simulate(cmds, 5.0)
    ekf, e = run_filter(cmds, truth)
    assert e[:, 2].max() < math.radians(5)
    assert -math.pi <= ekf.x[2] < math.pi


def test_theta_residual_is_wrapped():
    ekf = HeadingEKF(0.0, 0.0, math.radians(179))
    y, nis = ekf.update((0.0, 0.0), math.radians(-179))
    assert abs(math.degrees(y[2]) - 2.0) < 1e-6
    assert nis < 10


def test_position_only_start_waits_for_led_fix():
    ekf = HeadingEKF(10.0, 20.0)
    assert not ekf.theta_known and ekf.theta is None and ekf.heading_deg is None
    ekf.predict(DT, 50.0, 1.0)                 # no heading -> speed not applied
    assert ekf.pos_cm == (10.0, 20.0)
    ekf.update((10.0, 20.0))                   # position-only fix keeps it unknown
    assert not ekf.theta_known
    ekf.update((10.0, 20.0), math.radians(-90))
    assert ekf.theta_known
    assert abs(ekf.heading_deg - 270.0) < 1e-6
    ekf.predict(1.0, 10.0, 0.0)
    assert abs(ekf.pos_cm[1] - 10.0) < 1e-6


def test_mahalanobis_and_covariance_grow_without_fixes():
    ekf = HeadingEKF(0.0, 0.0, 0.0)
    near, far = ekf.mahalanobis2((1.0, 0.0)), ekf.mahalanobis2((10.0, 0.0))
    assert near < far
    p0 = np.trace(ekf.pos_cov)
    for _ in range(30):
        ekf.predict(DT, 0.0, 0.0)
    assert np.trace(ekf.pos_cov) > p0
    assert ekf.mahalanobis2((10.0, 0.0)) < far          # gate widens with P
    assert ekf.x[0] == 0.0                              # helper does not update


def test_position_noise_is_larger_without_heading():
    # without theta the speed is not applied: the random walk carries the motion
    known, unknown = HeadingEKF(0.0, 0.0, 0.0), HeadingEKF(0.0, 0.0)
    for ekf in (known, unknown):
        ekf.predict(0.5, 0.0, 0.0)
    assert np.allclose(np.diag(known.pos_cov), 1.0 + 100.0 * 0.5)
    assert np.allclose(np.diag(unknown.pos_cov), 1.0 + 500.0 * 0.5)


# ------------------------------------------------------------------ commands
def test_speed_uses_lagged_command():
    h = CommandHistory(lag_v_sec=0.45)
    h.add(1.0, 50, 0)
    h.add(2.0, 0, 0)
    assert h.inputs(1.40)[0] == 0.0
    assert abs(h.inputs(1.46)[0] - 50 * 0.61) < 1e-9
    assert abs(h.inputs(2.44)[0] - 50 * 0.61) < 1e-9
    assert h.inputs(2.46)[0] == 0.0


def test_turn_is_a_delta_rate_limited_and_sign_flipped():
    h = CommandHistory(turn_rate_dps=180.0)
    h.add(0.0, 0, 350)                         # first command: no turn, offset unknown
    t, total = 0.0, 0.0
    h.inputs(t)
    for _ in range(int(2 * FPS)):
        t += DT
        if abs(t - 0.5) < DT / 2:
            h.add(t, 0, 80)                    # +90 deg clockwise across 0
        _v, w = h.inputs(t)
        assert abs(w) <= math.radians(180) + 1e-9
        total += w * DT
    assert abs(math.degrees(total) + 90) < 1e-6   # CCW field = -90


def test_history_is_pruned():
    h = CommandHistory()
    t = 0.0
    for k in range(1000):
        t = 0.1 * k
        h.add(t, 50, 0)
        h.inputs(t)
    assert len(h._cmds) <= 6                   # only the last ~lag s of commands


# ------------------------------------------------------------------ offset
def test_offset_recovered_from_noisy_samples():
    rng = np.random.default_rng(1)
    est = HeadingOffsetEstimator()
    offset, t = 357.0, 0.0                     # near the seam on purpose
    for cmd in (0, 90, 200, 300):
        est.on_cmd(t, cmd)
        for _ in range(int(3 * FPS)):
            t += DT
            est.add(t, (offset - cmd) + rng.normal(0, 3.0))
    assert est.value is not None
    assert abs((est.value - offset + 180) % 360 - 180) < 1.5
    assert 0.0 <= est.value < 360.0


def test_offset_ignores_samples_within_settle_time():
    est = HeadingOffsetEstimator(settle_sec=1.5, n_init=5)
    est.on_cmd(0.0, 0)
    for k in range(80):                        # LED still shows the OLD heading
        assert not est.add(k / FPS, 123.0)
    assert est.value is None
    for k in range(5):
        assert est.add(1.5 + k / FPS, 40.0)
    assert abs(est.value - 40.0) < 1e-6
    est.on_cmd(2.0, 0)                         # same heading: not a change
    assert est.add(2.1, 40.0)


def test_offset_publish_bookkeeping_and_reset():
    est = HeadingOffsetEstimator(n_init=1, gain=0.5)
    assert not est.changed_since_publish()
    est.on_cmd(0.0, 10)
    est.add(2.0, 20.0)                         # offset 30
    assert est.changed_since_publish()
    est.mark_published()
    assert not est.changed_since_publish()
    est.add(2.1, 21.0)                         # -> 30.5
    assert not est.changed_since_publish(threshold_deg=1.0)
    est.add(2.2, 25.0)                         # -> 32.75
    assert est.changed_since_publish(threshold_deg=1.0)
    est.reset()
    assert est.value is None and not est.changed_since_publish()
    assert not est.add(5.0, 20.0)              # no command since reset

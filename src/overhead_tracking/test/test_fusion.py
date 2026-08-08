#!/usr/bin/env python3
"""13-state fusion filter. Guards the accel-is-passive design decision."""

import numpy as np

from overhead_tracking.fusion import PX, PY, VX, FusionKF, FusionParams, \
    rotate_body_to_field


def kf(x=10.0, y=20.0, **kw):
    return FusionKF(x, y, FusionParams(**kw))


def test_initial_state_is_the_seeded_position():
    f = kf(10.0, 20.0)
    assert f.pos_cm == (10.0, 20.0)


def test_predict_grows_covariance():
    f = kf()
    p0 = f.P[PX, PX]
    f.predict(0.1)
    assert f.P[PX, PX] > p0


def test_camera_update_pulls_position_toward_the_measurement():
    f = kf(0.0, 0.0)
    f.predict(0.1)
    f.update_camera(50.0, 0.0)
    assert 0.0 < f.pos_cm[0] <= 50.0


def test_repeated_camera_updates_converge():
    f = kf(0.0, 0.0)
    for _ in range(30):
        f.predict(0.1)
        f.update_camera(50.0, 25.0)
    assert abs(f.pos_cm[0] - 50.0) < 1.0 and abs(f.pos_cm[1] - 25.0) < 1.0


def test_accelerometer_never_moves_position():
    """The load-bearing one.

    Body accel gets only a yaw rotation, so gravity and bias leak into the
    field-plane components. Double-integrating that produced a quadratic runaway
    that drifted past the association gate. Accel stays a passive measured state.
    """
    f = kf(0.0, 0.0)
    for _ in range(50):
        f.predict(0.05)
        f.update_accel_body_g(1.0, 1.0, 1.0)      # a full g on every axis
    assert abs(f.pos_cm[0]) < 1e-6 and abs(f.pos_cm[1]) < 1e-6


def test_velocity_drives_dead_reckoning():
    """A missing camera blob must still advance the estimate."""
    f = kf(0.0, 0.0)
    for _ in range(20):
        f.predict(0.05)
        f.update_velocity_body(40.0, 0.0)
    before = f.pos_cm[0]
    for _ in range(10):
        f.predict(0.05)                            # predict-only, no camera
    assert f.pos_cm[0] > before


def test_gyro_integrates_into_yaw():
    f = kf()
    for _ in range(20):
        f.predict(0.05)
        f.update_gyro(0.0, 0.0, 90.0)
    assert f.yaw_deg > 5.0


def test_orientation_applies_the_body_to_field_offset():
    f = kf(body_to_field_yaw_offset=30.0)
    for _ in range(30):
        f.predict(0.05)
        f.update_orientation(90.0, 0.0, 0.0)
    assert abs(f.yaw_deg - 60.0) < 5.0


def test_rotate_body_to_field_quarter_turn():
    fx, fy = rotate_body_to_field(1.0, 0.0, 90.0)
    assert abs(fx) < 1e-9 and abs(fy - 1.0) < 1e-9


def test_velocity_is_rotated_by_the_estimated_yaw():
    f = kf(0.0, 0.0)
    f.x[4] = 90.0                                   # yaw = 90 deg
    f.predict(0.01)
    f.update_velocity_body(50.0, 0.0)               # body +x
    assert f.x[VX] < f.x[VX + 1]                    # ends up mostly in field +y


def test_state_stays_finite_under_a_long_mixed_run():
    f = kf(0.0, 0.0)
    rng = np.random.default_rng(0)
    for i in range(500):
        f.predict(1 / 30)
        f.update_velocity_body(float(rng.normal(20, 5)), 0.0)
        f.update_gyro(0.0, 0.0, float(rng.normal(0, 10)))
        f.update_accel_body_g(0.0, 0.0, 1.0)
        if i % 3 == 0:
            f.update_camera(float(i) * 0.1, 0.0)
    assert np.all(np.isfinite(f.x)) and np.all(np.isfinite(f.P))
    assert abs(f.pos_cm[PY]) < 1e4

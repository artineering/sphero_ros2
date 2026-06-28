"""Unit tests for the per-callsign sensor-fusion Kalman filter."""

import numpy as np

from kinect_field_tracking.fusion import (
    FusionKF,
    FusionParams,
    rotate_body_to_field,
    PX, VX, VY, YAW,
)


def _kf(**over):
    return FusionKF(0.0, 0.0, FusionParams(**over))


def test_predict_only_dead_reckons_and_grows_covariance():
    kf = _kf()
    kf.x[VX] = 10.0  # cm/s
    p_before = kf.P[PX, PX]
    kf.predict(1.0)
    # position integrated from velocity, no camera needed
    assert abs(kf.pos_cm[0] - 10.0) < 1e-9
    # covariance grew (uncertainty increases without measurement)
    assert kf.P[PX, PX] > p_before


def test_camera_update_reduces_position_covariance():
    kf = _kf()
    kf.predict(1.0)
    p_before = kf.P[PX, PX]
    kf.update_camera(5.0, -3.0)
    assert kf.P[PX, PX] < p_before
    # state pulled toward the measurement
    assert kf.pos_cm[0] > 0.0


def test_body_to_field_rotation():
    fx, fy = rotate_body_to_field(10.0, 0.0, 90.0)
    assert abs(fx) < 1e-9 and abs(fy - 10.0) < 1e-9
    fx, fy = rotate_body_to_field(0.0, 5.0, 0.0)
    assert abs(fx) < 1e-9 and abs(fy - 5.0) < 1e-9


def test_velocity_update_uses_estimated_yaw():
    kf = _kf()
    kf.x[YAW] = 90.0
    kf.update_velocity_body(10.0, 0.0)  # body +x at yaw 90 -> field +y
    assert kf.x[VY] > kf.x[VX]
    assert kf.x[VY] > 0.0


def test_orientation_offset_maps_to_field_heading():
    kf = _kf(body_to_field_yaw_offset=30.0, r_ori=0.5)
    # telemetry yaw 120 (magnetic) with offset 30 => field heading 90
    for _ in range(30):
        kf.update_orientation(120.0, 0.0, 0.0)
    assert abs(kf.yaw_deg - 90.0) < 2.0


def test_no_telemetry_position_fusion_path():
    # The only position measurement is the camera; there is deliberately no
    # telemetry-position update (would be circular via set_external_location).
    kf = _kf()
    assert hasattr(kf, 'update_camera')
    assert not hasattr(kf, 'update_position_telemetry')
    # telemetry-only updates never directly assign px to a telemetry x,y value
    kf.x[PX] = 7.0
    kf.update_velocity_body(0.0, 0.0)
    kf.update_orientation(0.0, 0.0, 0.0)
    kf.update_accel_body_g(0.0, 0.0, 0.0)
    kf.update_gyro(0.0, 0.0, 0.0)
    assert abs(kf.pos_cm[0] - 7.0) < 5.0  # px not yanked by telemetry

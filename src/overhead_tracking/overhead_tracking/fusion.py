#!/usr/bin/env python3
"""Per-callsign sensor-fusion Kalman filter (pure, no ROS, no hardware).

State (13), all in the FIELD frame, units cm / cm-s / deg:
    [ px, py, vx, vy, yaw, pitch, roll, ax, ay, az, gx, gy, gz ]
     0   1   2   3   4    5      6     7   8   9   10  11  12

Fusion design
-------------
* CAMERA gives absolute position (px, py) -- only when a blob is associated.
* TELEMETRY (sphero/<name>/sensors, ~10 Hz) gives field velocity, orientation,
  field accel and gyro EVERY cycle. We NEVER fuse telemetry position (x, y): the
  device controller injects our own /localization output back into its reported
  x,y via set_external_location, so fusing it would be circular.
* A missing camera blob => PREDICT-ONLY that cycle (the filter dead-reckons on
  telemetry velocity) and we STILL publish. Downstream consumers have no
  "invalid" path, so a slightly stale pose beats a gap in the stream.

Body->field rotation
--------------------
Telemetry velocity/accel are body-frame; they are rotated into the field frame
using the filter's current yaw estimate. Compass calibration makes telemetry yaw
magnetic-north-referenced across all robots, so a single global constant
`body_to_field_yaw_offset` maps telemetry yaw into the field-frame heading.
Keeping the rotation on the MEASUREMENT keeps H/R constant and the filter linear.
"""

from dataclasses import dataclass

import numpy as np

# state indices
PX, PY, VX, VY, YAW, PITCH, ROLL, AX, AY, AZ, GX, GY, GZ = range(13)
NSTATE = 13

G_TO_CMS2 = 980.665  # 1 g -> cm/s^2


@dataclass
class FusionParams:
    body_to_field_yaw_offset: float = 0.0   # deg: magnetic-north -> field +x
    # measurement noise (variances)
    r_cam: float = 1.0       # cm^2
    r_vel: float = 4.0       # (cm/s)^2
    r_ori: float = 2.0       # deg^2
    r_acc: float = 50.0      # (cm/s^2)^2
    r_gyro: float = 4.0      # (deg/s)^2
    # process noise (per-state, scaled by dt)
    q_pos: float = 0.04
    q_vel: float = 1.0
    q_yaw: float = 1.0
    q_angle: float = 1.0     # pitch/roll
    q_acc: float = 100.0
    q_gyro: float = 25.0
    p0: float = 100.0        # initial covariance


def rotate_body_to_field(vx_body, vy_body, yaw_deg):
    """Rotate a planar body-frame vector into the field frame by yaw (deg).

    yaw is the robot's heading in the field frame (+ccw from field +x). At yaw=90
    a body +x vector maps to field +y.
    """
    th = np.deg2rad(yaw_deg)
    c, s = np.cos(th), np.sin(th)
    fx = vx_body * c - vy_body * s
    fy = vx_body * s + vy_body * c
    return float(fx), float(fy)


class FusionKF:
    """13-state linear Kalman filter for one callsign, field frame, cm units."""

    def __init__(self, px_cm, py_cm, params: FusionParams, yaw0_deg=0.0):
        self.p = params
        self.x = np.zeros(NSTATE, dtype=float)
        self.x[PX] = float(px_cm)
        self.x[PY] = float(py_cm)
        self.x[YAW] = float(yaw0_deg)
        self.P = np.eye(NSTATE) * params.p0

    # ---------------------------------------------------------------- predict
    def _F(self, dt):
        # Constant-VELOCITY model. The accelerometer is NOT integrated into
        # velocity/position: the Sphero accel is body-frame and gets only a yaw
        # rotation (no pitch/roll), so gravity (~1g) + bias leak into the
        # field-plane accel; double-integrating it caused a quadratic runaway
        # that drifted past the association gate. ax/ay/az remain passive
        # measured states (updated by telemetry) but no longer drive motion.
        # Position integrates VELOCITY only and is pinned by the per-frame
        # camera update; yaw still integrates gyro (gz).
        F = np.eye(NSTATE)
        F[PX, VX] = dt
        F[PY, VY] = dt
        F[YAW, GZ] = dt
        return F

    def _Q(self, dt):
        p = self.p
        q = np.zeros(NSTATE)
        q[PX] = q[PY] = p.q_pos
        q[VX] = q[VY] = p.q_vel
        q[YAW] = p.q_yaw
        q[PITCH] = q[ROLL] = p.q_angle
        q[AX] = q[AY] = q[AZ] = p.q_acc
        q[GX] = q[GY] = q[GZ] = p.q_gyro
        return np.diag(q * dt)

    def predict(self, dt):
        F = self._F(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self._Q(dt)
        return self.x[:2].copy()

    # ----------------------------------------------------------------- update
    def _update(self, z, H, R):
        z = np.asarray(z, dtype=float)
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(NSTATE) - K @ H) @ self.P

    def _row(self, idxs):
        H = np.zeros((len(idxs), NSTATE))
        for r, c in enumerate(idxs):
            H[r, c] = 1.0
        return H

    def update_camera(self, x_cm, y_cm):
        H = self._row([PX, PY])
        R = np.diag([self.p.r_cam, self.p.r_cam])
        self._update([x_cm, y_cm], H, R)

    def update_orientation(self, yaw_deg, pitch_deg, roll_deg):
        """Telemetry orientation. yaw is mapped magnetic-north -> field heading."""
        field_yaw = yaw_deg - self.p.body_to_field_yaw_offset
        H = self._row([YAW, PITCH, ROLL])
        R = np.diag([self.p.r_ori, self.p.r_ori, self.p.r_ori])
        self._update([field_yaw, pitch_deg, roll_deg], H, R)

    def update_velocity_body(self, vx_body_cms, vy_body_cms):
        """Telemetry body velocity (cm/s) rotated into field by the est. yaw."""
        fx, fy = rotate_body_to_field(vx_body_cms, vy_body_cms, self.x[YAW])
        H = self._row([VX, VY])
        R = np.diag([self.p.r_vel, self.p.r_vel])
        self._update([fx, fy], H, R)

    def update_accel_body_g(self, ax_g, ay_g, az_g):
        """Telemetry body accel (g) -> cm/s^2, planar rotated into field by yaw."""
        ax = ax_g * G_TO_CMS2
        ay = ay_g * G_TO_CMS2
        az = az_g * G_TO_CMS2
        fx, fy = rotate_body_to_field(ax, ay, self.x[YAW])
        H = self._row([AX, AY, AZ])
        R = np.diag([self.p.r_acc, self.p.r_acc, self.p.r_acc])
        self._update([fx, fy, az], H, R)

    def update_gyro(self, gx_dps, gy_dps, gz_dps):
        """Telemetry gyro (deg/s). gz drives the yaw integration term."""
        H = self._row([GX, GY, GZ])
        R = np.diag([self.p.r_gyro, self.p.r_gyro, self.p.r_gyro])
        self._update([gx_dps, gy_dps, gz_dps], H, R)

    # ---------------------------------------------------------------- getters
    @property
    def pos_cm(self):
        return float(self.x[PX]), float(self.x[PY])

    @property
    def vel_cms(self):
        return float(self.x[VX]), float(self.x[VY])

    @property
    def yaw_deg(self):
        return float(self.x[YAW])

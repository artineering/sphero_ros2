#!/usr/bin/env python3
"""EKF on blob position + LED heading, driven by the motion_cmd echo (pure, no ROS).

    state    x = [px, py, theta]   cm, cm, rad -- FIELD frame, theta CCW from +x
    input    u = [v, omega]        cm/s, rad/s -- from CommandHistory
    meas     z = [px, py, theta]   theta from the LED pair; [px, py] without it

Ported from scripts/kf_replay.py (the "omega lag 0 + 180 deg/s turn-rate limit"
variant tuned on SB-418F, recordings/kf_20261005_171405): A = df/dx,
P = A P A' + Q dt, S = H P H' + R, K = P H' S^-1, heading residual wrapped.

Unlike the prototype the filter can start from a position alone: until the
first LED fix theta is unknown (`theta_known` False), the speed input is not
applied (no direction to apply it in), the position process noise is the
larger `q_pos_no_heading`, and the node publishes identity orientation.

Sphero headings are clockwise and field angles counter-clockwise. The EKF only
uses CHANGES of the commanded heading, so the robot's aim offset cancels;
HeadingOffsetEstimator recovers the offset itself (field = offset - cmd).
"""

import math
from collections import deque

import numpy as np

PX, PY, TH = range(3)


def wrap(a):
    """Angle in rad to [-pi, pi)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def wrap_deg(a):
    """Angle in deg to [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


class HeadingEKF:

    # Q (per second) from covariance consistency, not blind-error minimisation
    # (2026-11-04, kf_20261005_171405 + motion_20261005_16*, blind predictions
    # of 1 frame..1 s from every 3rd frame): with a heading the 3-sigma gate
    # holds 98-99 % of the real 0.3-1 s errors at q_pos 100 (1000 put sigma at
    # ~4x the error). Without one the speed is not applied, so the random walk
    # carries the whole motion: q_pos_no_heading 500. q_th 0.3 brings the LED
    # heading NIS from 2.1 to 1.0. Estimates (0.5 s blind 4.4 cm) unchanged.
    # R: still blobs jitter far less (~0.0002 cm^2); 0.05 covers moving shells.
    def __init__(self, px_cm, py_cm, theta=None, q_pos=100.0, q_th=0.3,
                 r_pos=0.05, r_th=0.001, p0_pos=1.0, p0_th=0.5,
                 q_pos_no_heading=500.0):
        self.x = np.array([float(px_cm), float(py_cm),
                           0.0 if theta is None else wrap(float(theta))])
        self.P = np.diag([p0_pos, p0_pos, p0_th])
        self.q = np.array([q_pos, q_pos, q_th], dtype=float)
        self.q_no_heading = np.array([q_pos_no_heading, q_pos_no_heading, q_th],
                                     dtype=float)
        self.R = np.diag([r_pos, r_pos, r_th])
        self.p0_th = float(p0_th)
        self.theta_known = theta is not None
        self.v = 0.0                       # last speed input, for vel_cms

    def predict(self, dt, v, omega):
        if not self.theta_known:
            v = omega = 0.0
        self.v = float(v)
        th = self.x[TH]
        self.x = self.x + np.array([v * math.cos(th) * dt, v * math.sin(th) * dt,
                                    omega * dt])
        self.x[TH] = wrap(self.x[TH])
        A = np.array([[1, 0, -v * math.sin(th) * dt],
                      [0, 1, v * math.cos(th) * dt],
                      [0, 0, 1]])
        q = self.q if self.theta_known else self.q_no_heading
        self.P = A @ self.P @ A.T + np.diag(q * dt)
        return self.pos_cm

    def _innovation(self, z_xy, z_theta):
        m = 2 if z_theta is None else 3
        H = np.eye(3)[:m]
        z = list(z_xy) + ([] if z_theta is None else [z_theta])
        y = np.asarray(z, float) - H @ self.x
        if m == 3:
            y[2] = wrap(y[2])
        S = H @ self.P @ H.T + self.R[:m, :m]
        return H, y, S

    def mahalanobis2(self, z_xy, z_theta=None):
        """Squared Mahalanobis distance of a candidate measurement, without
        updating. theta is ignored until the first LED fix."""
        if not self.theta_known:
            z_theta = None
        _H, y, S = self._innovation(z_xy, z_theta)
        return float(y @ np.linalg.solve(S, y))

    def update(self, z_xy, z_theta=None):
        """Fuse a camera fix (cm, and rad if the LED pair was found).
        Returns (innovation y, NIS = y' S^-1 y)."""
        if z_theta is not None and not self.theta_known:
            # first LED fix: take it as the prior, as kf_replay starts from it
            self.x[TH] = wrap(float(z_theta))
            self.P[TH, :] = self.P[:, TH] = 0.0
            self.P[TH, TH] = self.p0_th
            self.theta_known = True
        H, y, S = self._innovation(z_xy, z_theta)
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[TH] = wrap(self.x[TH])
        self.P = (np.eye(3) - K @ H) @ self.P
        return y, float(y @ np.linalg.solve(S, y))

    @property
    def pos_cm(self):
        return float(self.x[PX]), float(self.x[PY])

    @property
    def pos_cov(self):
        """2x2 position covariance (cm^2), for a P-scaled gate."""
        return self.P[:2, :2].copy()

    @property
    def theta(self):
        """Field heading in rad, or None before the first LED fix."""
        return float(self.x[TH]) if self.theta_known else None

    @property
    def heading_deg(self):
        """Field heading in [0, 360), or None before the first LED fix."""
        return math.degrees(self.x[TH]) % 360.0 if self.theta_known else None

    @property
    def vel_cms(self):
        th = self.x[TH]
        return self.v * math.cos(th), self.v * math.sin(th)


class CommandHistory:
    """One robot's /sphero/<ns>/motion_cmd echoes -> EKF inputs.

    v is the speed in effect `lag_v_sec` ago (x speed_to_cms). omega is the
    change of commanded heading (no lag, tuned value), negated (Sphero headings
    are clockwise), spread over time at <= turn_rate_dps. Only deltas are used,
    so the aim offset cancels.

    inputs() is stateful (the turn still to do carries over): call it once per
    predict step with non-decreasing t.
    """

    def __init__(self, lag_v_sec=0.45, turn_rate_dps=180.0, speed_to_cms=0.61,
                 maxlen=256):
        self.lag_v = float(lag_v_sec)
        self.rate = math.radians(turn_rate_dps)
        self.speed_to_cms = float(speed_to_cms)
        self._cmds = deque(maxlen=maxlen)          # (t, speed, sphero_heading_deg)
        self._prev_h = None                        # field-sense heading, rad
        self._pending = 0.0                        # commanded turn still to do, rad
        self._t_last = None

    def add(self, t, speed, heading_deg):
        self._cmds.append((float(t), float(speed), float(heading_deg)))

    def _at(self, t):
        """(speed, heading_deg) of the last command at or before t, or None."""
        last = None
        for tc, s, hd in self._cmds:
            if tc > t:
                break
            last = (s, hd)
        return last

    def inputs(self, t):
        """(v cm/s, omega rad/s) for the predict step ending at t."""
        dt = t - self._t_last if self._t_last is not None else 0.0
        self._t_last = t
        c = self._at(t - self.lag_v)
        v = c[0] * self.speed_to_cms if c is not None else 0.0
        c = self._at(t)
        hc = math.radians(-c[1]) if c is not None else None
        if hc is not None and self._prev_h is not None:
            self._pending += wrap(hc - self._prev_h)
        if hc is not None:
            self._prev_h = hc
        w = 0.0
        if dt > 0:
            w = max(-self.rate, min(self.rate, self._pending / dt))
            self._pending -= w * dt
        # entries before the one in effect at t - lag are never read again
        while len(self._cmds) >= 2 and self._cmds[1][0] <= t - self.lag_v:
            self._cmds.popleft()
        return v, w


class HeadingOffsetEstimator:
    """Per-robot heading offset, degrees: field_heading = offset - cmd_heading,
    so a controller aims with cmd = offset - desired_field_heading.

    Samples wrap(theta_LED + cmd) only once the commanded heading has been
    unchanged for `settle_sec` (covers the ~1 s BLE latency). Circular mean of
    the first `n_init` samples, then a low-gain update (as goto_target.py).
    """

    def __init__(self, settle_sec=1.5, n_init=20, gain=0.02):
        self.settle = float(settle_sec)
        self.n_init = int(n_init)
        self.gain = float(gain)
        self.reset()

    def reset(self):
        """Forget everything (aim reset / calibration, BLE reconnect)."""
        self._cmd = None                   # Sphero heading, deg
        self._cmd_since = None
        self._init = []
        self._value = None                 # deg, [-180, 180)
        self._published = None

    def on_cmd(self, t, heading_deg):
        heading_deg = float(heading_deg) % 360.0
        if self._cmd is None or heading_deg != self._cmd:
            self._cmd, self._cmd_since = heading_deg, float(t)

    def add(self, t, led_heading_deg):
        """Offer an LED heading (field deg) seen at t. Returns True if sampled."""
        if led_heading_deg is None or self._cmd is None \
                or t - self._cmd_since < self.settle:
            return False
        s = wrap_deg(led_heading_deg + self._cmd)
        if self._value is not None:
            self._value = wrap_deg(self._value + self.gain * wrap_deg(s - self._value))
            return True
        self._init.append(math.radians(s))
        if len(self._init) >= self.n_init:
            self._value = math.degrees(math.atan2(np.mean(np.sin(self._init)),
                                                  np.mean(np.cos(self._init))))
            self._init = []
        return True

    @property
    def value(self):
        """Offset in [0, 360), or None until estimated."""
        return None if self._value is None else self._value % 360.0

    def changed_since_publish(self, threshold_deg=1.0):
        """True if there is a value and it was never published or has moved by
        more than threshold_deg since mark_published()."""
        if self._value is None:
            return False
        return self._published is None or \
            abs(wrap_deg(self._value - self._published)) > threshold_deg

    def mark_published(self):
        self._published = self._value

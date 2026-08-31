#!/usr/bin/env python3
"""Kalman filter driven by the roll command (pure, no ROS).

    state    x = [px, py, v, theta]   cm, cm, cm/s, deg -- FIELD frame, CCW from +x
    control  u = (v_cmd, theta_cmd)   the last roll command, in field terms
    meas     z = [x_cm, y_cm]         template match through the homography

The command supplies speed and heading outright, so position integrates the
COMMANDED velocity instead of waiting for successive camera fixes to reveal that
the robot moved. F is identity on position and zero on (v, theta): those two rows
come entirely from the control.
"""

import numpy as np

PX, PY, V, TH = range(4)
NSTATE = 4


class MotionKF:

    def __init__(self, px_cm, py_cm, q_pos, q_vel, q_yaw, r_cam, p0=100.0):
        self.x = np.zeros(NSTATE)
        self.x[PX], self.x[PY] = float(px_cm), float(py_cm)
        self.P = np.eye(NSTATE) * float(p0)
        self.q = np.array([q_pos, q_pos, q_vel, q_yaw], dtype=float)
        self.r_cam = float(r_cam)

    def predict(self, dt, v_cmd, theta_cmd_deg):
        th = np.deg2rad(theta_cmd_deg)
        F = np.zeros((NSTATE, NSTATE))
        F[PX, PX] = F[PY, PY] = 1.0
        bu = np.array([v_cmd * np.cos(th) * dt,
                       v_cmd * np.sin(th) * dt,
                       v_cmd,
                       float(theta_cmd_deg)])
        self.x = F @ self.x + bu
        self.P = F @ self.P @ F.T + np.diag(self.q * dt)
        return self.pos_cm

    def update_camera(self, x_cm, y_cm):
        H = np.zeros((2, NSTATE))
        H[0, PX] = H[1, PY] = 1.0
        R = np.diag([self.r_cam, self.r_cam])
        y = np.array([float(x_cm), float(y_cm)]) - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(NSTATE) - K @ H) @ self.P

    @property
    def pos_cm(self):
        return float(self.x[PX]), float(self.x[PY])

    @property
    def vel_cms(self):
        th = np.deg2rad(self.x[TH])
        return (float(self.x[V] * np.cos(th)), float(self.x[V] * np.sin(th)))

    @property
    def heading_deg(self):
        return float(self.x[TH] % 360.0)

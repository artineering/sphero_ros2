#!/usr/bin/env python3
"""Forward-predicted ROI geometry (pure, no ROS).

Each tracked robot claims a region around where the Kalman filter says it will
be, sized at `roi_scale` x the ball diameter plus the distance it can travel in
one frame interval. On a miss the region grows geometrically, so a robot that
reappears after a brief occlusion is still inside its own window.

The velocity used here comes from the KF, which is fed by per-robot TELEMETRY --
the one signal the camera cannot supply. That is what makes the prediction
trustworthy while the robot is invisible.
"""

import math


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def roi_half(ball_d_px, scale, vel_px_per_s, dt, miss_count,
             growth=1.5, lo=30, hi=160):
    """Half-edge of the claim region, in pixels."""
    base = 0.5 * float(scale) * float(ball_d_px)
    travel = math.ceil(abs(float(vel_px_per_s)) * max(float(dt), 0.0))
    grown = (base + travel) * (float(growth) ** max(int(miss_count), 0))
    return int(clamp(grown, lo, hi))


def roi_from_center(cx, cy, half, width, height):
    """Axis-aligned rect clamped to the frame.

    Returns (x0, y0, x1, y1, clamped) with clamped True when the frame border cut
    the region -- the caller needs that to know the window may be too small to
    match in.
    """
    h = int(half)
    x0, y0 = int(round(cx)) - h, int(round(cy)) - h
    x1, y1 = int(round(cx)) + h, int(round(cy)) + h
    cx0, cy0 = max(0, x0), max(0, y0)
    cx1, cy1 = min(int(width), x1), min(int(height), y1)
    clamped = (cx0, cy0, cx1, cy1) != (x0, y0, x1, y1)
    return cx0, cy0, cx1, cy1, clamped


def degenerate(rect, tmpl_size, pad=4):
    """True if the (clamped) rect cannot hold the template plus its search pad.

    A robot at the frame edge produces one of these; the correct response is a
    clean miss, not a garbage match against a truncated window.
    """
    x0, y0, x1, y1 = rect[:4]
    need = int(tmpl_size) + 2 * int(pad)
    return (x1 - x0) < need or (y1 - y0) < need


def contains(rect, u, v):
    x0, y0, x1, y1 = rect[:4]
    return x0 <= u < x1 and y0 <= v < y1


def predict_center_px(x_cm, y_cm, vx_cms, vy_cms, dt, h_inv, apply_fn):
    """Where the robot will be next frame, in pixels.

    Extrapolates in FIELD cm (where the constant-velocity model is valid) and
    only then maps to pixels. Doing it the other way -- extrapolating a pixel
    velocity -- is wrong under perspective, because px/cm varies across the frame.
    """
    nx = float(x_cm) + float(vx_cms) * float(dt)
    ny = float(y_cm) + float(vy_cms) * float(dt)
    return apply_fn(nx, ny, h_inv)

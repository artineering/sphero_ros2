#!/usr/bin/env python3
"""Greedy nearest-neighbour data association (pure, no ROS).

Used ONLY for re-acquiring LOST tracks against leftover candidates. Steady-state
tracking does not go through here -- each robot claims a candidate inside its own
forward-predicted ROI, which is both cheaper and far less prone to identity
swaps than a global assignment.

The tracker set is FIXED at link time; association never spawns a new track.
"""

import numpy as np


def associate(pred_xy, meas_xy, gate):
    """Greedy nearest-neighbour association under a distance gate.

    Parameters
    ----------
    pred_xy : (Np, 2) predicted tracker positions.
    meas_xy : (Nm, 2) measurement positions.
    gate : max match distance (same units as the coords).

    Returns (matches, unmatched_pred, unmatched_meas) where matches is a list of
    (pred_index, meas_index) pairs, and the unmatched_* are sets of indices.
    """
    preds = (np.asarray(pred_xy, dtype=float).reshape(-1, 2)
             if len(pred_xy) else np.empty((0, 2)))
    meas = (np.asarray(meas_xy, dtype=float).reshape(-1, 2)
            if len(meas_xy) else np.empty((0, 2)))

    matches = []
    unmatched_p = set(range(len(preds)))
    unmatched_m = set(range(len(meas)))
    if len(preds) and len(meas):
        D = np.linalg.norm(preds[:, None, :] - meas[None, :, :], axis=2)
        while unmatched_p and unmatched_m:
            i, j = np.unravel_index(np.argmin(D), D.shape)
            if D[i, j] > gate:
                break
            matches.append((int(i), int(j)))
            D[i, :] = np.inf
            D[:, j] = np.inf
            unmatched_p.discard(int(i))
            unmatched_m.discard(int(j))
    return matches, unmatched_p, unmatched_m

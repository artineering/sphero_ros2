#!/usr/bin/env python3
"""Greedy nearest-neighbour data association (ported, self-contained).

PORTED BY COPY from aruco_slam/kinect_tracking.py (the _associate routine). No
runtime dependency on aruco_slam. Unlike the old anonymous Tracker, the field
tracker associates measurements to a FIXED set of locked callsign trackers
(registration creates them; tracking never spawns new ones), so only the pure
association primitive is needed here.
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
    preds = np.asarray(pred_xy, dtype=float).reshape(-1, 2) if len(pred_xy) else np.empty((0, 2))
    meas = np.asarray(meas_xy, dtype=float).reshape(-1, 2) if len(meas_xy) else np.empty((0, 2))

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

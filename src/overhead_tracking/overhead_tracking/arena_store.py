#!/usr/bin/env python3
"""Arena calibration persistence (pure, no ROS).

Stores the homography and everything needed to reproduce or audit it. Written
OUTSIDE the colcon install tree -- `colcon build` wipes `install/`, so a
calibration kept there is silently lost on the next build.

The camera settings are stored alongside deliberately: a homography is only valid
for the geometry it was captured under, and recording exposure/gain/resolution
makes it obvious when a stored arena predates a camera change.
"""

import os
from dataclasses import dataclass, field

import numpy as np
import yaml


@dataclass
class ArenaCalibration:
    corners_px: list                       # [[u, v] x 4]
    corners_field_cm: list                 # [[x, y] x 4], index-aligned
    H: list                                # 3x3 row-major, px -> cm
    long_edge_len_cm: float
    short_edge_len_cm: float
    frame_width: int = 0
    frame_height: int = 0
    residual_max_cm: float = 0.0
    residual_mean_cm: float = 0.0
    frame_parent: str = 'field'
    frame_child: str = 'overhead_camera'
    camera_settings: dict = field(default_factory=dict)
    created: str = ''
    source: str = 'manual'                 # 'manual' | 'auto'

    @property
    def H_np(self):
        return np.asarray(self.H, dtype=float).reshape(3, 3)

    @property
    def H_inv_np(self):
        return np.linalg.inv(self.H_np)

    @property
    def corners_px_np(self):
        return np.asarray(self.corners_px, dtype=float).reshape(-1, 2)

    @property
    def corners_cm_np(self):
        return np.asarray(self.corners_field_cm, dtype=float).reshape(-1, 2)

    def to_yaml_dict(self):
        return {
            'corners_px': [[float(a), float(b)] for a, b in self.corners_px],
            'corners_field_cm': [[float(a), float(b)]
                                 for a, b in self.corners_field_cm],
            'H': [[float(v) for v in row]
                  for row in np.asarray(self.H, dtype=float).reshape(3, 3)],
            'long_edge_len_cm': float(self.long_edge_len_cm),
            'short_edge_len_cm': float(self.short_edge_len_cm),
            'frame_width': int(self.frame_width),
            'frame_height': int(self.frame_height),
            'residual_max_cm': float(self.residual_max_cm),
            'residual_mean_cm': float(self.residual_mean_cm),
            'frame_parent': str(self.frame_parent),
            'frame_child': str(self.frame_child),
            'camera_settings': dict(self.camera_settings or {}),
            'created': str(self.created),
            'source': str(self.source),
        }

    @staticmethod
    def from_yaml_dict(d):
        return ArenaCalibration(
            corners_px=[list(map(float, p)) for p in d['corners_px']],
            corners_field_cm=[list(map(float, p)) for p in d['corners_field_cm']],
            H=[list(map(float, r)) for r in d['H']],
            long_edge_len_cm=float(d['long_edge_len_cm']),
            short_edge_len_cm=float(d['short_edge_len_cm']),
            frame_width=int(d.get('frame_width', 0)),
            frame_height=int(d.get('frame_height', 0)),
            residual_max_cm=float(d.get('residual_max_cm', 0.0)),
            residual_mean_cm=float(d.get('residual_mean_cm', 0.0)),
            frame_parent=str(d.get('frame_parent', 'field')),
            frame_child=str(d.get('frame_child', 'overhead_camera')),
            camera_settings=dict(d.get('camera_settings') or {}),
            created=str(d.get('created', '')),
            source=str(d.get('source', 'manual')),
        )


def save_arena(path, cal):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as fh:
        yaml.safe_dump(cal.to_yaml_dict(), fh, default_flow_style=False,
                       sort_keys=False)
    return path


def load_arena(path):
    with open(path, 'r') as fh:
        return ArenaCalibration.from_yaml_dict(yaml.safe_load(fh))

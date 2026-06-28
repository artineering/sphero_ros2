#!/usr/bin/env python3
"""Field-calibration persistence (pure: YAML + npy I/O, no ROS).

Schema for kinect_field.yaml is defined by CalibrationResult.to_yaml_dict /
from_yaml_dict. The empty-arena heightmap is a separate .npy (per-pixel, 640x480
float32) captured by the ~/capture_baseline service.
"""

import os
from dataclasses import dataclass, field
from datetime import datetime, timezone

import numpy as np
import yaml


@dataclass
class CalibrationResult:
    intrinsics: dict                       # {fx, fy, cx, cy}
    corners_px: list                       # [[u,v] x4]
    corners_cam_mm: list                   # [[x,y,z] x4]
    corners_field_cm: list                 # [[x,y] x4]
    plane_n: list                          # [a,b,c]
    plane_d: float
    T_field_from_cam: list                 # 4x4 row-major
    origin_corner_index: int
    long_edge_len_cm: float
    short_edge_len_cm: float
    created: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    frame_parent: str = 'field'
    frame_child: str = 'kinect_overhead'

    def to_yaml_dict(self):
        return {
            'field_calibration': {
                'created': self.created,
                'frame_parent': self.frame_parent,
                'frame_child': self.frame_child,
                'intrinsics': self.intrinsics,
                'corners_px': self.corners_px,
                'corners_cam_mm': self.corners_cam_mm,
                'corners_field_cm': self.corners_field_cm,
                'origin_corner_index': int(self.origin_corner_index),
                'long_edge_len_cm': float(self.long_edge_len_cm),
                'short_edge_len_cm': float(self.short_edge_len_cm),
                'plane': {'n': self.plane_n, 'd': float(self.plane_d)},
                'T_field_from_cam': self.T_field_from_cam,
            }
        }

    @classmethod
    def from_yaml_dict(cls, d):
        fc = d['field_calibration']
        return cls(
            intrinsics=fc['intrinsics'],
            corners_px=fc['corners_px'],
            corners_cam_mm=fc['corners_cam_mm'],
            corners_field_cm=fc['corners_field_cm'],
            plane_n=fc['plane']['n'],
            plane_d=fc['plane']['d'],
            T_field_from_cam=fc['T_field_from_cam'],
            origin_corner_index=fc['origin_corner_index'],
            long_edge_len_cm=fc['long_edge_len_cm'],
            short_edge_len_cm=fc['short_edge_len_cm'],
            created=fc.get('created', ''),
            frame_parent=fc.get('frame_parent', 'field'),
            frame_child=fc.get('frame_child', 'kinect_overhead'),
        )

    @property
    def T_field_from_cam_np(self):
        return np.asarray(self.T_field_from_cam, dtype=float).reshape(4, 4)


def save_calibration(path, result: CalibrationResult):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as f:
        yaml.safe_dump(result.to_yaml_dict(), f, default_flow_style=False, sort_keys=False)


def load_calibration(path):
    with open(path, 'r') as f:
        d = yaml.safe_load(f)
    return CalibrationResult.from_yaml_dict(d)


def save_heightmap(path, heightmap):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    np.save(path, np.asarray(heightmap, dtype=np.float32))


def load_heightmap(path):
    return np.load(path).astype(np.float32)

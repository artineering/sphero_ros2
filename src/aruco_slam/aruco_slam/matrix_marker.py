#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Active LED-matrix marker definitions and assignment.

Identity = (matrix hue in 8) x (matrix fill in {filled, ring}) = 16 markers.
Both primitives are rotation-invariant (a solid hue and a concentric
filled-vs-ring), suiting bare Spheros that yaw freely.

This module owns:
- the fixed 8-hue palette (name -> RGB),
- the ordered 16-slot marker pool (filled across all hues first, then ring),
- marker_to_matrix_cmd(): builds the sphero/<name>/matrix JSON,
- assign_markers(): publishes each robot's marker.

The matrix JSON message format mirrors the existing matrix topic:
  {pattern, matrix[64], red, green, blue, duration}
consumed by sphero_instance_device_controller_node.matrix_callback ->
Sphero.set_matrix(...). Filled = [1]*64; ring = the 'circle' pattern from
sphero_instance_controller core/sphero/matrix_patterns.py.
"""

import json
from typing import Callable, Dict, List, Tuple

# Fixed 8-hue palette (name -> RGB), per the Interface Contract.
# Chosen for emissive separability; White / near-off avoided (bloom washout).
HUE_PALETTE: Dict[str, Tuple[int, int, int]] = {
    "Red": (255, 0, 0),
    "Orange": (255, 80, 0),
    "Yellow": (255, 255, 0),
    "Green": (0, 255, 0),
    "Cyan": (0, 255, 255),
    "Blue": (0, 0, 255),
    "Magenta": (255, 0, 255),
    "Purple": (140, 0, 255),
}

# Ordered hue list (defines marker-pool ordering).
HUE_ORDER: List[str] = [
    "Red", "Orange", "Yellow", "Green",
    "Cyan", "Blue", "Magenta", "Purple",
]

FILLS: List[str] = ["filled", "ring"]

# 'ring' uses the existing 'circle' pattern from matrix_patterns.py.
RING_PATTERN_NAME = "circle"

# Solid fill = all 64 cells on.
FILLED_MATRIX: List[int] = [1] * 64


def marker_pool() -> List[Tuple[str, str]]:
    """Return the ordered 16-slot marker pool as (hue, fill) tuples.

    Allocation order (per contract): filled across all 8 hues first
    (slots 0-7), then ring across all 8 hues (slots 8-15). Maximizes hue
    diversity for small fleets.
    """
    return [(hue, fill) for fill in FILLS for hue in HUE_ORDER]


def marker_to_matrix_cmd(hue: str, fill: str, brightness_scale: float = 0.4) -> str:
    """Build the sphero/<name>/matrix JSON string for a (hue, fill) marker.

    filled -> custom matrix [1]*64 at the hue RGB.
    ring   -> 'circle' pattern at the hue RGB.

    brightness_scale dims the palette RGB so emissive markers do not bloom to
    white at the camera's moderate exposure (hue is preserved).
    """
    if hue not in HUE_PALETTE:
        raise ValueError(f"Unknown hue '{hue}'")
    if fill not in FILLS:
        raise ValueError(f"Unknown fill '{fill}'")

    red, green, blue = HUE_PALETTE[hue]
    red = max(0, min(255, int(red * brightness_scale)))
    green = max(0, min(255, int(green * brightness_scale)))
    blue = max(0, min(255, int(blue * brightness_scale)))

    if fill == "filled":
        cmd = {
            "pattern": "",
            "matrix": FILLED_MATRIX,
            "red": red,
            "green": green,
            "blue": blue,
            "duration": 0,
        }
    else:  # ring
        cmd = {
            "pattern": RING_PATTERN_NAME,
            "matrix": [],
            "red": red,
            "green": green,
            "blue": blue,
            "duration": 0,
        }

    return json.dumps(cmd)


def assign_markers(
    name_to_marker: Dict[str, Tuple[str, str]],
    publisher_fn: Callable[[str, str], None],
    brightness_scale: float = 0.4,
) -> None:
    """Publish each robot's marker command.

    Args:
        name_to_marker: map robot name -> (hue, fill).
        publisher_fn: callable (name, matrix_cmd_json) -> None that publishes
            the matrix command on that robot's matrix topic.
        brightness_scale: dims the palette RGB to preserve hue (no bloom).
    """
    for name, (hue, fill) in name_to_marker.items():
        publisher_fn(name, marker_to_matrix_cmd(hue, fill, brightness_scale))
